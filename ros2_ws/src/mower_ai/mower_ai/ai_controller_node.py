# /home/sy/ros2_ws/src/mower_ai/mower_ai/ai_controller_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import torch
import os
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# --- 파일 임포트 ---
from .rl.network import ActorCritic 
from .planning.cpp import CoveragePlanner, HeuristicType

# --- ROS 메시지 ---
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
# [확인] 리스트 메시지 타입 임포트 필수
from mower_msgs.msg import DetectedObject, DetectedObjectList 
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R

class AIParams:
    R_GOAL_M = 5.0
    R_OBJ_M = 5.0
    R_RAY_M = 6.0  
    T_CAP_S = 5.0
    VMAX_OBJ = 1.5
    RAY_COUNT = 64
    max_objs = 3
    
    GLOBAL_WIDTH_M = 20.0
    GLOBAL_HEIGHT_M = 20.0

    # 사람 감지 관련 파라미터
    HUMAN_CONFIRM_TIME = 0.5  # 0.5초 이상 감지되어야 PPO 진입
    HUMAN_CLEAR_TIME = 3.0    # 사람이 사라지고 3초 뒤에 복귀
    HUMAN_DETECT_DIST = 3.0   # 3m 이내일 때만 반응

    ACTION_MAP = {
        0: (0.5, 0.0),   # 전진
        1: (0.0, 0.8),   # 좌회전
        2: (-0.6, 0.0),  # 후진
        3: (0.0, -0.8),  # 우회전
        4: (0.0, 0.0),   # 정지
    }
    DANGER_M = 2.0 

class State:
    WAITING_FOR_MAP = 0 
    PLANNING = 1        
    EXECUTING = 2       
    FINISHED = 3        
    BACKING_UP = 4  
    WIGGLING = 5  
    PPO_HUMAN_AVOID = 6 # 사람 회피 모드

class AiControllerNode(Node):
    def __init__(self):
        super().__init__('ai_controller_node')
        self.get_logger().info("AI Controller: Initializing...")
        
        self.declare_parameter('map_width', 20.0)
        self.declare_parameter('map_height', 20.0)
        self.current_state = State.WAITING_FOR_MAP
        self.params = AIParams()
        self.ppo_active_timer = 0
        
        # --- PPO 모델 로드 ---
        self.ppo_model = None
        try:
            self.ppo_model = ActorCritic(obs_dim=100, act_dim=5)
            package_dir = get_package_share_directory('mower_ai')
            model_path = os.path.join(package_dir, 'models', 'best_ever.pt')
            if os.path.exists(model_path):
                self.ppo_model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
                self.ppo_model.eval()
                self.get_logger().info(f"PPO Loaded: {model_path}")
            else:
                self.get_logger().warn(f"Model not found at {model_path}. PPO will act random.")
        except Exception as e:
            self.get_logger().error(f"PPO Load Error: {e}")

        # --- 데이터 변수 ---
        self.latest_scan_data = np.ones(self.params.RAY_COUNT, dtype=np.float32) * self.params.R_RAY_M
        self.current_pose_xy = np.array([0.0, 0.0])
        self.current_pose_yaw = 0.0
        self.latest_obstacle_data = None # 단일 객체 저장용 (기존 로직 호환)
        self.latest_human_data = None

        self.visited_history = [] 
        self.last_record_pos = np.array([999.0, 999.0])

        self.current_linear_val = 0.0
        self.current_angular_val = 0.0

        self.steps_after_planning = 0
        self.backup_timer = 0
        self.last_wp_idx = -1
        self.wp_stuck_timer = 0

        self.planning_fail_count = 0
        
        # --- 사람 감지 타이머 ---
        self.human_detect_timer = 0.0
        self.human_clear_timer = 0.0
        
        # --- 경로 관련 ---
        self.global_path = [] 
        self.wp_idx = 0
        
        self.map_data = None
        self.map_info = None
        self.GRID_SIZE_M = 0.8 
        
        # 액션 락(Lock)을 위한 타이머와 저장소
        self.action_lock_timer = 0  # 이 값이 0보다 크면 AI 판단을 생략하고 이전 행동 반복
        self.locked_action = 4      # 저장된 행동 (기본 정지)

        # --- ROS 통신 ---
        qos_map = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
 
        # 카메라 데이터를 받는 구독(Subscription) 추가
        self.create_subscription(
            DetectedObjectList,          # 리스트 타입 메시지
            '/detected_obstacle',        # Unity와 동일한 토픽 이름
            self.human_detection_callback, # 연결할 콜백 함수
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, '/mower_path', 10)
        
        self.create_timer(0.1, self.main_loop)
        
        self.get_logger().info("Waiting for /map topic...")

    def odom_callback(self, msg: Odometry):
        self.current_pose_xy[0] = msg.pose.pose.position.x
        self.current_pose_xy[1] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_pose_yaw = np.arctan2(siny_cosp, cosy_cosp)

    # 사람 감지 전용으로 사용됩니다.
    # 기존 장애물 데이터(self.latest_obstacle_data) 업데이트는 여기서 하지 않거나
    # 가장 가까운 객체로 업데이트합니다.
    def human_detection_callback(self, msgs):
        # [디버깅] 데이터 수신 확인용 로그 (1초에 한 번만 출력)
        if len(msgs.objects) > 0:
            self.get_logger().info(f"Vision Recv: {len(msgs.objects)} objs", throttle_duration_sec=1.0)

        detected = False
        min_dist = 99.9
        
        # 1. 사람 감지 여부 판단 (타이머 업데이트용)
        for obj in msgs.objects:
            if obj.label == "person":
                if obj.distance < self.params.HUMAN_DETECT_DIST:
                    if abs(obj.angle) < 60.0:
                        detected = True
                        min_dist = min(min_dist, obj.distance)

        # 2. 타이머 로직 업데이트
        if detected:
            self.human_detect_timer += 0.1 
            self.human_clear_timer = 0.0   
        else:
            self.human_detect_timer = 0.0  
            self.human_clear_timer += 0.1  
        
        # 3. 기존 주행 로직 호환성을 위해 가장 가까운 객체 하나를 저장 
        #  감지된 경우, 가장 가까운 사람 정보를 '전용 변수'에 저장
        if len(msgs.objects) > 0:
             # 사람 라벨인 것 중 가장 가까운 것 찾기
             people = [obj for obj in msgs.objects if obj.label == "person"]
             if people:
                 self.latest_human_data = min(people, key=lambda x: x.distance)
             else:
                 self.latest_human_data = None
        else:
             self.latest_human_data = None

    def scan_callback(self, msg: LaserScan):
        if len(msg.ranges) == 0: return
        raw = np.array(msg.ranges, dtype=np.float32)
        raw[np.isinf(raw)] = self.params.R_RAY_M
        raw = np.clip(raw, 0.0, self.params.R_RAY_M)
        if len(raw) == self.params.RAY_COUNT:
            self.latest_scan_data = raw
        else:
            indices = np.linspace(0, len(raw)-1, self.params.RAY_COUNT).astype(int)
            self.latest_scan_data = raw[indices]

    def map_callback(self, msg: OccupancyGrid):
        if msg.info.width == 0 or msg.info.height == 0: return
        self.map_data = msg.data
        self.map_info = msg.info
        if self.current_state == State.WAITING_FOR_MAP:
            self.get_logger().info(f"Initial Map Received! Size: {msg.info.width}x{msg.info.height}")
            self.current_state = State.PLANNING

    def generate_path_from_map(self):
        self.get_logger().info("Start Path Planning...")
        if self.map_info is None:
            self.get_logger().warn("No Map Info yet.")
            return False
        
        slam_res = self.map_info.resolution
        slam_w = self.map_info.width
        slam_h = self.map_info.height
        slam_ox = self.map_info.origin.position.x
        slam_oy = self.map_info.origin.position.y
        
        raw_map = np.array(self.map_data).reshape(slam_h, slam_w)
        global_w = self.get_parameter('map_width').value
        global_h = self.get_parameter('map_height').value
        grid_res = self.GRID_SIZE_M 
        
        # 안전한 시작점 설정 (벽 끼임 방지)
        fixed_ox = 0.0  
        fixed_oy = -20.0 
        
        new_w = int(global_w / grid_res)
        new_h = int(global_h / grid_res)
        
        planner_grid = np.zeros((new_h, new_w), dtype=int)
        
        # 테두리 벽 설정
        planner_grid[0, :] = 1
        planner_grid[-1, :] = 1
        planner_grid[:, 0] = 1
        planner_grid[:, -1] = 1

        for r in range(new_h):
            for c in range(new_w):
                cell_min_x = fixed_ox + (c * grid_res)
                cell_max_x = cell_min_x + grid_res
                cell_min_y = fixed_oy + (r * grid_res)
                cell_max_y = cell_min_y + grid_res
                
                slam_c_min = int((cell_min_x - slam_ox) / slam_res)
                slam_c_max = int((cell_max_x - slam_ox) / slam_res)
                slam_r_min = int((cell_min_y - slam_oy) / slam_res)
                slam_r_max = int((cell_max_y - slam_oy) / slam_res)
                
                slam_c_min = max(0, slam_c_min); slam_c_max = min(slam_w, slam_c_max)
                slam_r_min = max(0, slam_r_min); slam_r_max = min(slam_h, slam_r_max)
                
                if slam_c_min >= slam_c_max or slam_r_min >= slam_r_max: continue
                
                chunk = raw_map[slam_r_min:slam_r_max, slam_c_min:slam_c_max]
                obstacle_count = np.count_nonzero(chunk > 50)
                
                if obstacle_count > 12:
                    planner_grid[r, c] = 1

        start_c = int((self.current_pose_xy[0] - fixed_ox) / grid_res)
        start_r = int((self.current_pose_xy[1] - fixed_oy) / grid_res)
        start_c = np.clip(start_c, 0, new_w-1)
        start_r = np.clip(start_r, 0, new_h-1)

        if planner_grid[start_r, start_c] == 1:
            self.get_logger().warn("Start pos in Obstacle! Searching nearby...")
            found_free = False
            search_range = 3  
            for r_off in range(-search_range, search_range + 1):
                for c_off in range(-search_range, search_range + 1):
                    nr, nc = start_r + r_off, start_c + c_off
                    if 0 <= nr < new_h and 0 <= nc < new_w and planner_grid[nr, nc] == 0:
                        start_r, start_c = nr, nc
                        found_free = True
                        break
                if found_free: break
            
            if found_free:
                self.get_logger().info(f"Moved Start to: ({start_r}, {start_c})")
            else:
                self.get_logger().error("CRITICAL: Stuck in walls!")
                return False

        # [강제 초기화] 시작점은 무조건 빈 공간으로 설정
        if 0 <= start_r < new_h and 0 <= start_c < new_w:
            planner_grid[start_r, start_c] = 0 
            self.get_logger().info(f"Forced Start Node ({start_r}, {start_c}) to be FREE.")

        try:
            planner = CoveragePlanner(planner_grid)
            planner.start(initial_orientation=0)
            planner.current_pos = [start_r, start_c, 0]

            visited_mask = np.zeros((new_h, new_w), dtype=bool)
            for pos in self.visited_history:
                vc = int((pos[0] - fixed_ox) / grid_res) 
                vr = int((pos[1] - fixed_oy) / grid_res)
                if 0 <= vr < new_h and 0 <= vc < new_w:
                    visited_mask[vr, vc] = True
            
            visited_mask[start_r, start_c] = False 
            planner.visited = visited_mask
            
            planner.compute()
            result = planner.result() 
            
            if not result[0] or len(result[4]) == 0:
                self.get_logger().error("Planning Failed! No path found")
                return False
            
            path_indices = result[4] 
            self.global_path = []
            for (r, c) in path_indices:
                wx = (c * grid_res) + fixed_ox + (grid_res/2)
                wy = (r * grid_res) + fixed_oy + (grid_res/2)
                self.global_path.append([float(wx), float(wy)])

            self.get_logger().info(f"Global Path Generated! Points: {len(self.global_path)}")
            self.publish_path()
            
            if not self.global_path: return False
            
            self.wp_idx = min(3, len(self.global_path) - 1)
            self.steps_after_planning = 0
            self.wp_stuck_timer = 0
            self.last_wp_idx = -1
            return True
        except Exception as e:
            self.get_logger().error(f"Planner Error: {e}")
            return False

    def build_state_for_ppo(self):

        # PPO 모드일 때는 목표점(Target)을 내 위치 기준으로 실시간 갱신해야 함
        # 그래야 사람을 피하면서도 '경로 쪽으로' 움직이려고 노력함
        
        current_target_idx = self.wp_idx

        # 만약 PPO 모드이고, 기존 경로가 남아있다면?
        if self.current_state == State.PPO_HUMAN_AVOID and self.global_path:
            #  내 현재 위치에서 가장 가까운 경로점 찾기 (수학적 계산)
            path_arr = np.array(self.global_path)
            dists = np.linalg.norm(path_arr - self.current_pose_xy, axis=1)
            nearest_idx = np.argmin(dists)
            
            # 그 점보다 조금 앞(Look Ahead)을 목표로 설정
            # 너무 가까운 점을 찍으면 제자리에서 돔, 3~5칸 앞을 보게 함
            look_ahead_step = 5 
            current_target_idx = min(nearest_idx + look_ahead_step, len(self.global_path) - 1)
            
            #  디버깅 AI가 어디를 목표로 삼았는지 출력
            self.get_logger().info(f"PPO Dynamic Target: WP {current_target_idx}", throttle_duration_sec=2.0)

        # 1. Goal Info (3)
        target = self.global_path[current_target_idx] if current_target_idx < len(self.global_path) else self.global_path[-1]
        dx = target[0] - self.current_pose_xy[0]
        dy = target[1] - self.current_pose_xy[1]
        dist = np.hypot(dx, dy)
        angle = np.arctan2(dy, dx) - self.current_pose_yaw
        
        # 각도 정규화 (-PI ~ PI)
        while angle > np.pi: angle -= 2 * np.pi
        while angle < -np.pi: angle += 2 * np.pi
        
        goal_feats = [min(dist, self.params.R_GOAL_M) / self.params.R_GOAL_M, 
                      np.cos(angle), 
                      np.sin(angle)
                    ]
        # 2. Obstacle Info (15)
        obj_feats = []
        # [논리적 수정] 모드에 따라 AI에게 주입할 데이터를 결정
        target_obs = None
        if self.current_state == State.PPO_HUMAN_AVOID:
            # 사람 회피 모드면 -> 카메라 데이터(human_data)를 최우선으로 사용
            if self.latest_human_data:
                target_obs = self.latest_human_data
            else:
                target_obs = self.latest_obstacle_data # 없으면 기존 데이터라도
        else:
            # 평상시 -> 기존 장애물 데이터 사용
            target_obs = self.latest_obstacle_data

        # 선택된 데이터로 특징 벡터 생성
        if target_obs and target_obs.detected:
             # 거리 정규화
             d = min(target_obs.distance, self.params.R_OBJ_M) / self.params.R_OBJ_M
             # 각도
             ang = np.deg2rad(target_obs.angle)
             #  vx, vy를 0.5로 설정 (정지한 물체로 가정하되 존재감 부각)
             obj_feats.extend([d, np.cos(ang), np.sin(ang), 0.5, 0.5]) 
        
        # 패딩 채우기 (기존 동일)
        while len(obj_feats) < 15:
            obj_feats.extend([1.0, 0.0, 0.0, 0.0, 1.0])


        # 3. Lidar (64)
        lidar_feats = np.clip(self.latest_scan_data / self.params.R_RAY_M, 0.0, 1.0)
        # Danger Scalar (2개: 현재 위험도, 주변 위험도)
        danger_scalar = np.zeros(2, dtype=np.float32)
        # Danger Lidar (16개)
        danger_lidar = np.zeros(16, dtype=np.float32)
        # 최종 결합 (총 100차원)
        return np.concatenate([goal_feats,      # 3
                               obj_feats,       # 15
                               lidar_feats,     # 64
                               danger_scalar,   # 2
                               danger_lidar     # 16
                               ], dtype=np.float32)
    
    def main_loop(self):
        twist = Twist()
        
        # ------------------------------------------------------------------
        # [1] 전역 상태 전이 체크 (어떤 상태에서든 사람 감지되면 납치)
        # ------------------------------------------------------------------
        if self.current_state != State.PPO_HUMAN_AVOID:
            # 콜백에서 업데이트된 타이머를 확인
            if self.human_detect_timer >= self.params.HUMAN_CONFIRM_TIME:
                self.get_logger().warn("🚨 HUMAN DETECTED! Switching to PPO.")
                self.current_state = State.PPO_HUMAN_AVOID
                self.human_detect_timer = 0
                self.human_clear_timer = 0
    
        # --- 상태 머신 ---
        # 지도 올 때까지 정지
        if self.current_state == State.WAITING_FOR_MAP:
            pass
            
        elif self.current_state == State.PLANNING:
            # 경로 생성 시도
            success = self.generate_path_from_map()
            if success:
                self.planning_fail_count = 0
                self.current_state = State.EXECUTING
                self.steps_after_planning = 0
                self.wp_stuck_timer = 0
                self.last_wp_idx = -1
                self.get_logger().info(f">>> Timer RESET. Starting from WP {self.wp_idx}")
            else:
                self.planning_fail_count += 1
                if self.planning_fail_count >= 2:
                    # [2단계] 두 번 연속 실패 -> 강력 후진 (Strong Backup)
                    self.get_logger().error("Wiggling failed! Force LONG BACKUP (4s)...")
                    self.current_state = State.BACKING_UP
                    self.backup_timer = 40
                    self.planning_fail_count = 0 
                    self.get_logger().warn("Planning failed! Force Wiggling (2s)...")
                else:
                    # [1단계] 첫 실패 -> 제자리 비비기 (Wiggle)
                    self.get_logger().warn("Planning failed! Attempting Wiggle (2s)...")
                    self.current_state = State.WIGGLING
                    self.backup_timer = 20
                
        elif self.current_state == State.EXECUTING:
            # 1. 현재 경로 완료 체크 -> [수정] 재계획(Re-planning) 시도
            if self.wp_idx >= len(self.global_path):
                self.get_logger().info("Current path finished. Checking for new areas...")
                # 바로 멈추지 말고, PLANNING 상태로 돌아가서 새 경로를 찾습니다.
                self.current_state = State.PLANNING 
                self.cmd_vel_publisher.publish(Twist())# 계산하는 동안 잠깐 정지
                return
            
            self.steps_after_planning += 1

            # ---------------------------------------------------------
            # 1. 방문 기록 (발자국) 남기기
            # ---------------------------------------------------------    
            dist_from_last = np.linalg.norm(self.current_pose_xy - self.last_record_pos)
            if dist_from_last > 0.5:
                self.visited_history.append(self.current_pose_xy.copy())
                self.last_record_pos = self.current_pose_xy.copy()
            # ---------------------------------------------------------
            # 2. Lidar Guard (충돌 방지 & 후진)
            # ---------------------------------------------------------
                
            if self.steps_after_planning > 20:
                # 좁은 감시 (정면)
                narrow_indices = range(30, 35)
                narrow_dist = np.min(self.latest_scan_data[narrow_indices]) if len(self.latest_scan_data) > 0 else 99.9
                # 넓은 감시 (측면)
                wide_indices = range(5, 60)  
                wide_dist = np.min(self.latest_scan_data[wide_indices]) if len(self.latest_scan_data) > 0 else 99.9

                #  긴급 충돌 방지 (Emergency) 
                # 코앞(0.5m)에 있거나 옆에 끼일 것 같으면 무조건 후진!
                if narrow_dist < 0.15 or wide_dist < 0.20:
                    self.current_state = State.BACKING_UP
                    self.backup_timer = 20
                    return
                
                # [장애물 감지 Re-planning 거리 1.2m]
                SAFE_DIST_THRESHOLD = 1.2 
                if narrow_dist < SAFE_DIST_THRESHOLD:
                    self.get_logger().warn(f"Obstacle detected ahead ({narrow_dist:.2f}m)! STOP & REPLAN.")
                    self.cmd_vel_publisher.publish(Twist()) # 정지
                    # 2. 현재 경로 폐기
                    self.global_path = []
                    # 3. 즉시 계획 상태로 전환 (SLAM이 지도를 업데이트했을 것이라 가정)
                    self.current_state = State.PLANNING
                    # 4. 무적 시간 초기화 (재계획 직후 바로 또 감지되는 것 방지
                    self.steps_after_planning = 0
                    return
            # =========================================================
            # 4. [업그레이드] 주행 로직 (빙글빙글 방지 & 코너 감속)
            # =========================================================
            
            # (1) Look Ahead 
            look_dist = 1
            look_ahead = min(self.wp_idx + look_dist, len(self.global_path) - 1)
            target = self.global_path[look_ahead]
            curr_x, curr_y = self.current_pose_xy

            # (2) 웨이포인트 계산 및 스킵 판정
            real_target = self.global_path[self.wp_idx]
            dx = real_target[0] - curr_x
            dy = real_target[1] - curr_y
            dist = np.hypot(dx, dy)
            # 목표와의 각도 차이 계산 (등 뒤에 있는지 확인용)
            target_angle = np.arctan2(dy, dx)
            angle_diff = target_angle - self.current_pose_yaw
            while angle_diff > np.pi: angle_diff -= 2*np.pi
            while angle_diff < -np.pi: angle_diff += 2*np.pi

            # 도달 판정
            arrival_threshold = 0.5 

            if dist < arrival_threshold:
                self.wp_idx += 1
                return
            # 등 뒤 스킵 (Behind Checkook_dis) 
            if dist < 1.0 and abs(angle_diff) > 2.0:
                self.wp_idx += 1
                return
            # (3) 제어용 각도 계산 (Look Ahead 타겟 기준)
            dx = target[0] - curr_x
            dy = target[1] - curr_y
            desired_yaw = np.arctan2(dy, dx)
            yaw_err = desired_yaw - self.current_pose_yaw
            while yaw_err > np.pi: yaw_err -= 2*np.pi
            while yaw_err < -np.pi: yaw_err += 2*np.pi
            # (4) 제어 로직 (속도 조절)
            if abs(yaw_err) < 0.05:
                target_ang = 0.0
            else:
                target_ang = np.clip(yaw_err * 1.5, -2.0, 2.0)

            # 전진: 
            #     ㄷ자 코너에서는 조금만 비스듬히 가도 벽을 긁기 때문에 세밀히 조정
            if abs(yaw_err) > 0.4:  
                target_lin = 0.0 
            else:
                # 각도가 완벽하게 맞으면 출발하되,
                # 아직 거리가 멀면 빠르게(0.8), 가까우면 천천히(0.2)
                target_lin = 0.6 if dist > 0.5 else 0.2
            # (C) 필터: 속도와 회전 값에 대한 반응속도. 값이 낮을수록 반응도 낮음.
            alpha_lin = 0.1 
            alpha_ang = 0.4 # 회전 반응 속도 

            self.current_linear_val = (target_lin * alpha_lin) + (self.current_linear_val * (1 - alpha_lin))
            self.current_angular_val = (target_ang * alpha_ang) + (self.current_angular_val * (1 - alpha_ang))
            if abs(self.current_angular_val) < 0.01: self.current_angular_val = 0.0
            
            twist.linear.x = float(self.current_linear_val)
            twist.angular.z = float(self.current_angular_val)

            # =========================================================
            #  5. 스마트 감시견 (Smart Watchdog)
            # =========================================================
            # 회전 중(각도 오차가 0.2rad 이상)일 때는 타이머를 멈춤
            is_turning = abs(yaw_err) > 0.2
            if self.wp_idx == self.last_wp_idx:
                if not is_turning: self.wp_stuck_timer += 1
                else: self.wp_stuck_timer = 0
            else:
                self.wp_stuck_timer = 0
                self.last_wp_idx = self.wp_idx

            if self.wp_stuck_timer > 100: #5초
                self.current_state = State.BACKING_UP
                self.backup_timer = 30
                self.wp_stuck_timer = 0
                return
        # 디버깅용 로그 (1초에 한 번만 출력)
            self.get_logger().info(
                f"WP: {self.wp_idx} | "
                f"CurrAngle: {np.rad2deg(self.current_pose_yaw):.1f} | "
                f"TargetAngle: {np.rad2deg(desired_yaw):.1f} | "
                f"Err: {np.rad2deg(yaw_err):.1f} | "
                f"Cmd: {self.current_angular_val:.2f}",
                throttle_duration_sec=1.0
            )
        elif self.current_state == State.BACKING_UP:
            #  처음 1.5초는 뒤로 빠진다 (후진)
            if self.backup_timer > 15:
                twist.linear.x = -0.5
                twist.angular.z = 0.0
                # 남은 1.5초는 제자리에서 비튼다 (회전)
            else:
                twist.linear.x = 0.0
                twist.angular.z = 1.5 
            self.backup_timer -= 1
            if self.backup_timer <= 0:
                self.cmd_vel_publisher.publish(Twist())
                self.global_path = []
                self.current_state = State.PLANNING

        elif self.current_state == State.WIGGLING:
             # 2초 동안 뒤로 가면서 강제로 돕니다 (구석 탈출)
            if self.backup_timer > 0:
                twist.linear.x = -0.3
                twist.angular.z = 1.5
                self.backup_timer -= 1
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.global_path = []
                self.current_state = State.PLANNING

        elif self.current_state == State.FINISHED:
            self.get_logger().info("Mission Complete!", throttle_duration_sec=5.0)
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # =================================================================
        #  사람 회피 모드 (PPO)
        # =================================================================
        elif self.current_state == State.PPO_HUMAN_AVOID:
             # [안전 장치] 사람이 너무 가까우면(1.5m 이내) PPO고 뭐고 일단 급정지
            if self.latest_human_data and self.latest_human_data.distance < 1.5:
                self.get_logger().error("🚨 HUMAN TOO CLOSE! EMERGENCY STOP!", throttle_duration_sec=1.0)
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_publisher.publish(twist)
                self.action_lock_timer = 0 # 락 초기화
                return # PPO 실행 스킵
            
            # 1. 탈출 조건 (3초 동안 사람 없으면 복귀)
            if self.human_clear_timer >= self.params.HUMAN_CLEAR_TIME:
                self.get_logger().info("✅ Human Clear. Re-planning path...")
                # 안전해지면 다시 경로 계획부터 시작 
                self.current_state = State.PLANNING
                self.global_path = [] 
                self.human_clear_timer = 0
                self.action_lock_timer = 0 # 락 초기화
                return 

            # 2. PPO 실행 (Action Locking 적용)
            if self.ppo_model:
                action = 4 # 기본 정지

                # (A) 락이 걸려있으면 -> AI한테 묻지 말고 저장된 행동 반복
                if self.action_lock_timer > 0:
                    action = self.locked_action
                    self.action_lock_timer -= 1
                    # self.get_logger().info(f"🔒 Locked Action: {action} (Rem: {self.action_lock_timer})")
                
                # (B) 락이 없으면 -> AI에게 물어봄
                else:
                    obs = self.build_state_for_ppo()
                    with torch.no_grad():
                        tensor = torch.FloatTensor(obs).unsqueeze(0)
                        logits, _ = self.ppo_model(tensor)
                        action = torch.argmax(logits).item()
                    
                    # [핵심 로직] 회전 행동(1:좌, 3:우)이 나오면 락을 건다!
                    # 1(Left), 3(Right)일 경우에만 5프레임(0.5초) 동안 유지
                    # 0(Forward)이나 2(Back)는 즉각 반응해도 괜찮음
                    if action == 1 or action == 3:
                        self.action_lock_timer = 5  # 0.1초 * 5 = 0.5초 동안 유지
                        self.locked_action = action
                        self.get_logger().warn(f"🤖 PPO Turn START! Action {action} Locked for 0.5s")
                    
                    # 후진(2)의 경우도 조금 길게 잡아주면 좋음
                    elif action == 2:
                        self.action_lock_timer = 3  # 0.3초
                        self.locked_action = action

                # 액션 실행
                lx, az = self.params.ACTION_MAP[action]
                twist.linear.x = lx
                twist.angular.z = az
                
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(twist)

    # 경로 발행 함수
    def publish_path(self):
        if not self.global_path: return
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for pt in self.global_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.2
            path_msg.poses.append(pose)
        self.path_publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AiControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()