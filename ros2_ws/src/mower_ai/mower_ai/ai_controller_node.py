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
    # HUMAN_CONFIRM_TIME = 0.5  # 0.5초 이상 감지되어야 PPO 진입
    # HUMAN_CLEAR_TIME = 3.0    # 사람이 사라지고 3초 뒤에 복귀
    # HUMAN_DETECT_DIST = 3.0   # 3m 이내일 때만 반응

    # 진입: 1.0m * 2 = 2.0m (이 안으로 들어오면 PPO 켜짐)
    # 해제: 1.4m * 2 = 2.8m (이 밖으로 나가야 PPO 꺼짐)
    PPO_ENTER_DIST = 1.5  
    PPO_EXIT_DIST  = 2.0

    ACTION_MAP = {
        0: (0.5, 0.0),   # 전진
        1: (0.1, 1.2),   # 좌회전
        2: (-0.6, 0.0),  # 후진
        3: (0.1, -1.2),  # 우회전
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
            self.ppo_model = ActorCritic(obs_dim=106, act_dim=5)
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

        # 사람의 이동 방향 계산을 위한 과거 위치 저장 변수
        self.prev_human_pos_x = None
        self.prev_human_pos_y = None
        self.human_move_cos = 0.0
        self.human_move_sin = 0.0

        # 사람 속도 저장 변수
        self.human_speed_norm = 0.0

        self.visited_history = [] 
        self.last_record_pos = np.array([999.0, 999.0])

        self.current_linear_val = 0.0
        self.current_angular_val = 0.0

        self.steps_after_planning = 0
        self.backup_timer = 0
        self.last_wp_idx = -1
        self.wp_stuck_timer = 0

        self.planning_fail_count = 0

        # 교착 상태(Stuck) 판단용 변수
        self.ppo_entry_time = 0.0       # PPO 진입 시각
        self.ppo_entry_pos = None       # PPO 진입 시 위치
        self.is_stuck = False           # 현재 교착 상태인가?
        
        # --- 사람 감지 타이머 ---
        self.human_detect_timer = 0.0
        self.human_clear_timer = 0.0
        
        # --- 경로 관련 ---
        self.global_path = [] 
        self.wp_idx = 0
        
        self.map_data = None
        self.map_info = None
        self.GRID_SIZE_M = 0.8 

        # 위험 구역(사람이 머물렀던 자리) 좌표 저장소
        # 형식: [(x, y), (x, y), ...] (World 좌표계)
        self.danger_zones = []
        
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
        # [디버깅] 데이터 수신 확인
        if len(msgs.objects) > 0:
            self.get_logger().info(f"Vision Recv: {len(msgs.objects)} objs", throttle_duration_sec=1.0)

        # 1. 사람 객체 필터링 및 최단 거리 데이터 추출
        closest_person = None
        min_dist = 99.9
        
        for obj in msgs.objects:
            if obj.label == "person":
                # 거리 조건 없이 일단 다 봅니다 (판단은 main_loop에서 함)
                if obj.distance < min_dist:
                    min_dist = obj.distance
                    closest_person = obj

        # 2. 데이터 업데이트 및 이동 방향 계산 (여기가 핵심!)
        if closest_person:
            self.latest_human_data = closest_person
            
            # (A) 현재 사람의 상대 좌표(x, y) 계산
            # ROS 좌표계: x가 전방, y가 좌측
            # distance와 angle(degree)을 이용해 좌표 변환
            rad = np.deg2rad(closest_person.angle)
            curr_x = closest_person.distance * np.cos(rad)
            curr_y = closest_person.distance * np.sin(rad)
            
            # (B) 과거 데이터가 있다면 이동 벡터 계산
            if self.prev_human_pos_x is not None:
                dx = curr_x - self.prev_human_pos_x
                dy = curr_y - self.prev_human_pos_y
                dist_moved = np.hypot(dx, dy) # 이동 거리
                
                # 움직임이 감지되면 (노이즈 0.05m 이상)
                if dist_moved > 0.05:
                    move_angle = np.arctan2(dy, dx)
                    self.human_move_cos = np.cos(move_angle)
                    self.human_move_sin = np.sin(move_angle)
                    # 속도 계산 (callback은 약 0.1초마다 불린다고 가정 or 시간차 계산)
                    # 정확히 하려면 time.time()을 써야 하지만, 여기서는 단순 추정
                    # 속도 = 거리 / 시간(약 0.1s) -> 거리 * 10
                    estimated_speed = dist_moved * 10.0 
                    
                    # 정규화 (최대 1.5m/s로 나눔)
                    self.human_speed_norm = min(estimated_speed, self.params.VMAX_OBJ) / self.params.VMAX_OBJ
                    # self.get_logger().info(f"Human Moving: dx={dx:.2f}, dy={dy:.2f}")
                else:
                    # 거의 안 움직이면 0으로 (또는 이전 값 유지)
                    self.human_move_cos = 0.0
                    self.human_move_sin = 0.0
                    self.human_speed_norm = 0.0 # 속도 0
            
            # (C) 현재 위치를 과거 위치로 저장 (다음 턴을 위해)
            self.prev_human_pos_x = curr_x
            self.prev_human_pos_y = curr_y
            
        else:
            self.latest_human_data = None
            # 사람이 사라지면 초기화
            self.prev_human_pos_x = None
            self.prev_human_pos_y = None
            self.human_move_cos = 0.0
            self.human_move_sin = 0.0
            self.human_speed_norm = 0.0 # 초기화
        
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
        #  위험 구역(Danger Zone)을 벽(1)으로 칠하기
        for (dx, dy) in self.danger_zones:
            # World 좌표 -> Grid 인덱스 변환
            # generate_path_from_map 함수 내의 변수(fixed_ox, grid_res) 사용
            d_col = int((dx - fixed_ox) / grid_res)
            d_row = int((dy - fixed_oy) / grid_res)
            
            # 위험 반경 (예: 1.0m) -> 격자 칸 수
            radius_cells = int(1.5 / grid_res) 
            
            # 사각형 형태로 벽 칠하기 (원형보다 계산 빠름)
            r_min = max(0, d_row - radius_cells)
            r_max = min(new_h, d_row + radius_cells + 1)
            c_min = max(0, d_col - radius_cells)
            c_max = min(new_w, d_col + radius_cells + 1)
            
            # 해당 영역을 벽(1)으로 설정 -> Planner가 여기로 경로 안 짬
            planner_grid[r_min:r_max, c_min:c_max] = 1
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
        if not self.global_path:
            # 경로가 없으면 목표도 없는 것이므로 거리 0, 각도 0으로 설정 (에러 방지)
            dist = 0.0
            angle = 0.0
        else:
            # 경로가 있을 때만 인덱스로 접근
            # 인덱스가 범위를 벗어나지 않게 안전장치(min) 한번 더 적용
            safe_idx = min(current_target_idx, len(self.global_path) - 1)
            target = self.global_path[safe_idx]
            
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
        # 2. Obstacle Info (15) -> (21)
        obj_feats = []
        # 모드에 따라 AI에게 주입할 데이터를 결정
        target_obs = None

        if self.current_state == State.PPO_HUMAN_AVOID:
            target_obs = self.latest_human_data if self.latest_human_data else self.latest_obstacle_data
        else:
            target_obs = self.latest_obstacle_data

        if target_obs and target_obs.detected:
             fake_dist = max(0.0, target_obs.distance - 0.5)
             d = min(fake_dist, self.params.R_OBJ_M) / self.params.R_OBJ_M
             # d = min(target_obs.distance, self.params.R_OBJ_M) / self.params.R_OBJ_M
             ang = np.deg2rad(target_obs.angle)
             
              # [수정] 학습 환경의 7개 Feature 순서에 맞춰 데이터 주입
             # 순서: [dist, cos, sin, speed, ttc, move_cos, move_sin]
             obj_feats.extend([
                 d, 
                 np.cos(ang), 
                 np.sin(ang), 
                 self.human_speed_norm,  
                 0.5,                    # TTC는 계산 어려우므로 0.5 유지 (중간값)
                 self.human_move_cos, 
                 self.human_move_sin
             ]) 
        
        # 패딩 채우기 (3마리 * 7개 = 21개가 될 때까지)
        while len(obj_feats) < 21: # [수정] 15 -> 21
            # 빈 슬롯 채울 때도 7개씩 채워야 함
            # [dist=1.0, cos=0, sin=0, speed=0, ttc=1, move_cos=0, move_sin=0]
            obj_feats.extend([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0])


        # 3. Lidar (64)
        lidar_feats = np.clip(self.latest_scan_data / self.params.R_RAY_M, 0.0, 1.0)
        # Danger Scalar (2개: 현재 위험도, 주변 위험도)
        danger_scalar = np.zeros(2, dtype=np.float32)
        # Danger Lidar (16개)
        danger_lidar = np.zeros(16, dtype=np.float32)
        # 최종 결합 (총 100차원)
        return np.concatenate([goal_feats,      # 3
                               obj_feats,       # 21
                               lidar_feats,     # 64
                               danger_scalar,   # 2
                               danger_lidar     # 16
                               ], dtype=np.float32)
    
    def main_loop(self):
        twist = Twist()
        
         # 1. 현재 사람과의 거리 계산 (없으면 무한대)
        current_human_dist = 99.9
        if self.latest_human_data:
            current_human_dist = self.latest_human_data.distance

            # ------------------------------------------------------------------
            #  전역 상태 전이 (거리 기반 히스테리시스)
            # ------------------------------------------------------------------
            
            # 사람의 현재 월드 좌표 계산 (로봇 위치 + 상대 좌표)
            # 주의: 정확한 World 좌표를 구하려면 TF 변환이 필요하지만, 
            # 약식으로 (로봇위치 + 상대위치)를 사용합니다.
            
            h_dist = self.latest_human_data.distance
            h_angle = np.deg2rad(self.latest_human_data.angle) # degree -> radian
            
            # 로봇의 현재 yaw(헤딩)까지 고려해야 월드 좌표가 나옴
            global_angle = self.current_pose_yaw + h_angle
            
            human_wx = self.current_pose_xy[0] + (h_dist * np.cos(global_angle))
            human_wy = self.current_pose_xy[1] + (h_dist * np.sin(global_angle))
            
            current_human_pos = np.array([human_wx, human_wy])

            # -----------------------------------------------------
            # [위험 구역 생성]
            # PPO 모드(회피 중)라면, 현재 사람 위치를 위험 구역으로 등록
            # 너무 촘촘하게 찍지 않도록, 기존 구역과 1.0m 이상 떨어져야 찍음
            if self.current_state == State.PPO_HUMAN_AVOID and self.is_stuck:
                is_new_zone = True
                for zone in self.danger_zones:
                    if np.linalg.norm(np.array(zone) - current_human_pos) < 1.0:
                        is_new_zone = False
                        break
                
                if is_new_zone:
                    self.danger_zones.append((human_wx, human_wy))
                    self.get_logger().info(f"🚫 Danger Zone Added at ({human_wx:.1f}, {human_wy:.1f})")

            # -----------------------------------------------------
            # [위험 구역 해제]
            # 사람이 특정 위험 구역에서 3.0m 이상 멀어지면, 그 구역은 해제(삭제)
            # 리스트를 순회하며 남길 것만 남김 (Filter)
            active_zones = []
            for zone in self.danger_zones:
                dist_to_zone = np.linalg.norm(np.array(zone) - current_human_pos)
                if dist_to_zone < 5.0: # 아직 사람이 근처에 있으면 유지
                    active_zones.append(zone)
                else:
                    # 멀어지면 삭제됨 (로그 생략 가능)
                    pass
                
            self.danger_zones = active_zones
            # (A) 진입 조건: 평상시인데, 사람이 진입 거리(2.0m)보다 가까워지면 -> PPO ON
            if self.current_state != State.PPO_HUMAN_AVOID:
                if current_human_dist < self.params.PPO_ENTER_DIST:
                    self.get_logger().warn(f"🚨 HUMAN NEAR ({current_human_dist:.1f}m)! PPO ON.")
                    # 진입 시점 기록 (Stuck 판단용)
                    self.current_state = State.PPO_HUMAN_AVOID
                    self.ppo_entry_time = self.get_clock().now().nanoseconds / 1e9
                    self.ppo_entry_pos = self.current_pose_xy.copy()
                    self.is_stuck = False # 초기화
                    self.action_lock_timer = 0 
            
            # (B) 해제 조건: PPO 중인데, 사람이 해제 거리(2.8m)보다 멀어지거나 사라지면 -> PPO OFF
            elif self.current_state == State.PPO_HUMAN_AVOID:
                # 사람이 아예 사라졌거나(None) or 안전 거리(2.8m) 밖으로 나갔으면
                if self.latest_human_data is None or current_human_dist > self.params.PPO_EXIT_DIST:
                    self.get_logger().info(f"✅ Human Safe ({current_human_dist:.1f}m). Return to Plan.")
                    
                    # 안전해지면 다시 경로 계획부터 시작
                    self.current_state = State.PLANNING
                    self.global_path = []
                    self.latest_human_data = None
                    self.action_lock_timer = 0
                    return # 이번 턴 종료
    
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
            
            #  교착 상태(Stuck) 판단 로직
            current_time = self.get_clock().now().nanoseconds / 1e9
            time_elapsed = current_time - self.ppo_entry_time
            
            # PPO 진입 후 3초가 지났는데
            if time_elapsed > 3.0:
                dist_moved = np.linalg.norm(self.current_pose_xy - self.ppo_entry_pos)
                # 이동 거리가 0.5m 미만이면 -> Stuck!
                if dist_moved < 0.5:
                    if not self.is_stuck:
                        self.get_logger().error("🧱 STUCK Detected! Creating Danger Zone.")
                        self.is_stuck = True

            # [안전 장치] 1.0m 이내 후진 로직은 유지 (최후의 보루)
            if current_human_dist < 0.7:
                self.get_logger().warn("🚨 TOO CLOSE! Backing up...", throttle_duration_sec=1.0)
                twist.linear.x = -0.4
                twist.angular.z = 0.0
                self.cmd_vel_publisher.publish(twist)
                return 

            # (기존의 시간 기반 탈출 조건 삭제됨 -> 위쪽 전역 체크에서 처리함)

            # 2. PPO 실행 (Action Locking 유지)
            if self.ppo_model:
                action = 4 
                
                # (A) 락이 걸려있으면
                if self.action_lock_timer > 0:
                    action = self.locked_action
                    self.action_lock_timer -= 1
                
                # (B) 락이 없으면
                else:
                    obs = self.build_state_for_ppo()
                    with torch.no_grad():
                        tensor = torch.FloatTensor(obs).unsqueeze(0)
                        logits, _ = self.ppo_model(tensor)
                        action = torch.argmax(logits).item()
                    
                    # 회전(1, 3) 시 0.5초 락킹
                    if action == 1 or action == 3:
                        self.action_lock_timer = 5 
                        self.locked_action = action
                        self.get_logger().info(f"Action Lock: {action}")
                    elif action == 2:
                        self.action_lock_timer = 3
                        self.locked_action = action

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