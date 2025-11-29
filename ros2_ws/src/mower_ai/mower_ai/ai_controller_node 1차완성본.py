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
# rl 폴더가 없으면 에러나므로 구조 확인 필요
from .rl.network import ActorCritic 
from .planning.cpp import CoveragePlanner, HeuristicType

# --- ROS 메시지 ---
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
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
    # 전체 작업 영역 설정 (Global Map Settings)
    GLOBAL_WIDTH_M = 20.0
    GLOBAL_HEIGHT_M = 20.0

    # 사람 감지 관련 파라미터 추가
    HUMAN_CONFIRM_TIME = 0.5  # 0.5초 이상 감지되어야 PPO 진입
    HUMAN_CLEAR_TIME = 3.0    # 사람이 사라지고 3초 뒤에 복귀
    HUMAN_DETECT_DIST = 3.0   # 3m 이내일 때만 반응

    # 액션 매핑 (인덱스 -> 선속도, 각속도)
    ACTION_MAP = {
        0: (0.5, 0.0),   # 전진 (속도 줄임)
        1: (0.0, 0.8),   # 좌회전
        2: (-0.6, 0.0),  # 후진
        3: (0.0, -0.8),  # 우회전
        4: (0.0, 0.0),   # 정지
    }
    DANGER_M = 2.0  # 감지거리에 장애물 감지 시 회피

class State:
    WAITING_FOR_MAP = 0 # 지도 대기 중
    PLANNING = 1        # 경로 계산 중
    EXECUTING = 2       # 이동 중
    FINISHED = 3        # 완료
    BACKING_UP = 4  #  강제 후진 상태
    WIGGLING = 5  # 제자리 비비기 상태
    PPO_HUMAN_AVOID = 6 # 사람 회피 모드

class AiControllerNode(Node):
    def __init__(self):
        super().__init__('ai_controller_node')
        self.get_logger().info("AI Controller: Initializing...")
        #  외부 파라미터 선언 (기본값 20.0 설정)
        # 이제 실행할 때 이 값을 바꿔주면 코드 수정 없이 맵 크기가 바뀝니다.
        self.declare_parameter('map_width', 20.0)
        self.declare_parameter('map_height', 20.0)
        self.current_state = State.WAITING_FOR_MAP
        self.params = AIParams()
        # PPO 유지 타이머
        self.ppo_active_timer = 0
        # --- PPO 모델 로드 ---
        self.ppo_model = None
        try:
            #  obs_dim=100 (Goal 3 + Objs 15 + Lidar 64 + Danger 2 + DangerLidar 16)
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
        self.latest_obstacle_data = None

        # 방문한 발자국(World 좌표)을 저장할 리스트
        self.visited_history = [] 
        self.last_record_pos = np.array([999.0, 999.0])

        # 스무딩을 위한 현재 속도 기억 변수
        self.current_linear_val = 0.0
        self.current_angular_val = 0.0

        self.steps_after_planning = 0
        #  후진 타이머
        self.backup_timer = 0

        # 멍 때림 방지용 타이머
        self.last_wp_idx = -1
        self.wp_stuck_timer = 0

        self.human_detect_timer = 0.0  # 감지 지속 시간 누적
        self.human_clear_timer = 0.0   # 사라짐 지속 시간 누적
        self.is_human_threatening = False # 현재 위협적인 사람이 있는지 플래그


        # 경로 생성 실패 횟수 카운터
        self.planning_fail_count = 0
        
        # --- 경로 관련 ---
        self.global_path = [] # [(x, y), ...]
        self.wp_idx = 0
        self.move_state = 'ALIGN_X'
        
        # 맵 처리 관련 설정
        self.map_data = None
        self.map_info = None
        self.GRID_SIZE_M = 0.8 # 0.9m 단위로 격자 생성 (경로에 영향)

        # --- ROS 통신 ---
        qos_map = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(DetectedObjectList, '/detected_obstacle', self.obstacle_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # 경로 시각화용 Publisher
        self.path_publisher = self.create_publisher(Path, '/mower_path', 10)
        
        # 메인 루프 (0.1초 주기)
        self.create_timer(0.1, self.main_loop)
        
        self.get_logger().info("Waiting for /map topic...")

    def odom_callback(self, msg: Odometry):
        self.current_pose_xy[0] = msg.pose.pose.position.x
        self.current_pose_xy[1] = msg.pose.pose.position.y
        
        # Quaternion -> Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_pose_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def obstacle_callback(self, msg: DetectedObjectList):
        # 1. 리스트가 비어있으면 -> 장애물 없음 처리
        if len(msg.objects) == 0:
            # 빈 껍데기 객체를 만들어서 "감지 안됨" 상태로 저장
            empty_obj = DetectedObject()
            empty_obj.detected = False
            empty_obj.distance =     99.9
            self.latest_obstacle_data = empty_obj
            return

        # 2. 리스트가 있으면 -> 가장 가까운 객체 하나를 선택 
        # 여러 개 중에 distance가 가장 작은 것을 찾음
        closest_obj = min(msg.objects, key=lambda x: x.distance)
        
        # 3. 데이터 업데이트
        self.latest_obstacle_data = closest_obj

    def scan_callback(self, msg: LaserScan):
        if len(msg.ranges) == 0: return
        
        # 360개 -> 64개 다운샘플링
        raw = np.array(msg.ranges, dtype=np.float32)
        raw[np.isinf(raw)] = self.params.R_RAY_M
        raw = np.clip(raw, 0.0, self.params.R_RAY_M)
        
        if len(raw) == self.params.RAY_COUNT:
            self.latest_scan_data = raw
        else:
            indices = np.linspace(0, len(raw)-1, self.params.RAY_COUNT).astype(int)
            self.latest_scan_data = raw[indices]

    def map_callback(self, msg: OccupancyGrid):
        # 1. 빈 지도 무시
        if msg.info.width == 0 or msg.info.height == 0:
            return

        # 2. [수정] 상태와 상관없이 항상 최신 지도 데이터를 갱신합니다.
        # 로봇이 움직이면서 지도가 계속 넓어지기 때문입니다.
        self.map_data = msg.data
        self.map_info = msg.info

        # 3. 처음 시작할 때만 상태를 변경하여 루프를 시작시킵니다.
        if self.current_state == State.WAITING_FOR_MAP:
            self.get_logger().info(f"Initial Map Received! Size: {msg.info.width}x{msg.info.height}")
            self.current_state = State.PLANNING

    def generate_path_from_map(self):
        """ SLAM 지도를 0.5m 격자로 압축하여 CoveragePlanner 실행 """
        self.get_logger().info("Start Path Planning...")
        
        # 1. SLAM 맵 메타데이터 가져오기 (장애물 확인용)

        if self.map_info is None:
            self.get_logger().warn("No Map Info yet.")
            return False
        
        slam_res = self.map_info.resolution
        slam_w = self.map_info.width
        slam_h = self.map_info.height
        slam_ox = self.map_info.origin.position.x
        slam_oy = self.map_info.origin.position.y
        
        # SLAM 데이터를 2차원 배열로 변환
        raw_map = np.array(self.map_data).reshape(slam_h, slam_w)

        global_w = self.get_parameter('map_width').value
        global_h = self.get_parameter('map_height').value
        # 2. 고정된 플래너 격자(Canvas) 생성
        # 우리가 정의한 20m x 20m 크기의 고정 맵을 만듭니다.
        grid_res = self.GRID_SIZE_M  # 0.8m
        
        # 고정된 맵의 시작점 (왼쪽 아래 구석 좌표)
        fixed_ox = 0.0
        fixed_oy = -20.0
        
        # 격자 개수 계산 (20m / 0.8m = 25칸)
        new_w = int(global_w / grid_res)
        new_h = int(global_h / grid_res)
        
        # 일단 "전부 빈 공간(0)"으로 초기화 -> 일단 가보자!
        planner_grid = np.zeros((new_h, new_w), dtype=int)
        
        #  맵의 테두리(Border)를 벽으로 막기
        # (알고리즘이 맵 밖으로 나가지 않고 안쪽을 채우게 유도함)
        planner_grid[0, :] = 1
        planner_grid[-1, :] = 1
        planner_grid[:, 0] = 1
        planner_grid[:, -1] = 1

        # 3. SLAM 데이터 오버레이 (장애물만 업데이트)
        # 고정된 격자 한 칸(0.8m)마다 SLAM 지도의 해당 영역을 확인합니다.
        for r in range(new_h):
            for c in range(new_w):
                # 현재 격자의 World 좌표 범위 계산
                # 격자 중심이 아니라 격자의 '영역'을 계산해야 함
                cell_min_x = fixed_ox + (c * grid_res)
                cell_max_x = cell_min_x + grid_res
                cell_min_y = fixed_oy + (r * grid_res)
                cell_max_y = cell_min_y + grid_res
                
                # World 좌표 -> SLAM Grid 인덱스로 변환
                # SLAM 지도 밖이라면(아직 안 가본 곳) -> 장애물 없음(0) 처리
                slam_c_min = int((cell_min_x - slam_ox) / slam_res)
                slam_c_max = int((cell_max_x - slam_ox) / slam_res)
                slam_r_min = int((cell_min_y - slam_oy) / slam_res)
                slam_r_max = int((cell_max_y - slam_oy) / slam_res)
                
                # 인덱스 범위 체크 (SLAM 맵 밖으로 나가는 경우 제외)
                slam_c_min = max(0, slam_c_min); slam_c_max = min(slam_w, slam_c_max)
                slam_r_min = max(0, slam_r_min); slam_r_max = min(slam_h, slam_r_max)
                
                # 유효한 영역이 없으면(SLAM 맵 밖) 패스 -> Free(0) 유지
                if slam_c_min >= slam_c_max or slam_r_min >= slam_r_max:
                    continue
                
                # 해당 영역의 장애물 개수 세기
                chunk = raw_map[slam_r_min:slam_r_max, slam_c_min:slam_c_max]
                obstacle_count = np.count_nonzero(chunk > 50)
                
                # 장애물 민감도 
                if obstacle_count > 12:
                    planner_grid[r, c] = 1  # 벽으로 표시
        
        # # 인플레이션(Inflation): 벽 두껍게 만들어 로봇이 벽에 붙지 않도록 함
        # inflation_radius = 0 # 1칸 = 격자의 크기
        # inflated_grid = planner_grid.copy()
        
        # rows, cols = planner_grid.shape
        # for r in range(rows):
        #     for c in range(cols):
        #         if planner_grid[r, c] == 1: # 진짜 벽을 발견하면
        #             # 주변을 벽으로 칠함
        #             r_min = max(0, r - inflation_radius)
        #             r_max = min(rows, r + inflation_radius + 1)
        #             c_min = max(0, c - inflation_radius)
        #             c_max = min(cols, c + inflation_radius + 1)
        #             inflated_grid[r_min:r_max, c_min:c_max] = 1
                    
        # # 이제 planner는 두꺼워진 벽 정보를 사용합니다.
        # planner_grid = inflated_grid 
            

        # 4. 시작 위치 설정
        # 현재 로봇 위치(World) -> Grid Index
        start_c = int((self.current_pose_xy[0] - fixed_ox) / grid_res)
        start_r = int((self.current_pose_xy[1] - fixed_oy) / grid_res)
        
        # 맵 범위 체크
        start_c = np.clip(start_c, 0, new_w-1)
        start_r = np.clip(start_r, 0, new_h-1)

        # 끼임 방지 (주변 빈칸 탐색)
        if planner_grid[start_r, start_c] == 1:
            self.get_logger().warn("Start position is inside OBSTACLE/INFLATION! Searching nearby free cell...")
            found_free = False
            # 주변 3칸(약 2.4m) 이내를 뒤짐
            search_range = 3  
            for r_off in range(-search_range, search_range + 1):
                for c_off in range(-search_range, search_range + 1):
                    nr, nc = start_r + r_off, start_c + c_off
                    # 맵 범위 안이고, 벽이 아니라면(0)
                    if 0 <= nr < new_h and 0 <= nc < new_w and planner_grid[nr, nc] == 0:
                        start_r, start_c = nr, nc
                        found_free = True
                        break
                if found_free: break
            
            if found_free:
                self.get_logger().info(f"Moved Start Node to Free Cell: ({start_r}, {start_c})")
            else:
                self.get_logger().error("CRITICAL: Robot is completely stuck inside walls!")
                return False
         # 2. [핵심] 시작 위치 강제 초기화 (Force Start Node Free)
        # 로봇이 벽 옆에 있어서 0.8m 격자에 걸렸더라도, 
        # "일단 출발은 하게 해줘"라고 강제로 빈 공간(0)으로 만듭니다.
        
        # 시작 인덱스가 맵 범위 안인지 확인
        if 0 <= start_r < new_h and 0 <= start_c < new_w:
            # 로봇이 서 있는 칸은 무조건 빈 땅으로 간주
            planner_grid[start_r, start_c] = 0 
            
            # (옵션) 끼임 방지를 위해 주변 8칸도 벽이 아니라고 할 수도 있음
            # 하지만 현재 칸만 비워도 탈출에는 충분합니다.
            self.get_logger().info(f"Forced Start Node ({start_r}, {start_c}) to be FREE for planning.")
        # 5. CoveragePlanner 실행
        try:
            planner = CoveragePlanner(planner_grid)
            planner.start(initial_orientation=0)
            planner.current_pos = [start_r, start_c, 0] # 보정된 위치 주입

            # 1. 현재 맵 크기에 맞는 빈 마스크 생성
            visited_mask = np.zeros((new_h, new_w), dtype=bool)
            
            # 2. 기록해둔 발자국(World 좌표)을 현재 격자(Grid)에 찍기
            for pos in self.visited_history:
                vc = int((pos[0] - fixed_ox) / grid_res) 
                vr = int((pos[1] - fixed_oy) / grid_res)
                
                # 맵 범위 내라면 '방문함'으로 마킹
                if 0 <= vr < new_h and 0 <= vc < new_w:
                    visited_mask[vr, vc] = True

            # 시작 위치는 방문 안 한 것으로 처리 (일단 출발은 해야 하니까)
            visited_mask[start_r, start_c] = False 
            planner.visited = visited_mask
            
            planner.compute()
            result = planner.result() 
            
            # [수정 1] 경로가 없거나 찾지 못했으면 바로 실패 처리
            # result[0]은 성공 여부, result[4]는 경로 리스트
            if not result[0] or len(result[4]) == 0:
                self.get_logger().error("Planning Failed! No path found")
                return False
            
            path_indices = result[4] # [(y, x), ...] 격자 좌표
            
            # 6. Grid Index -> World 좌표 변환 (고정맵 기준)
            self.global_path = []
            for (r, c) in path_indices:
                wx = (c * grid_res) + fixed_ox + (grid_res/2)
                wy = (r * grid_res) + fixed_oy + (grid_res/2)
                self.global_path.append([float(wx), float(wy)])

            self.get_logger().info(f"Global Path Generated! Points: {len(self.global_path)}")
            self.publish_path()
            
               
            #     # [수정] 설정된 격자 크기(GRID_SIZE_M)에 맞춰서 반올림
            #     grid_size = self.GRID_SIZE_M
                
            #     wx = round(raw_wx / grid_size) * grid_size
            #     wy = round(raw_wy / grid_size) * grid_size
                
            #     self.global_path.append([float(wx), float(wy)])
                
            #     # 3. 변환된 깔끔한 좌표 저장
            #     self.global_path.append([float(wx), float(wy)])
                
            # self.get_logger().info(f"Path Generated! Points: {len(self.global_path)}")
            # self.publish_path()

            #  로봇의 현재 위치와 경로상의 모든 점들 사이의 거리를 계산합니다.
            # 그리고 '가장 가까운 점'의 순서(Index)를 찾습니다.
            # 방어 코드: 경로가 비어있으면 거리 계산 스킵
            if not self.global_path:
                self.get_logger().error("Path conversion resulted in empty list!")
                return False
            
            # path_array = np.array(self.global_path)
            # dists = np.linalg.norm(path_array - self.current_pose_xy, axis=1)
            # nearest_idx = np.argmin(dists)

            # start_padding = 2
            # self.wp_idx = min(nearest_idx + start_padding, len(self.global_path) - 1)
            # self.wp_idx = 0 
            self.wp_idx = min(3, len(self.global_path) - 1)
            self.get_logger().info(f"Path Planned from scratch. WP Count: {len(self.global_path)}. Starting at WP 0.")
            self.steps_after_planning = 0

            # 타이머 초기화 
            self.wp_stuck_timer = 0
            self.last_wp_idx = -1

            return True
            
        except Exception as e:
            self.get_logger().error(f"Planner Error: {e}")
            return False

    # 카메라 콜백 (4개 카메라 통합 처리 가정)
    # msg는 bounding box 리스트나 detected object 리스트라고 가정
    def human_detection_callback(self, msgs):
        self.get_logger().info(f"Cam Data Recv: {len(msgs.objects)} objects", throttle_duration_sec=1.0)
        detected = False
        min_dist = 99.9
        
        for obj in msgs.objects:
            if obj.label == "person":
                # 1. 거리 체크 (Lidar와 융합되어 거리가 나오거나 BBox 크기로 추정)
                if obj.distance < self.params.HUMAN_DETECT_DIST:
                    # 2. 각도 체크 (전방 120도 이내인지)
                    # 내 로봇의 진행 방향 기준 -60 ~ +60도
                    if abs(obj.angle) < 60.0:
                        detected = True
                        min_dist = min(min_dist, obj.distance)

        # 상태 업데이트 (0.1초 주기 루프에서 사용하기 위해 플래그 저장)
        if detected:
            self.human_detect_timer += 0.1 # 타이머 증가 (콜백 주기에 따라 조정 필요)
            self.human_clear_timer = 0.0   # 클리어 타이머 리셋
        else:
            self.human_detect_timer = 0.0  # 감지 타이머 리셋
            self.human_clear_timer += 0.1  # 클리어 타이머 증가

        # [진입/해제 결정 로직]
        if self.current_state != State.PPO_HUMAN_AVOID:
            # 평상시 -> 사람 감지 모드 진입 조건
            if self.human_detect_timer >= self.params.HUMAN_CONFIRM_TIME:
                self.get_logger().warn(f"HUMAN DETECTED ({min_dist:.1f}m)! Switch to PPO.")
                self.current_state = State.PPO_HUMAN_AVOID
                self.human_detect_timer = 0 # 리셋
                
        elif self.current_state == State.PPO_HUMAN_AVOID:
            # 회피 중 -> 평상시 복귀 조건
            if self.human_clear_timer >= self.params.HUMAN_CLEAR_TIME:
                self.get_logger().info("Human clear. Resuming Path Following.")
                
                # [중요] 복귀 시 즉시 경로를 다시 찾도록 설정
                self.current_state = State.PLANNING 
                self.global_path = [] # 기존 경로 폐기 후 재탐색 (사람 피하느라 엉켰을 수 있으므로)

    def build_state_for_ppo(self):
        # PPO용 관측 벡터 생성 (82차원 -> 100차원)
        # 1. Goal Info (3)
        target = self.global_path[self.wp_idx] if self.wp_idx < len(self.global_path) else self.global_path[-1]
        dx = target[0] - self.current_pose_xy[0]
        dy = target[1] - self.current_pose_xy[1]
        dist = np.hypot(dx, dy)
        angle = np.arctan2(dy, dx) - self.current_pose_yaw
        
        goal_feats = [
            min(dist, self.params.R_GOAL_M) / self.params.R_GOAL_M,
            np.cos(angle),
            np.sin(angle)
        ]
        
        # 2. Obstacle Info (15)
        obj_feats = []
        # (간소화: 실제 감지된 객체 하나만 넣고 나머진 패딩)
        if self.latest_obstacle_data and self.latest_obstacle_data.detected:
             # 거리 정규화
             d = min(self.latest_obstacle_data.distance, self.params.R_OBJ_M) / self.params.R_OBJ_M
             # 각도 (이미 로컬 각도라고 가정)
             ang = np.deg2rad(self.latest_obstacle_data.angle)
             # ROS에서는 속도/ TTC를 정확히 알기 어려움을 추정값(0.5)이나 0.0 사용
             obj_feats.extend([d, np.cos(ang), np.sin(ang), 0.5, 0.5]) 
        # 패딩 채우기
        while len(obj_feats) < 15:
            obj_feats.extend([1.0, 0.0, 0.0, 0.0, 1.0])
            
        # 3. Lidar (64)
        lidar_feats = np.clip(self.latest_scan_data / self.params.R_RAY_M, 0.0, 1.0)
        
        
        # 4. Danger Features (18)
        # 학습 환경에는 Danger Map이 있지만, ROS 실전에서는 히트맵 정보가 없습니다.
        # 따라서 차원을 맞추기 위해 0.0으로 채웁니다. 
        
        # Danger Scalar (2개: 현재 위험도, 주변 위험도)
        danger_scalar = np.zeros(2, dtype=np.float32)
        
        # Danger Lidar (16개)
        danger_lidar = np.zeros(16, dtype=np.float32)
        
        # 최종 결합 (총 100차원)
        return np.concatenate([
            goal_feats,     # 3
            obj_feats,      # 15
            lidar_feats,    # 64
            danger_scalar,  # 2  
            danger_lidar    # 16 
        ], dtype=np.float32)
    
    def main_loop(self):
        twist = Twist()
        
        if self.current_state != State.PPO_HUMAN_AVOID:
            # human_detect_timer는 콜백에서 업데이트된다고 가정
            if self.human_detect_timer >= self.params.HUMAN_CONFIRM_TIME:
                self.get_logger().warn("🚨 HUMAN DETECTED! Switching to PPO.")
                self.current_state = State.PPO_HUMAN_AVOID
                self.human_detect_timer = 0
                self.human_clear_timer = 0
    
        # --- 상태 머신 ---
        if self.current_state == State.WAITING_FOR_MAP:
            # 지도 올 때까지 정지
            pass
            
        elif self.current_state == State.PLANNING:
            # 경로 생성 시도
            success = self.generate_path_from_map()
            if success:
                self.planning_fail_count = 0
                self.current_state = State.EXECUTING
                self.move_state = 'ALIGN_X'
                self.steps_after_planning = 0 # 무적 시간 초기화
                self.wp_stuck_timer = 0
                self.last_wp_idx = -1
                self.get_logger().info(f">>> Timer RESET. Starting from WP {self.wp_idx}")
            else:
                self.planning_fail_count += 1
                if self.planning_fail_count >= 2:
                    # [2단계] 두 번 연속 실패 -> 강력 후진 (Strong Backup)
                    self.get_logger().error("Wiggling failed! Force LONG BACKUP (4s)...")
                    
                    self.current_state = State.BACKING_UP
                    self.backup_timer = 40  # 4초 동안 길게 후진!
                    self.planning_fail_count = 0 
                    self.get_logger().warn("Planning failed! Force Wiggling (2s)...")
                else:
                    # [1단계] 첫 실패 -> 제자리 비비기 (Wiggle)
                    self.get_logger().warn("Planning failed! Attempting Wiggle (2s)...")
                    self.current_state = State.WIGGLING
                    self.backup_timer = 20 # 2초
                
        elif self.current_state == State.EXECUTING:
            # 1. 현재 경로 완료 체크 -> [수정] 재계획(Re-planning) 시도
            if self.wp_idx >= len(self.global_path):
                self.get_logger().info("Current path finished. Checking for new areas...")
                
                # 바로 멈추지 말고, PLANNING 상태로 돌아가서 새 경로를 찾습니다.
                self.current_state = State.PLANNING 
                self.cmd_vel_publisher.publish(Twist()) # 계산하는 동안 잠깐 정지
                return
            
            # 카메라 하나일때 사용하는 것
            # # 2. 긴급 회피 (PPO) 체크
            # use_ppo = False
            # if self.latest_obstacle_data and self.latest_obstacle_data.detected:
            #     if self.latest_obstacle_data.distance < self.params.DANGER_M:
            #         use_ppo = True
            
            # if use_ppo:
            #     self.ppo_active_timer = 15
            # # 타이머가 살아있으면 PPO 실행
            # if self.ppo_active_timer > 0 and self.ppo_model:
            #     self.ppo_active_timer -= 1 # 타이머 감소
            #     narrow_indices = range(28, 37)
            #     narrow_dist = np.min(self.latest_scan_data[narrow_indices]) if len(self.latest_scan_data) > 0 else 99.9
                
            #     if narrow_dist < 0.3:
            #         self.get_logger().error("PPO Fail-safe! Too close. Force Backup.")
            #         twist.linear.x = -0.5
            #         twist.angular.z = 0.0
            #         self.cmd_vel_publisher.publish(twist)
            #         return
                
            #     self.get_logger().info(f"AVOID: PPO Active (Timer: {self.ppo_active_timer})")
            #     obs = self.build_state_for_ppo()

            #     with torch.no_grad():
            #         tensor = torch.FloatTensor(obs).unsqueeze(0)
            #         logits, _ = self.ppo_model(tensor)
            #         action = torch.argmax(logits).item()

            #         # AI가 무슨 키를 눌렀는지 확인하는 로그
            #         # 0:전진, 1:좌회전, 2:후진, 3:우회전, 4:정지
            #     action_str = ["FWD", "LEFT", "BACK", "RIGHT", "STOP"]
            #     dist = self.latest_obstacle_data.distance
            #     self.get_logger().info(f"🤖 PPO Decided: {action_str[action]} | Dist: {dist:.2f}m")
            #     lx, az = self.params.ACTION_MAP[action]
            #     twist.linear.x = float(lx)
            #     twist.angular.z = float(az)
                

                #################################################
                ######               일반 주행             #######
                #################################################
            else:
                self.steps_after_planning += 1

                # ---------------------------------------------------------
                # 1. [유지] 방문 기록 (발자국) 남기기
                # ---------------------------------------------------------
                dist_from_last = np.linalg.norm(self.current_pose_xy - self.last_record_pos)
                if dist_from_last > 0.5:
                    self.visited_history.append(self.current_pose_xy.copy())
                    self.last_record_pos = self.current_pose_xy.copy()

                # ---------------------------------------------------------
                # 2. [유지] Lidar Guard (충돌 방지 & 후진)
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
                        detect_dist = min(narrow_dist, wide_dist)
                        self.get_logger().warn(f"Obstacle detected ({detect_dist:.2f}m)! Backing up...")
                        
                        self.current_state = State.BACKING_UP
                        self.backup_timer =20
                        return
                    
                    SAFE_DIST_THRESHOLD = 1.2 
                    if narrow_dist < SAFE_DIST_THRESHOLD:

                        self.get_logger().warn(f"Obstacle detected ahead ({narrow_dist:.2f}m)! STOP & REPLAN.")
                        
                        # 1. 즉시 정지 명령 발행 (관성 캔슬)
                        stop_twist = Twist()
                        stop_twist.linear.x = 0.0
                        stop_twist.angular.z = 0.0
                        self.cmd_vel_publisher.publish(stop_twist)

                        # 2. 현재 경로 폐기
                        self.global_path = []
                        
                        # 3. 즉시 계획 상태로 전환 (SLAM이 지도를 업데이트했을 것이라 가정)
                        self.current_state = State.PLANNING
                        
                        # 4. 무적 시간 초기화 (재계획 직후 바로 또 감지되는 것 방지)
                        self.steps_after_planning = 0
                        return
                    


                # =========================================================
                # 4. [업그레이드] 주행 로직 (빙글빙글 방지 & 코너 감속)
                # =========================================================
                
                # (1) Look Ahead 
                look_dist = 1  # 1칸에 0.8m 2~4사이 설정
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
                # arrival_threshold = 0.2 if abs(angle_diff) > 0.8 else 0.5
                arrival_threshold = 0.5 

                if dist < arrival_threshold:
                    self.wp_idx += 1
                    return
                
                # 등 뒤 스킵 (Behind Checkook_dis) 
                if dist < 1.0 and abs(angle_diff) > 2.0:
                    self.get_logger().info(f"Skipping passed WP {self.wp_idx}")
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
                if abs(yaw_err) < 0.05: # 3도 이내
                    target_ang = 0.0
                else:
                    # P-Gain
                    target_ang = np.clip(yaw_err * 1.5, -2.0, 2.0)

                # 전진: 
                #     ㄷ자 코너에서는 조금만 비스듬히 가도 벽을 긁기 때문입니다.
                if abs(yaw_err) > 0.4:  
                    target_lin = 0.0 
                else:
                    # 각도가 완벽하게 맞으면 출발하되,
                    # 아직 거리가 멀면 빠르게(0.8), 가까우면 천천히(0.1)
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
                # 회전 중(각도 오차가 0.2rad 이상)일 때는 타이머를 멈춥니다.
                is_turning = abs(yaw_err) > 0.2

                if self.wp_idx == self.last_wp_idx:
                    if not is_turning:
                        self.wp_stuck_timer += 1
                    else:
                        self.wp_stuck_timer = 0 # 회전 중이면 리셋
                else:
                    self.wp_stuck_timer = 0
                    self.last_wp_idx = self.wp_idx

                if self.wp_stuck_timer > 100: # 5초
                    self.get_logger().warn(f"Stuck at WP {self.wp_idx} (Aligned but blocked)! Force Backing Up...")
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
            # 랜덤한 방향이나 한쪽으로 틉니다.
            else:
                twist.linear.x = 0.0
                twist.angular.z = 1.5 # 왼쪽으로 강제 회전 (시야 변경)

            self.backup_timer -= 1

            #  탈출 시간이 끝나면?
            if self.backup_timer <= 0:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_publisher.publish(twist) # 일단 정지
                
                self.get_logger().info("Escape complete. Re-planning...")


                self.global_path = []
                self.current_state = State.PLANNING

        elif self.current_state == State.WIGGLING:
            # 2초 동안 뒤로 가면서 강제로 돕니다 (구석 탈출)
            if self.backup_timer > 0:
                twist.linear.x = -0.3   # 뒤로
                twist.angular.z = 1.5   # 강하게 회전
                self.backup_timer -= 1
            else:
                # 시간이 다 되면 정지하고 즉시 재계획 (지도 기다리지 않음!)
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
                
                self.get_logger().info("Wiggle complete. Retrying Planning...")
                self.global_path = []
                self.current_state = State.PLANNING # 바로 계획 시도

        elif self.current_state == State.FINISHED:
            self.get_logger().info("Mission Complete!", throttle_duration_sec=5.0)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif self.current_state == State.PPO_HUMAN_AVOID:
            # 1. 탈출 조건 체크 (사람이 사라지고 3초 지났는지?)
            if self.human_clear_timer >= self.params.HUMAN_CLEAR_TIME:
                self.get_logger().info("✅ Human Clear. Re-planning path...")
                
                # 안전해지면 다시 경로 계획부터 시작 (가장 안전함)
                self.current_state = State.PLANNING
                self.global_path = [] 
                self.human_clear_timer = 0
                return # 이번 턴 종료

            # 2. PPO 실행 (사람 회피 기동)
            if self.ppo_model:
                # PPO용 관측 데이터 생성 (카메라 정보를 반영하고 싶다면 여기서 obs 수정)
                obs = self.build_state_for_ppo()
                
                with torch.no_grad():
                    tensor = torch.FloatTensor(obs).unsqueeze(0)
                    logits, _ = self.ppo_model(tensor)
                    action = torch.argmax(logits).item()

                # PPO가 결정한 행동 수행
                lx, az = self.params.ACTION_MAP[action]
                twist.linear.x = lx
                twist.angular.z = az
                
                self.get_logger().info(f"RUNAWAY: PPO Action {action}", throttle_duration_sec=0.5)
            else:
                # 모델이 없으면 그냥 정지
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(twist)

    # 경로 발행 함수
    def publish_path(self):
        if not self.global_path:
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for pt in self.global_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.2 # 바닥보다 살짝 위에 그림
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