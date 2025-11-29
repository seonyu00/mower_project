# mower_launch.py (최종 버전)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # mower_ai 패키지의 공유 디렉토리 경로를 찾음
    mower_ai_share_dir = get_package_share_directory('mower_ai')
    
    # slam.yaml 설정 파일의 전체 경로를 만듦
    slam_params_file = os.path.join(mower_ai_share_dir, 'config', 'slam.yaml')

    # --- use_sim_time 파라미터를 모든 노드에 적용 ---
    use_sim_time = True
    
    return LaunchDescription([
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='unity_endpoint',
            parameters=[{'use_sim_time': use_sim_time}], # 추가!
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf_pub',
            arguments = ['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link'],
            parameters=[{'use_sim_time': use_sim_time}], # 추가!
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params_file, {'use_sim_time': use_sim_time}],  # 이제 파라미터 파일을 통째로 넘겨줌
            output='screen'
        ),
        Node(
            package='mower_ai',
            executable='ai_controller',
            name='ai_controller',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])