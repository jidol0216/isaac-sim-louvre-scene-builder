"""
Louvre SLAM Launch File
=======================
맵 바뀌면 이걸로 새 맵 생성

사용법:
1. Isaac Sim에서 씬 로드 & Play
2. ros2 launch louvre_navigation louvre_slam.launch.py
3. 로봇 조종해서 맵 스캔 (teleop 또는 RViz goal)
4. 스캔 완료 후: ros2 run nav2_map_server map_saver_cli -f ~/louvre_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    louvre_nav_dir = get_package_share_directory("louvre_navigation")
    
    # SLAM Toolbox params
    slam_params_file = os.path.join(louvre_nav_dir, "params", "slam_params.yaml")
    
    rviz_config_dir = os.path.join(louvre_nav_dir, "rviz2", "louvre_slam.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),

        # TF publishers for LiDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_laser_tf',
            arguments=['0.3', '0', '0.2', '0', '0', '0', 'base_link', 'front_laser']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_laser_tf',
            arguments=['-0.3', '0', '0.2', '3.14159', '0', '0', 'base_link', 'rear_laser']
        ),

        # Laser scan merger (front + rear -> /scan)
        Node(
            package='louvre_navigation',
            executable='laser_scan_merger',
            name='laser_scan_merger',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # SLAM Toolbox - Online Async Mode (가장 빠름)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
        ),

        # RViz for SLAM visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # Teleop - 별도 터미널에서 실행 필요:
        # ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ])
