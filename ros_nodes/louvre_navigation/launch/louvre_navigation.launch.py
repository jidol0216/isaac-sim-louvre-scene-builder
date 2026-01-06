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
    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(louvre_nav_dir, "maps", "louvre_map.yaml"),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(louvre_nav_dir, "params", "louvre_navigation_params.yaml"),
    )

    rviz_config_dir = os.path.join(louvre_nav_dir, "rviz2", "louvre_navigation.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=map_dir, description="Full path to map file"),
        DeclareLaunchArgument("params_file", default_value=param_dir, description="Full path to param file"),
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

        # RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
            launch_arguments={
                "namespace": "",
                "use_namespace": "False",
                "rviz_config": rviz_config_dir
            }.items(),
        ),

        # Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "bringup_launch.py")),
            launch_arguments={
                "map": map_dir,
                "use_sim_time": use_sim_time,
                "params_file": param_dir
            }.items(),
        ),
    ])
