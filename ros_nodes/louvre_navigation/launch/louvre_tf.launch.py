from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Front LiDAR TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_laser_tf',
            arguments=['0.3', '0', '0.2', '0', '0', '0', 'base_link', 'front_laser']
        ),
        # Rear LiDAR TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_laser_tf',
            arguments=['-0.3', '0', '0.2', '0', '0', '3.14159', 'base_link', 'rear_laser']
        ),
    ])
