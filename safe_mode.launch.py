from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aep_package_vr',
            executable='lidar_distance',  # Changed from lidar_distance_brake
            name='lidar_distance_node',
            output='screen'
        ),
        Node(
            package='aep_package_vr',
            executable='message_node',
            name='message_node',
            output='screen'
        ),
        Node(
            package='aep_package_vr',
            executable='stream',
            name='camera_stream_node',
            output='screen'
        ),
        Node(
            package='aep_package_vr',
            executable='gen_steering',
            name='gen_steering_node',
            output='screen'
        ),
    ])
