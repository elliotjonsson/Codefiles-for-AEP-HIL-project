#!/usr/bin/env python3
"""
ROS2 Launch file for AEP Package VR
Launches all nodes with proper dependencies and sequencing
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    """Generate launch description with all nodes in proper sequence"""
    
    # Node 1: LIDAR distance processing (foundation for obstacle detection)
    lidar_node = Node(
        package='aep_package_vr',
        executable='lidar_distance',
        name='lidar_distance_node',
        output='screen',
    )
    
    # Node 2: Steering generator (vehicle control with emergency brake)
    gen_steering_node = Node(
        package='aep_package_vr',
        executable='gen_steering',
        name='gen_steering',
        output='screen',
        parameters=[
            {'max_linear': 0.7},
            {'max_angular': 3.0},
            {'min_stop_distance_cm': 15.0}
        ]
    )
    
    # Node 3: Message node (collects robot state for overlays)
    message_node = Node(
        package='aep_package_vr',
        executable='message_node',
        name='message_node',
        output='screen',
    )
    
    # Node 4: VR listener (camera servo control via UDP)
    vr_listener_node = Node(
        package='aep_package_vr',
        executable='vr_listener',
        name='udp_servo_node',
        output='screen',
    )
    
    # Node 5: Camera stream with overlays
    stream_node = Node(
        package='aep_package_vr',
        executable='stream',
        name='camera_stream_node',
        output='screen',
    )
    
    # Sequence the launches with delays to ensure proper initialization
    return LaunchDescription([
        lidar_node,
        TimerAction(
            period=1.0,
            actions=[gen_steering_node]
        ),
        TimerAction(
            period=1.5,
            actions=[message_node]
        ),
        TimerAction(
            period=2.0,
            actions=[vr_listener_node]
        ),
        TimerAction(
            period=2.5,
            actions=[stream_node]
        ),
    ])
