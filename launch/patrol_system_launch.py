from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='patrol_system',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen'
        ),
        Node(
            package='patrol_system',
            executable='analyzer_node',
            name='analyzer_node',
            output='screen'
        ),
    ])
