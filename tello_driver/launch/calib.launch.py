from launch import LaunchDescription
from launch_ros.actions import Node


# Launch nodes required for joystick operation


def generate_launch_description():
    return LaunchDescription([
        Node(package='ros2_aruco', executable='aruco_node', output='screen'),
    ])
