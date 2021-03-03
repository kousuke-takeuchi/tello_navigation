from launch import LaunchDescription
from launch_ros.actions import Node


# Launch nodes required for joystick operation


def generate_launch_description():
    return LaunchDescription([
        Node(package='image_view',
             executable='image_view',
             remappings=[
                 ('image', '/image_raw'),
             ),
        Node(package='tello_driver', executable='tello_driver_main', output='screen'),
        Node(package='tello_nav', executable='test1', output='screen'),
    ])
