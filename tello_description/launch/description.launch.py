import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# https://qiita.com/hakuturu583/items/7e3a278630422e17f0ba
share_dir_path = os.path.join(get_package_share_directory('tello_description'))
urdf_path = os.path.join(share_dir_path, 'urdf', 'tello.urdf')


def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             arguments=[urdf_path],
             output='screen'
        ),
    ])