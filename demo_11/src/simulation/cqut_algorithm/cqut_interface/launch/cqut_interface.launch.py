import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        Node(name='cqut_interface',
             package='cqut_interface',
             executable='cqut_interface_node',
             parameters=[os.path.join(get_package_share_directory("cqut_interface"), "config", "cqut_interface.yaml")]),
    ])