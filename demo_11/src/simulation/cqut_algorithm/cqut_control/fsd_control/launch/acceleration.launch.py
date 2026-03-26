import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        Node(name='control',
             package='fsd_control',
             executable='fsd_control',
             parameters=[os.path.join(get_package_share_directory("fsd_control"), "config", "acceleration.yaml")]),
    ])
