import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        Node(name='line_detector',
             package='line_detect',
             executable='line_detect',
             parameters=[os.path.join(get_package_share_directory("line_detect"), "config", "line_detector.yaml")]),
    ])