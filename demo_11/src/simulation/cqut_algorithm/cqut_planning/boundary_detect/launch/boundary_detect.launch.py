import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        Node(name='boundaryDetector',
             package='boundary_detect',
             executable='boundary_detect',
             parameters=[os.path.join(get_package_share_directory("boundary_detect"), "config", "boundary_detector.yaml")]),
    ])