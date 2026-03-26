import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        Node(name='path_generator',
             package='path_generator',
             executable='path_generator',
             parameters=[os.path.join(get_package_share_directory("path_generator"), "config", "trackdrive.yaml")]),
    ])