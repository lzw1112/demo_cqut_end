import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        Node(name='lidar_cluster',
             package='lidar_cluster',
             executable='lidar_cluster',
             parameters=[os.path.join(get_package_share_directory("lidar_cluster"), "config", "lidar_cluster.yaml")]),
    ])