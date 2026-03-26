import os, yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    this_directory = get_package_share_directory("cqut")
    cqut_description_directory = get_package_share_directory("cqut_description")
    
    config_path = os.path.join(this_directory, "config", "trackdrive.yaml")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    world = os.path.join(cqut_description_directory + "/world/" + config["track_name"] + ".world")

    start_simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("cqut_description"), "launch", "cqut_description.launch.py")
        ),
        launch_arguments={
            "world": world, 
            "gui": "True",
            "robot_name": config["robot_name"],

        }.items(),
    )

    start_interface_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("cqut_interface"), "launch", "cqut_interface.launch.py")
        ),
    )

    start_planning_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("path_generator"), "launch", "trackdrive.launch.py")
        ),
    )

    start_control_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fsd_control"), "launch", "trackdrive.launch.py")
        ),
    )

    ld = LaunchDescription()
    ld.add_action(start_simulation_cmd)
    ld.add_action(start_interface_cmd)
    ld.add_action(start_planning_cmd)
    ld.add_action(start_control_cmd)
    return ld