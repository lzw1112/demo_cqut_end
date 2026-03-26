import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions.append_environment_variable import AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():

    this_directory = get_package_share_directory("cqut_description")

    xacro_path = os.path.join(this_directory, "urdf", "car.xacro")
    rviz_config_file = os.path.join(this_directory, "rviz", "rviz.rviz")
    rviz_icon_file = os.path.join(this_directory, "rviz", "cqut.png")
    
    append_environment = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.path.join(os.path.join(get_package_share_directory('cqut_gazebo_plugin'), '..', '..', 'lib', 'cqut_gazebo_plugin'))
    )

    declare_robot_name_cmd = DeclareLaunchArgument(name="robot_name", default_value="sj", description="仿真机器人名称")
    declare_gui_cmd = DeclareLaunchArgument(name="gui", default_value="True", description="是否可视化gazebo")
    declare_sensor_config_cmd = DeclareLaunchArgument(name="sensor_config", default_value=os.path.join(this_directory, "config", "sensors.yaml"), description="传感器配置文件")
    declare_car_config_cmd = DeclareLaunchArgument(name="car_config", default_value=os.path.join(this_directory, "config", "car.yaml"), description="车辆参数配置文件")
    declare_distance_config_cmd = DeclareLaunchArgument(name="distance_config", default_value=os.path.join(this_directory, "config", "distances.yaml"), description="关节安装位置配置文件")
    declare_world_cmd = DeclareLaunchArgument(name="world", default_value=os.path.join(this_directory, "world", "FSAC.world"), description="gazebo世界模型文件")
    robot_name = LaunchConfiguration("robot_name")
    gui = LaunchConfiguration("gui")
    sensor_config_file = LaunchConfiguration("sensor_config")
    car_config_file = LaunchConfiguration("car_config")
    distance_config_file = LaunchConfiguration("distance_config")
    world = LaunchConfiguration("world")

    robot_description = Command(["xacro", " ", xacro_path, 
                                 " robot_name:=", robot_name, 
                                 " sensors_config_file:=", sensor_config_file, 
                                 " car_config_file:=", car_config_file,
                                 " car_dimensions_file:=", distance_config_file],
    )

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )

    start_joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"description_file": robot_description}]
    )

    start_rviz_cmd = Node(
        package="rviz2", 
        executable="rviz2", 
        arguments=["-d", rviz_config_file, "-s", rviz_icon_file], 
        output="screen"
    )

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world, "gui": gui}.items(),
    )

    start_spawn_robot_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "cqut", "-topic", "robot_description",],
        output="screen",
    )

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd, 
            on_exit=EmitEvent(event=Shutdown(reason="rviz exited"))
        )
    )

    ld = LaunchDescription()
    ld.add_action(append_environment)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_sensor_config_cmd)
    ld.add_action(declare_car_config_cmd)
    ld.add_action(declare_distance_config_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_spawn_robot_cmd)
    ld.add_action(exit_event_handler)
    # ld.add_action(start_spawn_controller)

    return ld
