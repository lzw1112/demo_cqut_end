add_library(gazebo_ros_track SHARED
  src/gazebo_track.cpp
)

target_include_directories(gazebo_ros_track PUBLIC 
  /usr/include/eigen3
)

ament_target_dependencies(gazebo_ros_track
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "visualization_msgs"
  "cqut_msg"
)

ament_export_libraries(gazebo_ros_track)
ament_export_include_directories(${PROJECT_SOURCE_DIR})
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)
 
install(
  TARGETS gazebo_ros_track
  DESTINATION lib/${PROJECT_NAME}
)