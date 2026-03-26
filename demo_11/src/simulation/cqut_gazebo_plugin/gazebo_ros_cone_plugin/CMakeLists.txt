add_library(gazebo_ros_cone SHARED
  src/gazebo_ros_cone.cpp
  ../gazebo_utils/src/gazebo_utils.cpp
)

target_include_directories(gazebo_ros_cone PUBLIC 
  ${PROJECT_SOURCE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(gazebo_ros_cone
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "tf2"
  "pcl_ros"
  "visualization_msgs"
)
target_link_libraries(gazebo_ros_cone
  ${PCL_LIBRARIES}
)
ament_export_libraries(gazebo_ros_cone)
ament_export_include_directories(${PROJECT_SOURCE_DIR})
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)
 
install(
  TARGETS gazebo_ros_cone
  DESTINATION lib/${PROJECT_NAME}
)