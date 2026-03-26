#pragma once

#include "cqut_msg/msg/map.hpp"
#include "path_generator/Utils/param.h"
#include "path_generator/Utils/types.h"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ns_path_generator 
{

void visual_trajectory(const Trajectory &traj,
                       visualization_msgs::msg::MarkerArray &visual,
                       const std::string &frame, const std::vector<float> &color,
                       const std_msgs::msg::Header &header, bool is_vel);

void color_map(double vel, std::vector<float> &color);

void visual_map(const cqut_msg::msg::Map &map, visualization_msgs::msg::MarkerArray &visual);

} // namespace ns_path_generator