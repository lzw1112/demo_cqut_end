#pragma once

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "cqut_msg/msg/car_state.hpp"
#include "cqut_msg/msg/map.hpp"
#include "cqut_msg/msg/trajectory.hpp"
#include "cqut_msg/msg/trajectory_point.hpp"
#include "path_generator/Utils/param.h"
#include "path_generator/Utils/types.h"
#include "path_generator/Track/track_base.h"
#include "path_generator/Track/line_track.h"
#include "path_generator/Track/skidpad_track.h"
#include "path_generator/Track/trackdrive_track.h"

namespace ns_path_generator 
{

    class PathGenerator 
    {

    public:
        // Constructor
        PathGenerator(rclcpp::Node::SharedPtr nh);

        // Getters

        visualization_msgs::msg::MarkerArray getRefPath();
        cqut_msg::msg::Trajectory getRefTrajectory();

        // Setters
        void setCarState(const cqut_msg::msg::CarState &state);

        void setEndPoint(const geometry_msgs::msg::Point &point);

        void setLocalMap(const cqut_msg::msg::Map &map);

        void setTransMat(const std_msgs::msg::Float64MultiArray &array);

        void runAlgorithm();

    private:
        rclcpp::Node::SharedPtr nh_;
        std::string mission_;
        geometry_msgs::msg::Point endPoint_;
        Eigen::Matrix4f transMat_;
        cqut_msg::msg::Map local_map_;
        cqut_msg::msg::CarState car_state_;
        Trajectory refline_;

        cqut_msg::msg::ControlCommand cmd_;
        Track *track_;
        Autox_Track trackdrive_track_;
        Line_Track line_track_;
        Skidpad_Track skidpad_track_;

        visualization_msgs::msg::MarkerArray RefPath_;
        bool is_init = false;

        bool Check();

        void setTrack();
    };
}
