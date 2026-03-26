#include <rclcpp/rclcpp.hpp>
#include "path_generator/path_generator.hpp"
#include "path_generator/Utils/visual.h"
#include <sstream>

namespace ns_path_generator 
{

PathGenerator::PathGenerator(rclcpp::Node::SharedPtr nh) : nh_(nh)
{
    nh_->declare_parameter("mission", "acceleration");
    nh_->get_parameter("mission", mission_);

    param_.getParams(nh_, mission_);
    if (mission_ == "trackdrive")
        track_ = &trackdrive_track_;
    else if (mission_ == "acceleration")
        track_ = &line_track_;
    else if (mission_ == "skidpad") 
        track_ = &skidpad_track_;
    else
        RCLCPP_ERROR(nh_->get_logger(), "Undefined Mission name !");
    };

visualization_msgs::msg::MarkerArray PathGenerator::getRefPath()
{ 
    return RefPath_; 
}

void PathGenerator::setCarState(const cqut_msg::msg::CarState &state) 
{
    car_state_ = state;
}

void PathGenerator::setLocalMap(const cqut_msg::msg::Map &map) 
{
    local_map_ = map;
    RCLCPP_INFO(nh_->get_logger(), "set Local Map OK");
}

void PathGenerator::setEndPoint(const geometry_msgs::msg::Point &point) 
{
    endPoint_ = point;
    RCLCPP_INFO(nh_->get_logger(), "setEndPoint OK: %.3f, %.3f", point.x, point.y);
}

void PathGenerator::setTransMat(const std_msgs::msg::Float64MultiArray &array) 
{
    int element_counter = 0;
    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            transMat_(i, j) = array.data[element_counter];
            element_counter++;
        }
    }
}

void PathGenerator::runAlgorithm() 
{
    if (!Check()) 
    {
        RCLCPP_ERROR(nh_->get_logger(), "Message Check ERROR!");
        return;
    }
    RCLCPP_INFO(nh_->get_logger(), "set Track Start.");
    setTrack();
    RCLCPP_INFO(nh_->get_logger(), "set Track Done.");
    track_->setState(VehicleState(car_state_, cmd_));
    track_->CalculateTraj(refline_);

    std::vector<float> color = {1, 0, 0};
    visual_trajectory(refline_, RefPath_, "/base_link", color, car_state_.header, true);
}

bool PathGenerator::Check() 
{
    if (mission_ == "trackdrive") 
    {
        if (local_map_.cone_red.empty() || local_map_.cone_blue.empty()) 
        {
            RCLCPP_WARN(nh_->get_logger(), "Local Map Empty !");
            return false;
        }
    }
    if (mission_ == "acceleration") 
    {
        if (fabs(endPoint_.y) > 6 || endPoint_.x <= 40.0) 
        {
            RCLCPP_WARN(nh_->get_logger(), "Acceleration end point is error, current end point is (%.3f,%.3f).", endPoint_.x, endPoint_.y);
            return false;
        }
    }
    if (mission_ == "skidpad") 
    {
        if (transMat_(3, 3) != 1) 
        {
            RCLCPP_WARN(nh_->get_logger(), "transMatrix is not correct !");
            return false;
        }
    }
    RCLCPP_INFO(nh_->get_logger(), "Successfully passing check");
    return true;
}

void PathGenerator::setTrack() 
{
    if (!is_init) 
    {
        if (mission_ == "acceleration") 
        {
            track_->setEndPoint(endPoint_);
        }
        if (mission_ == "skidpad") 
        {
            track_->setTransMat(transMat_);
        }
        track_->genTraj();
    }
    
    is_init = true;

    if (mission_ == "trackdrive") 
    {
        track_->setMap(local_map_);
        track_->genTraj();
    }
}

cqut_msg::msg::Trajectory PathGenerator::getRefTrajectory() 
{
    cqut_msg::msg::Trajectory refTraj_;
    refTraj_.trajectory.clear();
    for (auto point : refline_) 
    {
        cqut_msg::msg::TrajectoryPoint ref_pt;
        ref_pt.pts.x = point.pts.x;
        ref_pt.pts.y = point.pts.y;
        ref_pt.acc.data = point.acc;
        ref_pt.curvature.data = point.curvature;
        ref_pt.r.data = point.r;
        ref_pt.yaw.data = point.yaw;
        ref_pt.velocity.data = point.velocity;

        // RCLCPP_INFO(nh_->get_logger(), "point vel = %f", point.velocity);

        refTraj_.trajectory.push_back(ref_pt);
    }
    refTraj_.header = car_state_.header;
    return refTraj_;
}
}