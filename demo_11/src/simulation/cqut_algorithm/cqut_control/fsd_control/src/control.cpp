#include <rclcpp/rclcpp.hpp>
#include "fsd_control/Utils/param.h"
#include "fsd_control/control.hpp"
#include <sstream>

namespace ns_control 
{

Param control_param_;

Control::Control(rclcpp::Node::SharedPtr nh) : nh_(nh) 
{
    nh_->declare_parameter("controller", "pure_pursuit");
    nh_->get_parameter("controller", controller_);

    control_param_.getParams(nh_, controller_);

    if (controller_ == "pure_pursuit") 
    { 
        solver_ = &pure_pursuit_solver_; 
    }
    else if (controller_ == "mpc") 
    { 
        solver_ = &mpc_solver_; 
    }
    else 
    {
        RCLCPP_ERROR(nh_->get_logger(), "Undefined Solver name !");
    }
}

void Control::setCarState(const cqut_msg::msg::CarState &msgs) 
{ 
    car_state_ = msgs; 
}

void Control::setTrack(const Trajectory &msgs) 
{
    refline_ = msgs; 
}

cqut_msg::msg::ControlCommand Control::getCmd() 
{ 
    return cmd_; 
}

visualization_msgs::msg::MarkerArray Control::getPrePath() 
{ 
    return PrePath_; 
}

bool Control::Check() 
{
    if (refline_.empty()) 
        return false;
    return true;
}

void Control::runAlgorithm() 
{
    if (!Check()) 
    {
        RCLCPP_WARN(nh_->get_logger(), "Check Error");
        return;
    }

    solver_->setState(VehicleState(car_state_, cmd_));
    solver_->setTrajectory(refline_);
    solver_->solve();

    cmd_ = solver_->getCmd();

    std::vector<float> color_ref = {1, 0, 0};
    std::vector<float> color_pre = {0, 1, 0};
    std::vector<float> color_init = {0, 0, 1};

    if (controller_ == "mpc")
        visual_trajectory(solver_->getTrajectory(), PrePath_, "base_link", color_pre, car_state_.header, true);

    RCLCPP_INFO(nh_->get_logger(), "steering: %lf, throttle: %lf", cmd_.steering_angle.data, cmd_.throttle.data);
}
}
