#include "fsd_control/Solver/pure_pursuit_solver.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace ns_control 
{
    
void Pure_Pursuit_Solver::solve() 
{
    RCLCPP_INFO(rclcpp::get_logger("Pure_Pursuit"), "begin solve");
    if (trajectory_.empty()) 
    {
        control_command_.throttle.data       = static_cast<float>(-1.0);
        control_command_.steering_angle.data = 0.0;
        RCLCPP_INFO(rclcpp::get_logger("Pure_Pursuit"), "trajectory empty");
        return;
    }

    double desire_vel = control_param_.desire_vel;

    const auto i_next = control_param_.look_ahead;
    geometry_msgs::msg::Point32 next_point;

    { // Steering Control
        const double beta_est = control_command_.steering_angle.data * 0.5;
        next_point.x = trajectory_[i_next].pts.x*std::cos(state_.yaw)  - trajectory_[i_next].pts.y*std::sin(state_.yaw);
        next_point.y = trajectory_[i_next].pts.x*std::sin(state_.yaw)  + trajectory_[i_next].pts.y*std::cos(state_.yaw);
        const double eta = std::atan2(next_point.y, next_point.x) - (state_.yaw + beta_est);
        const double length = std::hypot(next_point.y, next_point.x);

        control_command_.steering_angle.data    = static_cast<float>(1.5 * std::atan(2.0 / length * std::sin(eta)));
        RCLCPP_INFO(rclcpp::get_logger("Pure_Pursuit"), "steering: %lf", control_command_.steering_angle.data);
    }
    { // Speed Controller
        const double vel = state_.v;
        control_command_.throttle.data = static_cast<float>(desire_vel - vel);
    }
}

}