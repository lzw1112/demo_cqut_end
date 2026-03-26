#include "fsd_control/Solver/solver_base.h"
#include <rclcpp/rclcpp.hpp>


namespace ns_control 
{

Trajectory Solver::getTrajectory() 
{ 
    return predictive_path; 
}

void Solver::setTrajectory(const Trajectory &trajectory) 
{
    RCLCPP_INFO(rclcpp::get_logger("Solver_Base"), "set solver trajectory");
    trajectory_ = trajectory;
}

void Solver::setState(const VehicleState &state) 
{ 
    RCLCPP_INFO(rclcpp::get_logger("Solver_Base"), "set state");
    state_ = state; 
    RCLCPP_INFO(rclcpp::get_logger("Solver_Base"), "finish set state");
}

cqut_msg::msg::ControlCommand Solver::getCmd() 
{
    return control_command_;
}

} // namespace ns_control
