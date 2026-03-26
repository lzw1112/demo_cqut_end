#pragma once

#include "fsd_control/Utils/types.h"
#include "fsd_control/Utils/param.h"
#include "fsd_control/Utils/visual.h"

#include "cqut_msg/msg/trajectory.hpp"
#include "cqut_msg/msg/trajectory_point.hpp"

#include "fsd_control/Solver/solver_base.h"
#include "fsd_control/Solver/mpc_solver.h"
#include "fsd_control/Solver/pure_pursuit_solver.h"

namespace ns_control 
{

class Control 
{
public:
    Control(rclcpp::Node::SharedPtr nh);

    void runAlgorithm();

    void setCarState(const cqut_msg::msg::CarState &msgs);

    void setTrack(const Trajectory &msgs);

    visualization_msgs::msg::MarkerArray getPrePath();

    cqut_msg::msg::ControlCommand getCmd();

    visualization_msgs::msg::MarkerArray PrePath_;

private:

    bool Check();

    rclcpp::Node::SharedPtr nh_{nullptr};
    std::string controller_;

    Solver *solver_;
    MPC_Solver mpc_solver_;
    Pure_Pursuit_Solver pure_pursuit_solver_;

    cqut_msg::msg::CarState car_state_;
    cqut_msg::msg::ControlCommand cmd_;

    Trajectory refline_;

    bool is_init = false;
};

}
