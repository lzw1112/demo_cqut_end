#pragma once

#include "cqut_msg/msg/control_command.hpp"
#include "fsd_control/Utils/types.h"
#include "fsd_control/Utils/param.h"
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace ns_control 
{

class Solver 
{
public:
    void setTrajectory(const Trajectory &trajectory); // only for mpc
    void setState(const VehicleState &state);
    Trajectory getTrajectory();
    Trajectory predictive_path;
    cqut_msg::msg::ControlCommand getCmd();
    virtual void solve() = 0;

protected:
    Trajectory trajectory_;
    VehicleState state_;    // a marker point for skidpad
    cqut_msg::msg::ControlCommand control_command_;
};

}; // namespace ns_control
