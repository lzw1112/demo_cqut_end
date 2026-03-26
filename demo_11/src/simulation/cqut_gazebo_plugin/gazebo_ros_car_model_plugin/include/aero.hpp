#pragma once

#include "gazebo_ros_car_model_plugin/include/config.hpp"
#include "gazebo_ros_car_model_plugin/include/definitions.hpp"

namespace gazebo_plugins 
{
namespace cqut 
{

class Aero 
{
public:
    explicit Aero(Param::Aero &_params) : params_(_params) {}

    double getFdown(const State &x) { return params_.c_down * x.v_x * x.v_x; }
    double getFdrag(const State &x) { return params_.c_drag * x.v_x * x.v_x; }

 private:
    Param::Aero &params_; // Paramters
};

} // namespace cqut
} // namespace gazebo_plugins
