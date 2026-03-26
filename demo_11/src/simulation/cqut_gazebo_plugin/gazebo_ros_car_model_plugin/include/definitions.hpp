#pragma once

#include "cqut_msg/msg/state.hpp"
#include <rclcpp/rclcpp.hpp>

namespace gazebo_plugins 
{

namespace cqut 
{

struct State 
{
    State(double x_ = 0.0, double y_ = 0.0, double yaw_ = 0.0, double v_x_ = 0.0, double v_y_ = 0.0, 
          double r_ = 0.0, double a_x_ = 0.0, double a_y_ = 0.0, double theta_ = 0.0)
    : x(x_), y(y_), yaw(yaw_), v_x(v_x_), v_y(v_y_), r(r_), a_x(a_x_), a_y(a_y_), theta(theta_) {}

    State operator*(const double &dt) const 
    {
        return {dt * x, dt * y, dt * yaw, dt * v_x, dt * v_y, dt * r, dt * a_x, dt * a_y};
    }

    State operator+(const State &x2) const 
    {
        return {x + x2.x, y + x2.y, yaw + x2.yaw, v_x + x2.v_x, v_y + x2.v_y, r + x2.r, a_x + x2.a_x, a_y + x2.a_y};
    }

    std::stringstream &operator<<(std::stringstream &os) 
    {
        os << getString();
        return os;
    }

    inline std::string getString() const 
    {
        std::string str = "x:" + std::to_string(x)
                          + "| y:" + std::to_string(y)
                          + "| yaw:" + std::to_string(yaw)
                          + "| v_x:" + std::to_string(v_x)
                          + "| v_y:" + std::to_string(v_y)
                          + "| r:" + std::to_string(r)
                          + "| a_x:" + std::to_string(a_x)
                          + "| a_y:" + std::to_string(a_y);
        return str;
    }

    cqut_msg::msg::State toRos(const rclcpp::Time &time) const 
    {
        cqut_msg::msg::State state;
        state.header.stamp    = time;
        state.header.frame_id = "map";
        state.x               = x;
        state.y               = y;
        state.yaw             = yaw;
        state.vx              = v_x;
        state.vy              = v_y;
        state.r               = r;
        state.ax             = a_x;
        state.ay             = a_y;
        state.theta           = theta;
        return state;
    }

    void validate() { v_x = std::max(0.0, v_x); }

    double x;
    double y;
    double yaw;
    double v_x;
    double v_y;
    double r;
    double a_x;
    double a_y;
    double theta;
};

struct Input 
{
    Input() : dc(0.0), delta(0.0) {}

    std::string getString() 
    {
        return "dc:" + std::to_string(dc) + " | delta:" + std::to_string(delta);
    }
    double dc;
    double delta;
};

} // namespace cqut
} // namespace gazebo_plugins
