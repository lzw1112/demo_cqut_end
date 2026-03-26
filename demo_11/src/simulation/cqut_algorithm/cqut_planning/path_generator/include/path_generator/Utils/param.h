#pragma once

#include <rclcpp/rclcpp.hpp>
struct Param 
{
    int N;
    double dt;
    bool simulation;

    double car_length;

    double initial_velocity;

    // Trajectory Interval
    double interval;

    double forward_distance;

    double circle_radius;

    double max_lat_acc;

    // For Pure Pursuit
    double desire_vel;

    // Get Parameters from yaml
    void getParams(rclcpp::Node::SharedPtr nh, const std::string &mission) 
    {
        nh->declare_parameter("car_length", 1.88);
        nh->declare_parameter("N", 40);
        nh->declare_parameter("dt", 0.04);
        nh->declare_parameter("simulation", true);
        nh->declare_parameter("interval", 0.08);
        nh->declare_parameter("forward_distance", 15.0);
        nh->declare_parameter("circle_radius", 9.125);
        nh->declare_parameter("max_lat_acc", 3.0);
        nh->declare_parameter("initial_velocity", 2.0);
        nh->declare_parameter("desire_vel", 3.0);

        nh->get_parameter("car_length", car_length);
        nh->get_parameter("N", N);
        nh->get_parameter("dt", dt);
        nh->get_parameter("simulation", simulation);
        nh->get_parameter("interval", interval);
        nh->get_parameter("forward_distance", forward_distance);
        nh->get_parameter("circle_radius", circle_radius);
        nh->get_parameter("max_lat_acc", max_lat_acc);
        nh->get_parameter("initial_velocity", initial_velocity);
        nh->get_parameter("desire_vel", desire_vel);
    }
};

extern Param param_;
