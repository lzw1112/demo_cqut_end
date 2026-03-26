#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ns_control
{
    struct Weight 
    {
        double px;
        double py;
        double pyaw;
        double cte;
        double epsi;
        double v;
        double steer;
        double throttle;
        double steer_rate;
        double throttle_rate;
    };

    struct Param 
    {
        // MPC Solver N, dt
        int N;
        double dt;
        // Weight
        Weight weight;

        double car_length;

        double initial_velocity;

        double look_ahead;

        // For Pure Pursuit
        double desire_vel;

        // Get Parameters from yaml
        void getParams(rclcpp::Node::SharedPtr nh, const std::string &controller) 
        {
            nh->declare_parameter("car_length", 1.88);
            nh->declare_parameter("N", 40);
            nh->declare_parameter("dt", 0.04);
            nh->declare_parameter("weight/px", 3.0);
            nh->declare_parameter("weight/py", 10.0);
            nh->declare_parameter("weight/pyaw", 8.0);
            nh->declare_parameter("weight/cte", 1.0);
            nh->declare_parameter("weight/epsi", 4.0);
            nh->declare_parameter("weight/v", 0.4);
            nh->declare_parameter("weight/steer", 10.0);
            nh->declare_parameter("weight/throttle", 10.0);
            nh->declare_parameter("weight/steer_rate", 2000.0);
            nh->declare_parameter("weight/throttle_rate", 10.0);
            nh->declare_parameter("look_ahead", 10.0);
            nh->declare_parameter("initial_velocity", 2.0);
            nh->declare_parameter("weight/desire_vel", 15.0);
            nh->declare_parameter("desire_vel", 3.0);

            nh->get_parameter("car_length", car_length);
            nh->get_parameter("N", N);
            nh->get_parameter("dt", dt);
            nh->get_parameter("weight/px", weight.px);
            nh->get_parameter("weight/py", weight.py);
            nh->get_parameter("weight/pyaw", weight.pyaw);
            nh->get_parameter("weight/cte", weight.cte);
            nh->get_parameter("weight/epsi", weight.epsi);
            nh->get_parameter("weight/v", weight.v);
            nh->get_parameter("weight/steer", weight.steer);
            nh->get_parameter("weight/throttle", weight.throttle);
            nh->get_parameter("weight/steer_rate", weight.steer_rate);
            nh->get_parameter("weight/throttle_rate", weight.throttle_rate);
            nh->get_parameter("look_ahead", look_ahead);
            nh->get_parameter("initial_velocity", initial_velocity);

            if (controller == "mpc")
                nh->get_parameter("weight/desire_vel", desire_vel);
            else
                nh->get_parameter("desire_vel", desire_vel);
        }
    };

    extern Param control_param_;

}


