#pragma once

// CQUT Includes
#include "gazebo_ros_car_model_plugin/include/axle.hpp"
#include "gazebo_ros_car_model_plugin/include/aero.hpp"

// ROS2 Msgs
#include "cqut_msg/msg/car_info.hpp"
#include "cqut_msg/msg/cmd.hpp"
#include "cqut_msg/msg/wheel_speeds.hpp"
#include "cqut_msg/msg/res_state.hpp"
#include "cqut_msg/msg/mission.hpp"
#include "cqut_msg/msg/state.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// Utills
#include "gazebo_utils/include/gazebo_utils.hpp"

// ROS2
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo_plugins 
{

namespace cqut 
{

class Vehicle 
{
public:
    Vehicle(gazebo::physics::ModelPtr &_model,
            sdf::ElementPtr &_sdf,
            rclcpp::Node::SharedPtr nh,
            gazebo::transport::NodePtr &gznode);

    void onRes(const cqut_msg::msg::ResState::SharedPtr msg);

    void update(double dt);

    void printInfo();

    void publish(double sim_time);

 private:

    State f(const State &x,
            const Input &u,
            double Fx,
            double M_TV,
            const AxleTires &FyF,
            const AxleTires &FyR);

    State f_kin_correction(const State &x_in,
                           const State &x_state,
                           const Input &u,
                           const double Fx,
                           const double M_TV,
                           const AxleTires &FyF,
                           const AxleTires &FyR,
                           const double dt);


    void setPositionFromWorld();

    void publishTf(const State &x);

    void publishCarInfo(const AxleTires &alphaF,
                        const AxleTires &alphaR,
                        const AxleTires &FyF,
                        const AxleTires &FyR,
                        const double Fx) const;

    double getFx(const State &x, const Input &u);

    double getMTv(const State &x, const Input &u) const;

    void onCmd(const cqut_msg::msg::Cmd::SharedPtr msg);

    void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void initModel(sdf::ElementPtr &_sdf);

    void initVehicleParam(sdf::ElementPtr &_sdf);

    double getNormalForce(const State &x);

    void setModelState(const State &x);

    double getGaussianNoise(double mean, double var) const;

    State &getState() { return state_; }

    Input &getInput() { return input_; }

 private:

    // ROS Nodehandle
    rclcpp::Node::SharedPtr nh_{nullptr};

    // ROS Publishrs
    rclcpp::Publisher<cqut_msg::msg::State>::SharedPtr pub_ground_truth_{nullptr};
    rclcpp::Publisher<cqut_msg::msg::CarInfo>::SharedPtr pub_car_info_{nullptr};

    // ROS Subscribers
    rclcpp::Subscription<cqut_msg::msg::Cmd>::SharedPtr sub_cmd_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_{nullptr};
    rclcpp::Subscription<cqut_msg::msg::ResState>::SharedPtr sub_res_{nullptr};

    // ROS TF
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_{nullptr};

    /// Pointer to the parent model
    gazebo::physics::ModelPtr model;

    /// Chassis link and Base Link
    gazebo::physics::LinkPtr chassisLink;
    gazebo::physics::LinkPtr base_link_;

    // Front and Rear Axle
    FrontAxle front_axle_;
    RearAxle  rear_axle_;

    // Car Infor and RES
    cqut_msg::msg::CarInfo  car_info_;
    cqut_msg::msg::ResState res_state_;

    // Parameters
    Param param_;

    // States
    State state_;
    Input input_;
    double time_last_cmd_;

    // Consider Aerodynamics
    Aero aero_;

    // Name of the System
    std::string robot_name_;
};

typedef std::unique_ptr<Vehicle> VehiclePtr;

} // namespace cqut
} // namespace gazebo_plugins
