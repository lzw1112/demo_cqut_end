#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>

// Gazebo Includes
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>

// ROS RACE CAR PLUGIN
#include "gazebo_ros_car_model_plugin/include/vehicle.hpp"

namespace gazebo_plugins 
{

class gazebo_race_car_private 
{
public:
    rclcpp::Node::SharedPtr rosnode;

    gazebo::physics::WorldPtr world;

    gazebo::physics::ModelPtr model;

    gazebo::transport::NodePtr gznode;

public:

    gazebo::event::ConnectionPtr updateConnection;

    cqut::VehiclePtr vehicle;

    gazebo::common::Time lastSimTime;

    gazebo::transport::SubscriberPtr keyboardSub;

    std::mutex mutex;

    gazebo::transport::PublisherPtr worldControlPub;

};

} // namespace gazebo_plugins