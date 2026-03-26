#pragma once

// Gazebo Include
#include <gazebo/physics/physics.hh>
#include <gazebo/common/PID.hh>

// STD Include
#include <string>

// cqut Include
#include "gazebo_utils/include/gazebo_utils.hpp"
#include "gazebo_ros_car_model_plugin/include/config.hpp"
#include "gazebo_ros_car_model_plugin/include/definitions.hpp"

// Ignition Robotics Includes
#include <ignition/math/Pose3.hh>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker_array.hpp"

namespace gazebo_plugins 
{

namespace cqut 
{

class Wheel 
{
public:

    Wheel(gazebo::physics::ModelPtr &_model,
          sdf::ElementPtr &_sdf,
          std::string _name,
          gazebo::transport::NodePtr &gznode,
          rclcpp::Node::SharedPtr nh);

    virtual void printInfo();

    double getFy(double alpha, double Fz);

    virtual bool isSteering() { return false; }

    void setParam(const Param::Tire &param) { param_ = param; }

    virtual void setAngle(const double delta) {}

    const ignition::math::Vector3<double> &getCenterPos() const;

private:

    double getCollisionRadius(gazebo::physics::CollisionPtr _coll);

    gazebo::physics::ModelPtr &model_;           // Reference to the model

    std::string full_name_;                      // Name of the element

    Param::Tire param_;                          // Pacejka Tire parameters
    double      radius;                          // Radious of the tire from STL [m]

    ignition::math::Vector3<double> center_pos_; // Center location of the tire
};

}  // namespace cqut
}  // namespace gazebo_plugins
