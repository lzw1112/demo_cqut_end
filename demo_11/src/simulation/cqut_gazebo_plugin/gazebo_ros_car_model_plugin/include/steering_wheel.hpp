#pragma once

#include "gazebo_ros_car_model_plugin/include/wheel.hpp"

namespace gazebo_plugins 
{

namespace cqut 
{

class WheelStearing : public Wheel 
{
public:

    WheelStearing(gazebo::physics::ModelPtr &_model,
                  sdf::ElementPtr &_sdf,
                  const std::string _name,
                  gazebo::transport::NodePtr &gznode,
                  rclcpp::Node::SharedPtr nh) : Wheel(_model, _sdf, _name, gznode, nh) 
    {
        max_steer_            = 1.0;
        std::string full_name = _model->GetName() + "::" + _sdf->Get<std::string>(_name + "_steering");
        getJoint(steering_joint_, _model, full_name);
    }

    void setAngle(const double delta) override 
    {
        steering_joint_->SetPosition(0, delta);
    }

    void printInfo() override 
    {
        Wheel::printInfo();
        RCLCPP_DEBUG(rclcpp::get_logger("steering_wheel"), " - STEERING");
    }

    bool isSteering() override { return true; }

 private:

    gazebo::physics::JointPtr steering_joint_;      // Joint Holder

    double max_steer_;                      // Maximal steering angle [rad]
};
}
}
