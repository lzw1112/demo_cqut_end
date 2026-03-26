#pragma once

#include "gazebo_ros_car_model_plugin/include/steering_wheel.hpp"

namespace gazebo_plugins 
{

namespace cqut 
{

struct AxleTires 
{
    double left;
    double right;

    double avg() const { return (left + right) / 2.0; }
};

template<class WheelType>
class Axle 
{
public:

    Axle(gazebo::physics::ModelPtr &_model,
         sdf::ElementPtr &_sdf,
         const std::string name,
         gazebo::transport::NodePtr &gznode,
         rclcpp::Node::SharedPtr nh);

    const ignition::math::Vector3<double> &getAxlePos() const;

    void printInfo();

    void getSlipAngles(const State &x, const Input &u, double &alphaL, double &alphaR);

    void getFy(const State &x, const Input &u, const double Fz, AxleTires &Fy, AxleTires *alpha = nullptr);

    void setSteering(double delta);

    void setLeverArm(double _car_length, double _weight_factor, double _width);

    double getDownForce(const double Fz);

    void setParam(Param param);

    const WheelType &getWheelLeft() const { return wheel_l_; }
    const WheelType &getWheelRight() const { return wheel_r_; }

private:
    // Name of the model
    std::string name_;

    // Left and Right wheel
    WheelType wheel_l_;
    WheelType wheel_r_;

    // Center Position of the axle
    ignition::math::Vector3<double> axlePos;

    // Parameters
    double axle_width_;        // Width [m]
    double lever_arm_length_;  // COG to middle axle [m]
    double car_length_;        // Length of the car [m]
    double weight_factor_;     // Weight distribution [m]

    int axle_factor_;      // Decides whether its front or rear [no units]

};

typedef Axle<Wheel>         RearAxle;
typedef Axle<WheelStearing> FrontAxle;

} // namespace cqut
} // namespace gazebo_plugins
