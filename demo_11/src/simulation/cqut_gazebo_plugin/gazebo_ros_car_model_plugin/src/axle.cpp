// Vehicle Includes
#include "gazebo_ros_car_model_plugin/include/axle.hpp"
#include "gazebo_ros_car_model_plugin/include/steering_wheel.hpp"

namespace gazebo_plugins 
{

namespace cqut 
{

template<class WheelType>
Axle<WheelType>::Axle(gazebo::physics::ModelPtr &_model,
                      sdf::ElementPtr &_sdf,
                      const std::string name,
                      gazebo::transport::NodePtr &gznode,
                      rclcpp::Node::SharedPtr nh) 
    : name_(name)
    , wheel_l_(_model, _sdf, name + "_left_wheel", gznode, nh)
    , wheel_r_(_model, _sdf, name + "_right_wheel", gznode, nh)
{
    const auto vec3 = wheel_l_.getCenterPos() - wheel_r_.getCenterPos();
    axle_width_ = vec3.Length();

    axlePos = (wheel_l_.getCenterPos() + wheel_r_.getCenterPos()) / 2.0;

    axle_factor_ = name == "front" ? 1 : -1;
}

template<class WheelType>
const ignition::math::Vector3<double> &Axle<WheelType>::getAxlePos() const 
{
    return axlePos;
}

template<class WheelType>
void Axle<WheelType>::printInfo() 
{
    std::string str = "- Axle " + name_ + ":";
    RCLCPP_DEBUG(rclcpp::get_logger("Param:"), "%s", str.c_str());
    RCLCPP_DEBUG(rclcpp::get_logger("Param:"), " - axle_width_: %f", axle_width_);
    RCLCPP_DEBUG(rclcpp::get_logger("Param:"), " - lever_arm_length_: %f", lever_arm_length_);
    RCLCPP_DEBUG(rclcpp::get_logger("Param:"), " - axle_factor: %i", axle_factor_);
    RCLCPP_DEBUG(rclcpp::get_logger("Param:"), " - is steering: L: %i | R: %i", wheel_l_.isSteering(), wheel_r_.isSteering());
    wheel_l_.printInfo();
    wheel_r_.printInfo();
}

template<class WheelType>
void Axle<WheelType>::getSlipAngles(const State &x, const Input &u, double &alphaL, double &alphaR) 
{
    double v_x = std::max(1.0, x.v_x);
    alphaL = std::atan((x.v_y + axle_factor_ * lever_arm_length_ * x.r) / (v_x - 0.5 * axle_width_ * x.r)) - u.delta * wheel_l_.isSteering();
    alphaR = std::atan((x.v_y + axle_factor_ * lever_arm_length_ * x.r) / (v_x + 0.5 * axle_width_ * x.r)) - u.delta * wheel_r_.isSteering();
}

template<class WheelType>
void Axle<WheelType>::getFy(const State &x, const Input &u, const double Fz, AxleTires &Fy, AxleTires *alpha) 
{
    double alphaL, alphaR;
    getSlipAngles(x, u, alphaL, alphaR);

    const double Fz_axle = getDownForce(Fz);

    Fy.left  = wheel_l_.getFy(alphaL, Fz_axle);
    Fy.right = wheel_r_.getFy(alphaR, Fz_axle);

    if (alpha != nullptr) 
    {
        alpha->left  = alphaL;
        alpha->right = alphaR;
    }
}

template<class WheelType>
void Axle<WheelType>::setSteering(const double delta) 
{
    wheel_l_.setAngle(delta);
    wheel_r_.setAngle(delta);
}

template<class WheelType>
void Axle<WheelType>::setLeverArm(const double _car_length, const double _weight_factor, const double _width) 
{
    car_length_       = _car_length;
    lever_arm_length_ = car_length_ * _weight_factor;
    weight_factor_    = _weight_factor;
    axle_width_       = _width;
}

template<class WheelType>
double Axle<WheelType>::getDownForce(const double Fz) 
{
    double FzAxle = 0.5 * (1.0 - weight_factor_) * Fz;
    return FzAxle;
}

template<class WheelType>
void Axle<WheelType>::setParam(const Param param) 
{
    wheel_l_.setParam(param.tire);
    wheel_r_.setParam(param.tire);
}

template
class Axle<WheelStearing>;
template
class Axle<Wheel>;

}  // namespace cqut
}  // namespace gazebo_plugins
