// Main Include
#include "gazebo_ros_car_model_plugin/include/wheel.hpp"

namespace gazebo_plugins 
{

namespace cqut 
{

Wheel::Wheel(gazebo::physics::ModelPtr &_model,
             sdf::ElementPtr &_sdf,
             const std::string _name,
             gazebo::transport::NodePtr &gznode,
             rclcpp::Node::SharedPtr nh) : model_(_model) 
{
    full_name_ = _model->GetName() + "::" + _sdf->Get<std::string>(_name);
    gazebo::physics::JointPtr joint;

    getJoint(joint, _model, full_name_);
    unsigned int id = 0;
    radius = getCollisionRadius(joint->GetChild()->GetCollision(id));

    center_pos_ = joint->GetChild()->GetCollision(id)->WorldPose().Pos();
}

double Wheel::getCollisionRadius(gazebo::physics::CollisionPtr _coll) 
{
    if (!_coll || !(_coll->GetShape())) { return 0; }

    if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE)) 
    {
        gazebo::physics::CylinderShape *cyl = static_cast<gazebo::physics::CylinderShape *>(_coll->GetShape().get());
        return cyl->GetRadius();
    } 
    else if (_coll->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE)) 
    {
        gazebo::physics::SphereShape *sph = static_cast<gazebo::physics::SphereShape *>(_coll->GetShape().get());
        return sph->GetRadius();
    }
    return 0;
}

void Wheel::printInfo() 
{
    RCLCPP_DEBUG(rclcpp::get_logger("Param"), "Name: %s", full_name_.c_str());
}

double Wheel::getFy(const double alpha, const double Fz) 
{
    const double B    = param_.B;
    const double C    = param_.C;
    const double D    = param_.D;
    const double E    = param_.E;
    const double mu_y = D * std::sin(C * std::atan(B * (1.0 - E) * alpha + E * std::atan(B * alpha)));
    const double Fy   = Fz * mu_y;
    return Fy;
}

const ignition::math::Vector3<double> &Wheel::getCenterPos() const { return center_pos_; }

}  // namespace cqut
}  // namespace gazebo_plugins
