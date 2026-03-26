#pragma once

#include <string>
#include <Eigen/Dense>
#include <tf2/LinearMath/Vector3.h>
#include <gazebo/physics/physics.hh>

namespace gazebo_plugins 
{

template<class VecOut, class VecIn>
VecOut getRosVesMsg(VecIn &in) 
{
    VecOut out;
    out.x = in.X();
    out.y = in.Y();
    out.z = in.Z();
    return out;
};

template<class Joint, class Model>
void getJoint(Joint &joint, Model &model, const std::string &name) 
{
    joint = model->GetJoint(name);
    if (!joint) 
    {
        gzerr << "could not find " << name << " joint" << std::endl;
    }
}

template<class Link, class Model>
void getLink(Link &link, Model &model, const std::string &name)
 {
    link = model->GetLink(name);
    if (!link) 
    {
        gzerr << "could not find " << name << " link" << std::endl;
    }
}

template<class type>
type getParam(sdf::ElementPtr &sdf, std::string paramName, type paramDefault) 
{
    type para;
    if (sdf->HasElement(paramName)) 
    {
        para = sdf->Get<type>(paramName);
    } 
    else 
    {
        gzwarn << "Setting default value for: " << paramName << ":" << paramDefault << std::endl;
        para = paramDefault;
    }
    return para;
}

template<class type>
type getParam(sdf::ElementPtr &sdf, std::string paramName) 
{
    type para;
    if (sdf->HasElement(paramName)) 
    {
        para = sdf->Get<type>(paramName);
    } 
    else 
    {
        para = type();
    }
    return para;
}

inline Eigen::Vector3d toVector(const boost::shared_ptr<gazebo::physics::Entity> &v) 
{
    return {v->WorldPose().Pos().X(), v->WorldPose().Pos().Y(), 0.0};
}

namespace noise
{
Eigen::Vector2d gaussiaNoise2D(double mu, double sigma);

Eigen::Vector3d gaussiaNoise3D(double mu, double sigma);

double getGaussianNoise(double mean, double var);

bool probability(const double likelihood, double &lik_res);
}

namespace ros_utils
{

inline tf2::Vector3 toVector(const Eigen::Vector3d &v) 
{
    tf2::Vector3 ret;
    ret.setX(v.x());
    ret.setY(v.y());
    ret.setZ(v.z());
    return ret;
}

inline Eigen::Vector3d toVector(const tf2::Vector3 &v) 
{
    return {v.x(), v.y(), v.z()};
}

}

}