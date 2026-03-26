// Main Include
#include "gazebo_ros_car_model_plugin/include/gazebo_ros_race_car.hpp"

// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

// Gazebo
#include <gazebo_ros/node.hpp>

namespace gazebo_plugins 
 {

RaceCarModelPlugin::RaceCarModelPlugin() : dataPtr(new gazebo_race_car_private) 
{
}

RaceCarModelPlugin::~RaceCarModelPlugin() 
{
    this->dataPtr->updateConnection.reset();
}

void RaceCarModelPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
{
    this->dataPtr->rosnode = gazebo_ros::Node::Get(_sdf, "RaceCarModelPlugin");
    RCLCPP_INFO(this->dataPtr->rosnode->get_logger(), "Loading RaceCarModelPlugin");
    RCLCPP_INFO(this->dataPtr->rosnode->get_logger(), "RaceCarModelPlugin loading params");

    this->dataPtr->model = _model;
    this->dataPtr->world = this->dataPtr->model->GetWorld();

    this->dataPtr->gznode = gazebo::transport::NodePtr(new gazebo::transport::Node());

    this->dataPtr->gznode->Init();

    this->dataPtr->vehicle = std::unique_ptr<cqut::Vehicle>(new cqut::Vehicle(_model, _sdf, this->dataPtr->rosnode, this->dataPtr->gznode));
    this->dataPtr->vehicle->printInfo();

    this->dataPtr->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RaceCarModelPlugin::update, this));

    this->dataPtr->worldControlPub = this->dataPtr->gznode->Advertise<gazebo::msgs::WorldControl>("~/world_control");

    this->dataPtr->lastSimTime = this->dataPtr->world->SimTime();
}

void RaceCarModelPlugin::Reset() 
{
    this->dataPtr->lastSimTime = 0;
}

void RaceCarModelPlugin::update() 
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    publishInfo();

    gazebo::common::Time curTime = this->dataPtr->world->SimTime();
    double dt = 0.0;
    if (!isLoopTime(this->dataPtr->world->SimTime(), dt)) 
    {
        return;
    }

    this->dataPtr->lastSimTime = curTime;
    this->dataPtr->vehicle->update(dt);
}

void RaceCarModelPlugin::publishInfo() 
{
    this->dataPtr->vehicle->publish(this->dataPtr->world->SimTime().Double());
}

bool RaceCarModelPlugin::isLoopTime(const gazebo::common::Time &time, double &dt) 
{
    dt = (time - this->dataPtr->lastSimTime).Double();
    if (dt < 0.0) 
    {
        this->Reset();
        return false;
    } 
    else if (ignition::math::equal(dt, 0.0)) 
    {
        return false;
    }
    return true;
}

GZ_REGISTER_MODEL_PLUGIN(RaceCarModelPlugin)

} // namespace gazebo_plugins

