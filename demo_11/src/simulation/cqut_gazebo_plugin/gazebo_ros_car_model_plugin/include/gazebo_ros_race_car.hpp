#pragma once

// Structure holding run-time important objects
#include "gazebo_ros_car_model_plugin/include/gazebo_race_car_private.hpp"

namespace gazebo_plugins 
{

class RaceCarModelPlugin : public gazebo::ModelPlugin 
{
public:

    RaceCarModelPlugin();

    ~RaceCarModelPlugin() override;

    void Reset() override;

    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

private:

    void update();

    void publishInfo();

    bool isLoopTime(const gazebo::common::Time &time, double &dt);

    std::unique_ptr<gazebo_race_car_private> dataPtr;
};

} // namespace gazebo_plugins
