#include "path_generator/Track/track_base.h"

namespace ns_path_generator 
{

void Track::setMap(const cqut_msg::msg::Map &map) 
{ 
    map_ = map; 
}

void Track::setState(const VehicleState &state) 
{ 
    state_ = state; 
}

void Track::setEndPoint(const geometry_msgs::msg::Point &endPoint) 
{ 
    endPoint_ = endPoint; 
}

void Track::setTransMat(const Eigen::Matrix4f &transMat) 
{ 
    transMat_ = transMat; 
}

} // namespace ns_path_generator
