#include <string>
#include <opencv2/imgproc.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "boundary_detect/type.hpp"
#include "cqut_msg/msg/map.hpp"

namespace FSD 
{

void visual(cv::Subdiv2D coneSet, SearchTree Path, 
            cqut_msg::msg::Map boundaryDetections, 
            std::vector<PathPoint> BestPath,
            visualization_msgs::msg::Marker &visualTriangles, 
            visualization_msgs::msg::MarkerArray &visualTree, 
            visualization_msgs::msg::MarkerArray &visualBoundary,
            visualization_msgs::msg::Marker &visualPath);

};