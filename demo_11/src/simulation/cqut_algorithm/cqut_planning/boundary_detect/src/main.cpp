#include <rclcpp/rclcpp.hpp>
#include "boundary_detect/boundaryDetector_handle.hpp"

typedef ns_boundaryDetector::BoundaryDetectorHandle BoundaryDetectorHandle;

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared("boundaryDetector");
    BoundaryDetectorHandle myBoundaryDetectorHandle(nodeHandle);
    rclcpp::Rate loop_rate(myBoundaryDetectorHandle.getNodeRate());
    while (rclcpp::ok()) 
    {
        myBoundaryDetectorHandle.run();

        rclcpp::spin_some(nodeHandle);      
        loop_rate.sleep();     
    }
    return 0;
}

