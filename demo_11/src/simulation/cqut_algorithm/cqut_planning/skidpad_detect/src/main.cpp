#include "skidpad_detect/skidpad_detector_handle.hpp"

typedef ns_skidpad_detector::SkidpadDetectorHandle SkidpadDetectorHandle;

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared("skidpad_detector");
    SkidpadDetectorHandle mySkidpadDetectorHandle(nodeHandle);
    rclcpp::Rate loop_rate(mySkidpadDetectorHandle.getNodeRate());
    while (rclcpp::ok()) 
    {
        mySkidpadDetectorHandle.run();

        rclcpp::spin_some(nodeHandle);         
        loop_rate.sleep();            
    }
  return 0;
}

