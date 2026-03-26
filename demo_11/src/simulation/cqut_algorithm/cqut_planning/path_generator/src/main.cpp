#include <rclcpp/rclcpp.hpp>
#include "path_generator/path_generator_handle.hpp"

typedef ns_path_generator::PathGeneratorHandle PathGeneratorHandle;

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared("path_generator");
    PathGeneratorHandle myPathGeneratorHandle(nodeHandle);
    rclcpp::Rate loop_rate(myPathGeneratorHandle.getNodeRate());
    while (rclcpp::ok()) 
    {
        myPathGeneratorHandle.run();

        rclcpp::spin_some(nodeHandle);
        loop_rate.sleep(); 
    }
    return 0;
}

