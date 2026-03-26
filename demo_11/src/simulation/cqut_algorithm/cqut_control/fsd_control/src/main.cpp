#include <rclcpp/rclcpp.hpp>
#include "fsd_control/control_handle.hpp"

typedef ns_control::ControlHandle ControlHandle;

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared("control");
    ControlHandle myControlHandle(nodeHandle);
    rclcpp::Rate loop_rate(myControlHandle.getNodeRate());
    while (rclcpp::ok()) 
    {

        myControlHandle.run();

        rclcpp::spin_some(nodeHandle);
        loop_rate.sleep();             
    }
    return 0;
}

