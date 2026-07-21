#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("agv_talker_node");
    auto loop_rate = rclcpp::Rate(1.0);
    int count = 0;
    while (rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "中文 vibe coding ");
        count++;
        if (count>=10)
        {
            break;
        }
        
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;

}