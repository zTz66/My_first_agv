#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("battery_station");
    auto publisher = node->create_publisher<std_msgs::msg::Int32>("battery_level", 10);
    auto loop_rate = rclcpp::Rate(1.0);

    int battery_value = 50;
    while (rclcpp::ok())
    {
        auto message = std_msgs::msg::Int32();
        message.data = battery_value;
        publisher->publish(message);
        RCLCPP_INFO(node->get_logger(), "battery level: %d", battery_value);
        battery_value--;
        if (battery_value<=0)
        {
            break;
        }
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}