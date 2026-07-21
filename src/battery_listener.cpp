#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"




void battery_callback(const std_msgs::msg::Int32::SharedPtr msg);

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("battery_monitor");
    auto subscriber = node->create_subscription<std_msgs::msg::Int32>("battery_level", 10, battery_callback);
    RCLCPP_INFO(node->get_logger(), "电池监控节点已启动");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}

void battery_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int32_t battery_level = msg->data;
    if (battery_level<=20)
    {
        RCLCPP_WARN(rclcpp::get_logger("battery_alert"), 
                    "🚨 警告！警告！电量过低！当前：%d %%", battery_level);
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("battery_monitor"), 
                    "✅ 电量正常！当前：%d %%", battery_level);
    }


}

