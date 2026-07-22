#ifndef MY_FIRST_AGV_CORRIDOR_CONTROLLER_HPP
#define MY_FIRST_AGV_CORRIDOR_CONTROLLER_HPP

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"

// 引入自定义消息类型
#include "my_first_agv/msg/obstacle_warning.hpp"

namespace my_first_agv
{

/**
 * @brief 控制器状态枚举
 * 
 * 状态机包含三个状态：
 * - NORMAL: 正常沿路径行驶
 * - STOPPED: 检测到障碍物，已停车
 * - WARNING: 障碍物持续时间超过阈值，发布警告
 */
enum class ControllerState
{
  NORMAL,    // 正常行驶
  STOPPED,   // 遇到障碍物，已停车
  WARNING    // 障碍物持续超时，发布警告
};

/**
 * @class CorridorController
 * @brief 走廊导航控制器插件
 * 
 * 功能特性：
 * 1. 沿全局路径进行路径跟踪（Pure Pursuit 算法）
 * 2. 前方障碍物检测与立即停车（180° 扇形区域，0.1-3.0m 可调）
 * 3. 障碍物持续时间监控与警告发布（默认 60 秒）
 * 4. 严格禁止重规划和绕行行为
 * 
 * 参数配置：
 * - lookahead_dist: 前视距离（double，默认 0.5m）
 * - max_linear_speed: 最大线速度（double，默认 0.22 m/s）
 * - min_linear_speed: 最小线速度（double，默认 0.05 m/s）
 * - max_angular_speed: 最大角速度（double，默认 1.0 rad/s）
 * - obstacle_detect_range_min: 障碍物检测最小距离（double，默认 0.1m）
 * - obstacle_detect_range_max: 障碍物检测最大距离（double，默认 0.5m）
 * - obstacle_detect_angle: 障碍物检测角度（double，默认 3.14159 rad = 180°）
 * - obstacle_warning_duration: 障碍物警告超时时间（double，默认 60.0s）
 */
class CorridorController : public nav2_core::Controller
{
public:
  /**
   * @brief 默认构造函数
   */
  CorridorController() = default;

  /**
   * @brief 析构函数
   */
  ~CorridorController() override = default;

  /**
   * @brief 配置控制器
   * 
   * 在控制器加载时被调用，负责：
   * 1. 保存节点、TF缓冲区和 costmap 引用
   * 2. 声明并读取 ROS 参数
   * 3. 创建警告消息发布者
   * 4. 初始化状态机
   * 
   * @param parent 父节点（生命周期节点）
   * @param name 控制器名称
   * @param tf_buffer TF 缓冲区指针
   * @param costmap_ros costmap ROS 包装器指针
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 清理资源
   * 
   * 在控制器卸载时被调用，释放所有资源
   */
  void cleanup() override;

  /**
   * @brief 激活控制器
   * 
   * 在控制器启动时被调用，激活发布者等资源
   */
  void activate() override;

  /**
   * @brief 停用控制器
   * 
   * 在控制器停止时被调用，停用发布者等资源
   */
  void deactivate() override;

  /**
   * @brief 设置全局路径
   * 
   * 由 Nav2 框架调用，传入全局规划器生成的路径
   * 
   * @param path 全局路径
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief 计算速度指令
   * 
   * 核心控制循环，每次调用（20Hz）执行：
   * 1. 在路径上找到前视点
   * 2. 使用 Pure Pursuit 算法计算期望速度
   * 3. 检测前方障碍物
   * 4. 根据状态机逻辑决定最终速度
   * 
   * @param pose 当前机器人位姿
   * @param velocity 当前机器人速度
   * @param goal_checker 目标检查器指针
   * @return 计算出的速度指令
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief 设置速度限制
   * 
   * @param speed_limit 速度限制值
   * @param percentage 是否为百分比限制
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  /**
   * @brief 在路径上查找前视点
   * 
   * 从机器人当前位置开始，沿路径向前搜索，
   * 找到距离机器人 lookahead_dist 的点作为前视点
   * 
   * @param pose 当前机器人位姿
   * @param path 全局路径
   * @return 前视点（世界坐标系）
   */
  geometry_msgs::msg::Point getLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_msgs::msg::Path & path);

  /**
   * @brief 检测前方是否有障碍物
   * 
   * 在机器人前方扇形区域内扫描 costmap，
   * 检测是否存在致命障碍（LETHAL_OBSTACLE）
   * 
   * @param pose 当前机器人位姿
   * @return true 检测到障碍物，false 未检测到
   */
  bool checkObstacleAhead(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief 激光雷达回调函数
   *
   * 接收 /scan 话题数据并保存最新一帧激光扫描，
   * 用于直接检测前方障碍物（不依赖 costmap 清除）。
   *
   * @param msg 激光扫描消息
   */
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief 使用激光雷达检测前方障碍物
   *
   * 直接分析 /scan 数据，检测机器人前方扇形区域内
   * 是否有距离在 [min, max] 范围内的障碍物。
   *
   * @param pose 当前机器人位姿
   * @return true 检测到障碍物，false 未检测到
   */
  bool checkObstacleByLaser(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief 发布障碍物警告消息
   * 
   * 构造并发布 ObstacleWarning 消息到 /obstacle_warning 话题
   * 
   * @param obstacle_pos 障碍物位置
   * @param duration 持续时间
   */
  void publishObstacleWarning(
    const geometry_msgs::msg::Point & obstacle_pos,
    double duration);

  // 节点和框架相关
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("corridor_controller")};

  // 发布者
  rclcpp_lifecycle::LifecyclePublisher<my_first_agv::msg::ObstacleWarning>::SharedPtr
    warning_pub_;

  // 激光雷达订阅者
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  // 最新激光扫描数据
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::mutex scan_mutex_;

  // 全局路径
  nav_msgs::msg::Path global_plan_;
  std::mutex plan_mutex_;

  // 控制参数
  double lookahead_dist_;              // 前视距离（米）
  double max_linear_speed_;            // 最大线速度（m/s）
  double min_linear_speed_;            // 最小线速度（m/s）
  double max_angular_speed_;           // 最大角速度（rad/s）
  double speed_limit_;                 // 速度限制

  // 障碍物检测参数
  double obstacle_detect_range_min_;   // 检测最小距离（米）
  double obstacle_detect_range_max_;   // 检测最大距离（米）
  double obstacle_detect_angle_;       // 检测角度（弧度）
  double obstacle_warning_duration_;   // 警告超时时间（秒）

  // 状态机
  ControllerState current_state_;      // 当前状态
  rclcpp::Time obstacle_start_time_;   // 障碍物开始时间
  bool obstacle_detected_;             // 当前是否检测到障碍物
  geometry_msgs::msg::Point last_obstacle_pos_;  // 最近障碍物位置
};

}  // namespace my_first_agv

#endif  // MY_FIRST_AGV_CORRIDOR_CONTROLLER_HPP
