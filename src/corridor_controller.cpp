/**
 * @file corridor_controller.cpp
 * @brief 走廊导航控制器插件实现
 * 
 * 本文件实现了 CorridorController 类，提供以下核心功能：
 * 1. 沿全局路径进行路径跟踪（Pure Pursuit 算法）
 * 2. 前方障碍物检测与立即停车
 * 3. 障碍物持续时间监控与警告发布
 * 4. 严格禁止重规划和绕行行为
 */

#include "my_first_agv/corridor_controller.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include <cmath>
#include <algorithm>

// 使用命名空间简化代码
using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace my_first_agv
{

/**
 * @brief 配置控制器
 * 
 * 在控制器加载时被调用，负责：
 * 1. 保存节点、TF缓冲区和 costmap 引用
 * 2. 声明并读取 ROS 参数
 * 3. 创建警告消息发布者
 * 4. 初始化状态机
 */
void CorridorController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // 保存引用
  node_ = parent;
  plugin_name_ = name;
  tf_buffer_ = tf_buffer;
  costmap_ros_ = costmap_ros;
  
  // 获取节点指针
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("CorridorController: Failed to lock node");
  }
  
  // 设置日志器
  logger_ = node->get_logger();
  
  RCLCPP_INFO(logger_, "Configuring CorridorController: %s", name.c_str());
  
  // 声明并读取参数
  // 注意：这些参数需要在 my_nav_params.yaml 中配置
  
  // 路径跟踪参数
  node->declare_parameter(plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  
  node->declare_parameter(plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.22));
  node->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  
  node->declare_parameter(plugin_name_ + ".min_linear_speed", rclcpp::ParameterValue(0.05));
  node->get_parameter(plugin_name_ + ".min_linear_speed", min_linear_speed_);
  
  node->declare_parameter(plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0));
  node->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  
  // 障碍物检测参数
  node->declare_parameter(plugin_name_ + ".obstacle_detect_range_min", rclcpp::ParameterValue(0.1));
  node->get_parameter(plugin_name_ + ".obstacle_detect_range_min", obstacle_detect_range_min_);
  
  node->declare_parameter(plugin_name_ + ".obstacle_detect_range_max", rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name_ + ".obstacle_detect_range_max", obstacle_detect_range_max_);
  
  node->declare_parameter(plugin_name_ + ".obstacle_detect_angle", rclcpp::ParameterValue(1.0));
  node->get_parameter(plugin_name_ + ".obstacle_detect_angle", obstacle_detect_angle_);
  
  node->declare_parameter(plugin_name_ + ".obstacle_warning_duration", rclcpp::ParameterValue(60.0));
  node->get_parameter(plugin_name_ + ".obstacle_warning_duration", obstacle_warning_duration_);
  
  // 创建警告消息发布者
  warning_pub_ = node->create_publisher<my_first_agv::msg::ObstacleWarning>(
    "/obstacle_warning", rclcpp::QoS(10));

  // 订阅激光雷达数据，用于直接检测障碍物
  laser_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&CorridorController::laserCallback, this, std::placeholders::_1));

  // 初始化状态机
  current_state_ = ControllerState::NORMAL;
  obstacle_detected_ = false;
  obstacle_start_time_ = node->now();
  speed_limit_ = max_linear_speed_;
  
  // 输出配置信息
  RCLCPP_INFO(logger_, 
    "CorridorController configured:\n"
    "  lookahead_dist: %.2f m\n"
    "  max_linear_speed: %.2f m/s\n"
    "  obstacle_detect_range: [%.2f, %.2f] m\n"
    "  obstacle_detect_angle: %.2f rad (%.1f°)\n"
    "  obstacle_warning_duration: %.1f s",
    lookahead_dist_,
    max_linear_speed_,
    obstacle_detect_range_min_, obstacle_detect_range_max_,
    obstacle_detect_angle_, obstacle_detect_angle_ * 180.0 / M_PI,
    obstacle_warning_duration_);
}

/**
 * @brief 清理资源
 */
void CorridorController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up CorridorController");
  warning_pub_.reset();
  laser_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_.reset();
  }
}

/**
 * @brief 激活控制器
 */
void CorridorController::activate()
{
  RCLCPP_INFO(logger_, "Activating CorridorController");
  warning_pub_->on_activate();
}

/**
 * @brief 停用控制器
 */
void CorridorController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating CorridorController");
  warning_pub_->on_deactivate();
}

/**
 * @brief 设置全局路径
 */
void CorridorController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  global_plan_ = path;
  RCLCPP_DEBUG(logger_, "Received new global plan with %zu points", path.poses.size());
  
  // 重置状态机
  current_state_ = ControllerState::NORMAL;
  obstacle_detected_ = false;
}

/**
 * @brief 在路径上查找前视点
 * 
 * 算法说明：
 * 1. 从机器人当前位置开始，沿路径向前搜索
 * 2. 找到距离机器人 lookahead_dist 的点作为前视点
 * 3. 如果路径太短，返回路径末端点
 */

 /* -----------------------------原Trae纯追踪算法代码-----------------*/
geometry_msgs::msg::Point CorridorController::getLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path)
{
  // 如果路径为空，返回当前位置
  if (path.poses.empty()) {
    return pose.pose.position;
  }
  
  // 获取机器人当前位置
  double robot_x = pose.pose.position.x;
  double robot_y = pose.pose.position.y;
  
  // 找到路径上距离机器人最近的点
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_idx = 0;
  
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - robot_x;
    double dy = path.poses[i].pose.position.y - robot_y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  
  // 从最近点开始，沿路径向前搜索，找到距离 >= lookahead_dist 的点
  for (size_t i = closest_idx; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - robot_x;
    double dy = path.poses[i].pose.position.y - robot_y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    if (dist >= lookahead_dist_) {
      return path.poses[i].pose.position;
    }
  }
  
  // 如果路径太短，返回路径末端点
  return path.poses.back().pose.position;
}

/**
 * @brief 检测前方是否有障碍物
 * 
 * 算法说明：
 * 1. 获取机器人当前位姿（位置和朝向）
 * 2. 在机器人前方扇形区域内扫描 costmap
 * 3. 扇形区域参数：
 *    - 距离范围：[obstacle_detect_range_min_, obstacle_detect_range_max_]
 *    - 角度范围：[-obstacle_detect_angle_/2, +obstacle_detect_angle_/2]
 * 4. 如果检测到致命障碍（LETHAL_OBSTACLE），返回 true
 */

 /*--------------------原检测障碍物逻辑代码--------------------*/
bool CorridorController::checkObstacleAhead(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // 获取 costmap 指针
  auto costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());

  // 获取机器人当前位置和朝向
  double robot_x = pose.pose.position.x;
  double robot_y = pose.pose.position.y;
  double robot_yaw = tf2::getYaw(pose.pose.orientation);

  // 获取 costmap 分辨率
  double resolution = costmap->getResolution();

  // 在扇形区域内扫描
  // 使用极坐标扫描：距离从 min 到 max，角度从 -angle/2 到 +angle/2
  double angle_step = 0.1;  // 角度步长（弧度）
  double dist_step = resolution;  // 距离步长（与 costmap 分辨率相同）

  // 用于调试：记录检测到的最大代价值和位置
  unsigned char max_cost = 0;
  double max_cost_x = robot_x;
  double max_cost_y = robot_y;

  for (double dist = obstacle_detect_range_min_;
       dist <= obstacle_detect_range_max_;
       dist += dist_step)
  {
    for (double angle = -obstacle_detect_angle_ / 2.0;
         angle <= obstacle_detect_angle_ / 2.0;
         angle += angle_step)
    {
      // 计算检测点的世界坐标
      double check_x = robot_x + dist * std::cos(robot_yaw + angle);
      double check_y = robot_y + dist * std::sin(robot_yaw + angle);

      // 将世界坐标转换为 costmap 栅格坐标
      unsigned int mx, my;
      if (!costmap->worldToMap(check_x, check_y, mx, my)) {
        continue;  // 如果超出 costmap 范围，跳过
      }

      // 获取该栅格的代价值
      unsigned char cost = costmap->getCost(mx, my);

      // 记录最大代价值（用于调试）
      if (cost > max_cost) {
        max_cost = cost;
        max_cost_x = check_x;
        max_cost_y = check_y;
      }

      // 如果是致命障碍，说明检测到障碍物
      if (cost >= LETHAL_OBSTACLE) {
        // 记录障碍物位置（用于警告消息）
        last_obstacle_pos_.x = check_x;
        last_obstacle_pos_.y = check_y;
        last_obstacle_pos_.z = 0.0;
        RCLCPP_DEBUG(logger_,
          "检测到障碍物：位置 (%.2f, %.2f)，代价值 %d",
          check_x, check_y, static_cast<int>(cost));
        return true;
      }
    }
  }

  // 未检测到障碍物时，输出调试信息（每 2 秒一次，避免刷屏）
  static rclcpp::Time last_debug_time = node_.lock()->now();
  rclcpp::Time now = node_.lock()->now();
  if ((now - last_debug_time).seconds() > 2.0) {
    RCLCPP_INFO(logger_,
      "前方无障碍：机器人 (%.2f, %.2f)，扇形内最大代价值 %d（位置 %.2f, %.2f）",
      robot_x, robot_y, static_cast<int>(max_cost), max_cost_x, max_cost_y);
    last_debug_time = now;
  }

  return false;  // 未检测到障碍物
}

/**
 * @brief 激光雷达回调函数
 *
 * 保存最新一帧激光扫描数据。
 */
void CorridorController::laserCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = msg;
}

/**
 * @brief 使用激光雷达检测前方障碍物
 *
 * 直接分析 /scan 数据，检测机器人前方扇形区域内
 * 是否有距离在 [min, max] 范围内的障碍物。
 * 不依赖 costmap，障碍物移走后可立即恢复。
 */
bool CorridorController::checkObstacleByLaser(
  const geometry_msgs::msg::PoseStamped & pose)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);

  // 如果没有收到激光数据，回退到 costmap 检测
  if (!latest_scan_) {
    return checkObstacleAhead(pose);
  }

  double robot_x = pose.pose.position.x;
  double robot_y = pose.pose.position.y;
  double robot_yaw = tf2::getYaw(pose.pose.orientation);

  const auto & scan = *latest_scan_;
  double angle_min = scan.angle_min;
  double angle_increment = scan.angle_increment;
  double range_min = scan.range_min;
  double range_max = scan.range_max;

  // 用于调试：记录扇形内最小距离
  double min_range_in_fov = std::numeric_limits<double>::max();
  double min_range_angle = 0.0;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];

    // 跳过无效值
    if (std::isnan(range) || std::isinf(range)) {
      continue;
    }

    // 跳过超出激光有效范围的值
    if (range < range_min || range > range_max) {
      continue;
    }

    // 计算该激光束相对于机器人朝向的角度
    double beam_angle = angle_min + i * angle_increment;

    // 只考虑前方扇形区域内的激光束
    if (std::abs(beam_angle) > obstacle_detect_angle_ / 2.0) {
      continue;
    }

    // 记录扇形内最小距离（用于调试）
    if (range < min_range_in_fov) {
      min_range_in_fov = range;
      min_range_angle = beam_angle;
    }

    // 如果距离在检测范围内，认为检测到障碍物
    if (range >= obstacle_detect_range_min_ && range <= obstacle_detect_range_max_) {
      // 计算障碍物在世界坐标系中的位置
      double obstacle_x = robot_x + range * std::cos(robot_yaw + beam_angle);
      double obstacle_y = robot_y + range * std::sin(robot_yaw + beam_angle);

      last_obstacle_pos_.x = obstacle_x;
      last_obstacle_pos_.y = obstacle_y;
      last_obstacle_pos_.z = 0.0;

      RCLCPP_DEBUG(logger_,
        "激光检测到障碍物：距离 %.2f m，角度 %.2f rad，位置 (%.2f, %.2f)",
        range, beam_angle, obstacle_x, obstacle_y);
      return true;
    }
  }

  // 未检测到障碍物，输出调试信息（每 2 秒一次）
  static rclcpp::Time last_laser_debug_time = node_.lock()->now();
  rclcpp::Time now = node_.lock()->now();
  if ((now - last_laser_debug_time).seconds() > 2.0) {
    RCLCPP_INFO(logger_,
      "激光前方无障碍：机器人 (%.2f, %.2f)，扇形内最小距离 %.2f m（角度 %.2f rad）",
      robot_x, robot_y, min_range_in_fov, min_range_angle);
    last_laser_debug_time = now;
  }

  return false;
}

/**
 * @brief 发布障碍物警告消息
 */
void CorridorController::publishObstacleWarning(
  const geometry_msgs::msg::Point & obstacle_pos,
  double duration)
{
  auto msg = my_first_agv::msg::ObstacleWarning();
  
  // 设置消息头
  msg.header.stamp = node_.lock()->now();
  msg.header.frame_id = "map";
  
  // 设置障碍物位置
  msg.obstacle_position = obstacle_pos;
  
  // 设置持续时间
  msg.duration = duration;
  
  // 设置建议操作
  msg.suggested_action = "等待障碍物移开，禁止重规划或绕行";
  
  // 设置阻塞状态
  msg.is_blocking = true;
  
  // 发布消息
  warning_pub_->publish(msg);
  
  // 同时在终端输出警告日志
  RCLCPP_WARN(logger_, 
    "⚠️ 障碍物警告：位置 (%.2f, %.2f)，持续时间 %.1f 秒，%s",
    obstacle_pos.x, obstacle_pos.y, duration, msg.suggested_action.c_str());
}

/**
 * @brief 计算速度指令
 * 
 * 核心控制循环，每次调用（20Hz）执行：
 * 1. 在路径上找到前视点
 * 2. 使用 Pure Pursuit 算法计算期望速度
 * 3. 检测前方障碍物
 * 4. 根据状态机逻辑决定最终速度
 * 
 * 状态机逻辑：
 * - NORMAL: 正常行驶，持续检测障碍物
 * - STOPPED: 检测到障碍物，停车并计时
 * - WARNING: 障碍物持续超时，发布警告
 */
geometry_msgs::msg::TwistStamped CorridorController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  // 获取节点指针
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("CorridorController: Failed to lock node");
  }
  
  // 创建速度指令消息
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "base_link";
  cmd_vel.header.stamp = node->now();
  
  // 获取全局路径（加锁）
  nav_msgs::msg::Path current_plan;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    current_plan = global_plan_;
  }
  
  // 如果路径为空，抛出异常
  if (current_plan.poses.empty()) {
    throw std::runtime_error("CorridorController: Global plan is empty");
  }
  
 // 优先使用激光雷达直接检测障碍物，避免 costmap 清除延迟导致无法恢复
	 bool obstacle_now = checkObstacleByLaser(pose);

  // 状态机逻辑
  switch (current_state_) {
    case ControllerState::NORMAL: {
      if (obstacle_now) {
        // 检测到障碍物，切换到 STOPPED 状态
        current_state_ = ControllerState::STOPPED;
        obstacle_detected_ = true;
        obstacle_start_time_ = node->now();
        RCLCPP_INFO(logger_, "🛑 检测到前方障碍物，停车等待");
        
        // 返回零速度
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
      } else {
        // 正常行驶，使用 Pure Pursuit 算法计算速度
        
        // 1. 找到前视点
        geometry_msgs::msg::Point lookahead_point = getLookaheadPoint(pose, current_plan);
        
        // 2. 将前视点转换到机器人坐标系
        double robot_x = pose.pose.position.x;
        double robot_y = pose.pose.position.y;
        double robot_yaw = tf2::getYaw(pose.pose.orientation);
        
        // 计算前视点相对于机器人的位置
        double dx = lookahead_point.x - robot_x;
        double dy = lookahead_point.y - robot_y;
        
        // 转换到机器人坐标系
        double local_y = std::sin(-robot_yaw) * dx + std::cos(-robot_yaw) * dy;
        
        // 3. 使用 Pure Pursuit 算法计算曲率
        // 曲率 kappa = 2 * y / L^2，其中 L 是前视距离
        double L_sq = lookahead_dist_ * lookahead_dist_;
        double kappa = 2.0 * local_y / L_sq;
        
        // 4. 计算角速度
        double angular_vel = kappa * max_linear_speed_;
        angular_vel = std::clamp(angular_vel, -max_angular_speed_, max_angular_speed_);

        
        // 5. 计算线速度（根据角速度调整）
        double linear_vel = max_linear_speed_;
        if (std::abs(angular_vel) > max_angular_speed_ * 0.8) {
          // 如果角速度较大，降低线速度
          linear_vel = min_linear_speed_;
        }
        
        // 应用速度限制
        linear_vel = std::min(linear_vel, speed_limit_);
        
        // 6. 设置速度指令
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        
        return cmd_vel;
      }
    }
    
    case ControllerState::STOPPED: {
      if (!obstacle_now) {
        // 障碍物消失，切换到 NORMAL 状态
        current_state_ = ControllerState::NORMAL;
        obstacle_detected_ = false;
        RCLCPP_INFO(logger_, "✅ 障碍物已消失，从 STOPPED 恢复正常行驶");

        // 继续正常行驶逻辑
        geometry_msgs::msg::Point lookahead_point = getLookaheadPoint(pose, current_plan);
        double robot_x = pose.pose.position.x;
        double robot_y = pose.pose.position.y;
        double robot_yaw = tf2::getYaw(pose.pose.orientation);

        double dx = lookahead_point.x - robot_x;
        double dy = lookahead_point.y - robot_y;
        double local_y = std::sin(-robot_yaw) * dx + std::cos(-robot_yaw) * dy;

        double L_sq = lookahead_dist_ * lookahead_dist_;
        double kappa = 2.0 * local_y / L_sq;
        double angular_vel = kappa * max_linear_speed_;
        angular_vel = std::clamp(angular_vel, -max_angular_speed_, max_angular_speed_);

        double linear_vel = max_linear_speed_;
        if (std::abs(angular_vel) > max_angular_speed_ * 0.8) {
          linear_vel = min_linear_speed_;
        }
        linear_vel = std::min(linear_vel, speed_limit_);

        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
      } else {
        // 障碍物仍然存在，检查是否超时
        double duration = (node->now() - obstacle_start_time_).seconds();

        // 每 2 秒输出一次停车等待日志
        static rclcpp::Time last_stopped_log_time = node->now();
        if ((node->now() - last_stopped_log_time).seconds() > 2.0) {
          RCLCPP_INFO(logger_,
            "🛑 停车等待中：障碍物已持续 %.1f 秒，位置 (%.2f, %.2f)",
            duration, last_obstacle_pos_.x, last_obstacle_pos_.y);
          last_stopped_log_time = node->now();
        }

        if (duration >= obstacle_warning_duration_) {
          // 超时，切换到 WARNING 状态
          current_state_ = ControllerState::WARNING;
          RCLCPP_WARN(logger_,
            "⚠️ 障碍物持续时间超过 %.1f 秒，进入警告状态",
            obstacle_warning_duration_);

          // 发布警告消息
          publishObstacleWarning(last_obstacle_pos_, duration);
        }

        // 返回零速度（继续停车）
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
      }
    }
    
    case ControllerState::WARNING: {
      if (!obstacle_now) {
        // 障碍物消失，切换到 NORMAL 状态
        current_state_ = ControllerState::NORMAL;
        obstacle_detected_ = false;
        RCLCPP_INFO(logger_, "✅ 障碍物已消失，从 WARNING 恢复正常行驶");

        // 继续正常行驶逻辑（与 STOPPED 状态中的代码相同）
        geometry_msgs::msg::Point lookahead_point = getLookaheadPoint(pose, current_plan);
        double robot_x = pose.pose.position.x;
        double robot_y = pose.pose.position.y;
        double robot_yaw = tf2::getYaw(pose.pose.orientation);

        double dx = lookahead_point.x - robot_x;
        double dy = lookahead_point.y - robot_y;
        double local_y = std::sin(-robot_yaw) * dx + std::cos(-robot_yaw) * dy;

        double L_sq = lookahead_dist_ * lookahead_dist_;
        double kappa = 2.0 * local_y / L_sq;
        double angular_vel = kappa * max_linear_speed_;
        angular_vel = std::clamp(angular_vel, -max_angular_speed_, max_angular_speed_);

        double linear_vel = max_linear_speed_;
        if (std::abs(angular_vel) > max_angular_speed_ * 0.8) {
          linear_vel = min_linear_speed_;
        }
        linear_vel = std::min(linear_vel, speed_limit_);

        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
      } else {
        // 障碍物仍然存在，继续发布警告
        double duration = (node->now() - obstacle_start_time_).seconds();
        publishObstacleWarning(last_obstacle_pos_, duration);

        // 返回零速度（继续停车）
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
      }
    }
  }
  
  // 默认返回零速度（理论上不会到达这里）
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  return cmd_vel;
}

/**
 * @brief 设置速度限制
 */
void CorridorController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (percentage) {
    // 百分比限制：speed_limit 是 0-100 的值
    speed_limit_ = max_linear_speed_ * (speed_limit / 100.0);
  } else {
    // 绝对值限制：speed_limit 是 m/s
    speed_limit_ = speed_limit;
  }
  
  // 确保速度限制在合理范围内
  speed_limit_ = std::clamp(speed_limit_, min_linear_speed_, max_linear_speed_);
  
  RCLCPP_DEBUG(logger_, "Speed limit set to %.2f m/s", speed_limit_);
}

}  // namespace my_first_agv

// 注册插件（让 pluginlib 能够加载这个类）
PLUGINLIB_EXPORT_CLASS(my_first_agv::CorridorController, nav2_core::Controller)
