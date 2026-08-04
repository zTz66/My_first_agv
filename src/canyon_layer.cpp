/**
 * @file canyon_layer.cpp
 * @brief 走廊约束层插件实现
 * 
 * 本文件实现了 CanyonLayer 类，用于在 Nav2 costmap 中创建虚拟走廊边界。
 * 走廊外的区域会被标记为致命障碍，确保机器人严格限制在走廊内移动。
 */

#include "my_first_agv/canyon_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

// 使用命名空间，简化代码
using nav2_costmap_2d::LETHAL_OBSTACLE;  // 致命障碍，值为 254
using nav2_costmap_2d::FREE_SPACE;       // 自由空间，值为 0

namespace my_first_agv
{

/**
 * @brief 构造函数
 * 初始化所有成员变量为默认值
 */
CanyonLayer::CanyonLayer()
: corridor_enabled_(true),
  corridor_width_(1.5),
  corridor_half_width_(0.75),
  corridor_min_x_(0.0),
  corridor_min_y_(0.0),
  corridor_max_x_(0.0),
  corridor_max_y_(0.0)
{
}

/**
 * @brief 析构函数
 */
CanyonLayer::~CanyonLayer() = default;

/**
 * @brief 初始化回调函数
 * 
 * 在插件加载时被调用，负责：
 * 1. 从 ROS 参数服务器读取走廊配置
 * 2. 将航点数组转换为走廊段列表
 * 3. 计算走廊的外接矩形边界
 */
void CanyonLayer::onInitialize()
{
  // 获取节点指针（弱引用转强引用）
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("CanyonLayer: Failed to lock node");
  }
  
  // 声明并读取参数
  // 注意：必须先用 declare_parameter_if_not_declared 声明，
  //       my_nav_params.yaml 中的覆盖值才会生效（rclcpp 默认不自动声明参数覆盖值）
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".corridor_enabled", rclcpp::ParameterValue(corridor_enabled_));
  node->get_parameter(name_ + ".corridor_enabled", corridor_enabled_);
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".corridor_width", rclcpp::ParameterValue(corridor_width_));
  node->get_parameter(name_ + ".corridor_width", corridor_width_);
  corridor_half_width_ = corridor_width_ / 2.0;
  
  // 读取走廊航点（一维数组，格式 [x1, y1, x2, y2, ...]）
  // 例如：[-2.0, 0.0, 8.0, 0.0] 表示从 (-2, 0) 到 (8, 0) 的直线走廊
  std::vector<double> waypoints_raw;
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".corridor_waypoints", rclcpp::ParameterValue(std::vector<double>{}));
  node->get_parameter(name_ + ".corridor_waypoints", waypoints_raw);
  
  // 将一维数组转换为航点列表
  waypoints_.clear();
  for (size_t i = 0; i + 1 < waypoints_raw.size(); i += 2) {
    CorridorWaypoint wp;
    wp.x = waypoints_raw[i];
    wp.y = waypoints_raw[i + 1];
    waypoints_.push_back(wp);
  }
  
  // 初始化走廊段
  initializeCorridorSegments();
  
  // 输出初始化信息
  RCLCPP_INFO(node->get_logger(), 
    "CanyonLayer initialized: enabled=%s, width=%.2f, waypoints=%zu",
    corridor_enabled_ ? "true" : "false",
    corridor_width_,
    waypoints_.size());
  
  if (!waypoints_.empty()) {
    RCLCPP_INFO(node->get_logger(), 
      "Corridor bounds: [%.2f, %.2f] to [%.2f, %.2f]",
      corridor_min_x_, corridor_min_y_,
      corridor_max_x_, corridor_max_y_);
  }
}

/**
 * @brief 初始化走廊段
 * 
 * 根据航点列表创建走廊段，每个段包含：
 * - 起始和结束航点
 * - 方向向量（单位向量）
 * - 段长度
 */
void CanyonLayer::initializeCorridorSegments()
{
  segments_.clear();
  
  // 至少需要 2 个航点才能创建走廊段
  if (waypoints_.size() < 2) {
    RCLCPP_WARN(node_.lock()->get_logger(), 
      "CanyonLayer: Need at least 2 waypoints to create corridor segments");
    return;
  }
  
  // 遍历相邻航点对，创建走廊段
  for (size_t i = 0; i + 1 < waypoints_.size(); ++i) {
    CorridorSegment seg;
    seg.start = waypoints_[i];
    seg.end = waypoints_[i + 1];
    
    // 计算方向向量
    seg.dx = seg.end.x - seg.start.x;
    seg.dy = seg.end.y - seg.start.y;
    
    // 计算段长度
    seg.length = std::sqrt(seg.dx * seg.dx + seg.dy * seg.dy);
    
    // 跳过零长度的段（两个航点重合）
    if (seg.length > 1e-6) {
      // 归一化方向向量（变成单位向量）
      seg.dx /= seg.length;
      seg.dy /= seg.length;
      segments_.push_back(seg);
    } else {
      RCLCPP_WARN(node_.lock()->get_logger(), 
        "CanyonLayer: Skipping zero-length segment at index %zu", i);
    }
  }
  
  // 计算走廊的外接矩形边界（用于优化 updateBounds）
  getCorridorBounds(corridor_min_x_, corridor_min_y_, 
                    corridor_max_x_, corridor_max_y_);
}

/**
 * @brief 计算点到线段的最短距离
 * 
 * 算法说明：
 * 1. 将点投影到线段上，得到投影参数 t
 * 2. 如果 t <= 0，投影在起点之前，返回到起点的距离
 * 3. 如果 t >= length，投影在终点之后，返回到终点的距离
 * 4. 否则，投影在线段上，返回垂直距离
 * 
 * @param px 点的 X 坐标
 * @param py 点的 Y 坐标
 * @param segment 走廊段
 * @return 点到线段的最短距离（米）
 */
double CanyonLayer::pointToSegmentDistance(double px, double py, 
                                            const CorridorSegment & segment)
{
  // 计算从线段起点到目标点的向量
  double vx = px - segment.start.x;
  double vy = py - segment.start.y;
  
  // 计算投影长度（点积）
  // t 表示投影点在线段上的位置（从起点开始的距离）
  double t = vx * segment.dx + vy * segment.dy;
  
  // 限制投影在线段范围内
  if (t <= 0.0) {
    // 投影在起点之前，返回到起点的距离
    double dx = px - segment.start.x;
    double dy = py - segment.start.y;
    return std::sqrt(dx * dx + dy * dy);
  } else if (t >= segment.length) {
    // 投影在终点之后，返回到终点的距离
    double dx = px - segment.end.x;
    double dy = py - segment.end.y;
    return std::sqrt(dx * dx + dy * dy);
  } else {
    // 投影在线段上，计算垂直距离
    // 先计算投影点的坐标
    double proj_x = segment.start.x + t * segment.dx;
    double proj_y = segment.start.y + t * segment.dy;
    
    // 再计算目标点到投影点的距离
    double dx = px - proj_x;
    double dy = py - proj_y;
    return std::sqrt(dx * dx + dy * dy);
  }
}

/**
 * @brief 计算点到走廊中心线的最短距离
 * 
 * 遍历所有走廊段，找到最近的距离
 * 
 * @param px 点的 X 坐标
 * @param py 点的 Y 坐标
 * @return 到走廊中心线的最短距离（米）
 */
double CanyonLayer::distanceToCorridorCenter(double px, double py)
{
  // 如果没有走廊段，返回最大值
  if (segments_.empty()) {
    return std::numeric_limits<double>::max();
  }
  
  double min_dist = std::numeric_limits<double>::max();
  
  // 遍历所有走廊段，找到最近的距离
  for (const auto & segment : segments_) {
    double dist = pointToSegmentDistance(px, py, segment);
    min_dist = std::min(min_dist, dist);
  }
  
  return min_dist;
}

/**
 * @brief 计算走廊段的外接矩形边界
 * 
 * 用于优化 updateBounds，避免每次都遍历所有航点
 * 
 * @param min_x 输出最小 X 坐标
 * @param min_y 输出最小 Y 坐标
 * @param max_x 输出最大 X 坐标
 * @param max_y 输出最大 Y 坐标
 */
void CanyonLayer::getCorridorBounds(double & min_x, double & min_y, 
                                     double & max_x, double & max_y)
{
  // 如果没有航点，返回零边界
  if (waypoints_.empty()) {
    min_x = min_y = max_x = max_y = 0.0;
    return;
  }
  
  // 初始化边界为第一个航点
  min_x = max_x = waypoints_[0].x;
  min_y = max_y = waypoints_[0].y;
  
  // 扩展边界以包含所有航点
  for (const auto & wp : waypoints_) {
    min_x = std::min(min_x, wp.x);
    min_y = std::min(min_y, wp.y);
    max_x = std::max(max_x, wp.x);
    max_y = std::max(max_y, wp.y);
  }
  
  // 扩展边界以包含走廊宽度（两侧各扩展半宽）
  min_x -= corridor_half_width_;
  min_y -= corridor_half_width_;
  max_x += corridor_half_width_;
  max_y += corridor_half_width_;
}

/**
 * @brief 更新边界
 * 
 * 告诉 costmap 哪些区域需要更新。
 * 我们只需要更新走廊所在的区域。
 */
void CanyonLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                double * min_x, double * min_y,
                                double * max_x, double * max_y)
{
  // 如果走廊未启用或没有走廊段，不做任何操作
  if (!corridor_enabled_ || segments_.empty()) {
    return;
  }

  // 扩展边界以包含走廊区域
  // 使用 std::min/max 确保不会缩小已有的边界
  *min_x = std::min(*min_x, corridor_min_x_);
  *min_y = std::min(*min_y, corridor_min_y_);
  *max_x = std::max(*max_x, corridor_max_x_);
  *max_y = std::max(*max_y, corridor_max_y_);
}

/**
 * @brief 更新 costmap 代价
 * 
 * 核心逻辑：
 * 1. 遍历指定范围内的所有栅格
 * 2. 将栅格坐标转换为世界坐标
 * 3. 计算该点到走廊中心线的距离
 * 4. 如果距离大于走廊半宽，标记为致命障碍
 */
void CanyonLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                               int min_x, int min_y, int max_x, int max_y)
{
  // 如果走廊未启用或没有走廊段，不做任何操作
  if (!corridor_enabled_ || segments_.empty()) {
    return;
  }
  
  // 获取 costmap 的数据指针和参数
  unsigned char * master_array = master_grid.getCharMap();
  double resolution = master_grid.getResolution();  // 每个栅格的边长（米）
  double origin_x = master_grid.getOriginX();       // 地图原点的 X 坐标
  double origin_y = master_grid.getOriginY();       // 地图原点的 Y 坐标
  
  // 遍历指定范围内的所有栅格
  for (int x = min_x; x < max_x; ++x) {
    for (int y = min_y; y < max_y; ++y) {
      // 将栅格坐标 (x, y) 转换为世界坐标 (wx, wy)
      // 栅格中心的世界坐标 = 原点 + (栅格索引 + 0.5) * 分辨率
      double wx = origin_x + (x + 0.5) * resolution;
      double wy = origin_y + (y + 0.5) * resolution;
      
      // 计算该点到走廊中心线的最短距离
      double dist = distanceToCorridorCenter(wx, wy);
      
      // 如果距离大于走廊半宽，说明该点在走廊外
      // 将其标记为致命障碍（LETHAL_OBSTACLE = 254）
      if (dist > corridor_half_width_) {
        unsigned int index = master_grid.getIndex(x, y);
        master_array[index] = LETHAL_OBSTACLE;
      }
    }
  }
}

/**
 * @brief 重置层状态
 * 
 * 走廊约束是静态的（基于固定航点），不需要动态重置
 */
void CanyonLayer::reset()
{
  // 无需重置
}

/**
 * @brief 是否可清除
 * 
 * @return true 走廊约束可以被清除
 */
bool CanyonLayer::isClearable()
{
  return true;
}

}  // namespace my_first_agv

// 注册插件（让 pluginlib 能够加载这个类）
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_first_agv::CanyonLayer, nav2_costmap_2d::Layer)
