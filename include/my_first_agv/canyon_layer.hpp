#ifndef MY_FIRST_AGV_CANYON_LAYER_HPP
#define MY_FIRST_AGV_CANYON_LAYER_HPP

#include "nav2_costmap_2d/layer.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <utility>

namespace my_first_agv
{
/**
 * @brief 走廊航点结构体
 * 用于定义走廊的中心线路径
 */
struct CorridorWaypoint
{
  double x;  // X坐标（米）
  double y;  // Y坐标（米）
};

/**
 * @brief 走廊段结构体
 * 表示两个相邻航点之间的走廊段
 */
struct CorridorSegment
{
  CorridorWaypoint start;  // 起始航点
  CorridorWaypoint end;    // 结束航点
  double dx;               // X方向分量（单位向量）
  double dy;               // Y方向分量（单位向量）
  double length;           // 段长度（米）
};

/**
 * @class CanyonLayer
 * @brief 走廊约束层插件
 * 
 * 功能说明：
 * - 根据配置的走廊航点，在costmap中生成虚拟墙
 * - 走廊宽度可配置（默认1.5米）
 * - 走廊外的区域被标记为致命障碍（LETHAL_OBSTACLE）
 * - 确保机器人严格限制在走廊内移动
 * 
 * 参数配置：
 * - corridor_enabled: 是否启用走廊约束（bool，默认true）
 * - corridor_width: 走廊宽度（double，默认1.5米）
 * - corridor_waypoints: 走廊航点数组（double[]，格式[x1,y1, x2,y2, ...]）
 */
class CanyonLayer : public nav2_costmap_2d::Layer
{
public:
  CanyonLayer();
  virtual ~CanyonLayer();
  
  /**
   * @brief 初始化回调函数
   * 读取ROS参数并初始化走廊段
   */
  void onInitialize() override;
  
  /**
   * @brief 更新costmap代价
   * 将走廊外区域标记为致命障碍
   */
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_x, int min_y, int max_x, int max_y) override;
  
  /**
   * @brief 更新边界
   * 标记走廊区域为需要更新的"脏区域"
   */
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double * min_x, double * min_y, double * max_x, double * max_y) override;
  
  /**
   * @brief 重置层状态
   */
  void reset() override;
  
  /**
   * @brief 是否可清除
   * @return true 走廊约束是静态的，可以被清除
   */
  bool isClearable() override;

private:
  /**
   * @brief 初始化走廊段
   * 根据航点参数创建走廊段列表
   */
  void initializeCorridorSegments();
  
  /**
   * @brief 计算点到走廊中心线的最短距离
   * @param px 点的X坐标
   * @param py 点的Y坐标
   * @return 到走廊中心线的最短距离（米）
   */
  double distanceToCorridorCenter(double px, double py);
  
  /**
   * @brief 计算点到线段的最短距离
   * @param px 点的X坐标
   * @param py 点的Y坐标
   * @param segment 走廊段
   * @return 点到线段的最短距离（米）
   */
  double pointToSegmentDistance(double px, double py, const CorridorSegment & segment);
  
  /**
   * @brief 计算走廊段的外接矩形边界
   * @param min_x 输出最小X坐标
   * @param min_y 输出最小Y坐标
   * @param max_x 输出最大X坐标
   * @param max_y 输出最大Y坐标
   */
  void getCorridorBounds(double & min_x, double & min_y, double & max_x, double & max_y);

  // 走廊配置参数
  bool corridor_enabled_;              // 是否启用走廊约束
  double corridor_width_;              // 走廊宽度（米）
  double corridor_half_width_;         // 走廊半宽（米）
  std::vector<CorridorWaypoint> waypoints_;  // 走廊航点列表
  std::vector<CorridorSegment> segments_;    // 走廊段列表
  
  // 走廊边界缓存（用于updateBounds优化）
  double corridor_min_x_;
  double corridor_min_y_;
  double corridor_max_x_;
  double corridor_max_y_;
};

}  // namespace my_first_agv

#endif  // MY_FIRST_AGV_CANYON_LAYER_HPP