# 受限走廊导航系统 - 测试指南

## 📋 目录
1. [系统概述](#系统概述)
2. [启动系统](#启动系统)
3. [功能测试](#功能测试)
4. [常见问题排查](#常见问题排查)

---

## 系统概述

本系统实现了以下核心功能：

### 1. 走廊约束（CanyonLayer）
- 在 costmap 中创建 1.5 米宽的虚拟走廊
- 走廊外区域被标记为致命障碍（LETHAL_OBSTACLE）
- 确保机器人严格限制在走廊内移动
- 走廊边界检测精度 ±5cm

### 2. 遇障停车（CorridorController）
- 检测前方 0.5 米内的障碍物（180° 扇形区域）
- 立即停止所有运动
- 严格禁止重规划和绕行行为

### 3. 超时警告系统
- 监测障碍物持续时间
- 超过 60 秒后发布警告消息到 `/obstacle_warning` 话题
- 警告消息包含：障碍物位置、持续时间、建议操作

---

## 启动系统

### 步骤 1：设置环境变量
```bash
export TURTLEBOT3_MODEL=burger
```

### 步骤 2：启动完整系统
```bash
cd /home/ztz/demo01_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_first_agv corridor_nav_launch.py
```

### 步骤 3：等待系统初始化
- Gazebo 启动（约 5 秒）
- RViz2 启动（约 8 秒）
- 所有导航节点激活（约 10 秒）

你应该能看到：
- Gazebo 窗口：显示走廊环境和 TurtleBot3
- RViz2 窗口：显示地图、costmap、机器人位置

---

## 功能测试

### 测试 1：走廊约束验证

**目标**：验证机器人被限制在走廊内

**步骤**：
1. 在 RViz2 中，点击 "2D Nav Goal" 按钮
2. 在走廊**外**（例如 y=2.0）设置一个目标点
3. 观察全局路径

**预期结果**：
- ✅ 全局路径应该沿着走廊内部生成
- ✅ 在 RViz2 的 costmap 显示中，走廊外区域应该是深色（致命障碍）
- ✅ 机器人不会尝试穿越走廊边界

**验证方法**：
```bash
# 查看全局 costmap
ros2 topic echo /global_costmap/costmap
```

---

### 测试 2：遇障停车验证

**目标**：验证机器人在检测到障碍物时立即停车

**步骤**：
1. 在 RViz2 中设置一个走廊内的目标点（例如 x=6.0, y=0.0）
2. 机器人开始移动
3. 在 Gazebo 中，使用鼠标拖动红色盒子到机器人前方（距离 < 0.5 米）
4. 观察机器人行为

**预期结果**：
- ✅ 机器人在检测到障碍物后立即停止
- ✅ 终端输出：`🛑 检测到前方障碍物，停车等待`
- ✅ 机器人不会尝试绕行

**验证方法**：
```bash
# 查看机器人速度
ros2 topic echo /cmd_vel
# 应该看到 linear.x = 0.0, angular.z = 0.0
```

---

### 测试 3：障碍物消失后恢复

**目标**：验证障碍物消失后机器人恢复正常行驶

**步骤**：
1. 完成测试 2，机器人处于停车状态
2. 在 Gazebo 中，将红色盒子移开（距离 > 0.5 米）
3. 观察机器人行为

**预期结果**：
- ✅ 终端输出：`✅ 障碍物已消失，恢复正常行驶`
- ✅ 机器人继续向目标点移动
- ✅ 速度恢复正常

---

### 测试 4：60 秒超时警告

**目标**：验证障碍物持续 60 秒后发布警告

**步骤**：
1. 设置一个目标点，让机器人开始移动
2. 在机器人前方放置障碍物
3. 保持障碍物 60 秒以上
4. 监听警告话题

**预期结果**：
- ✅ 60 秒后，终端输出：`⚠️ 障碍物持续时间超过 60.0 秒，进入警告状态`
- ✅ `/obstacle_warning` 话题收到警告消息

**验证方法**：
```bash
# 监听警告话题
ros2 topic echo /obstacle_warning
```

**预期消息格式**：
```yaml
header:
  stamp:
    sec: 123
    nanosec: 456
  frame_id: map
obstacle_position:
  x: 5.0
  y: 0.0
  z: 0.0
duration: 60.5
suggested_action: "等待障碍物移开，禁止重规划或绕行"
is_blocking: true
```

---

### 测试 5：参数调整验证

**目标**：验证可配置参数的有效性

**步骤**：
1. 编辑配置文件：
```bash
nano /home/ztz/demo01_ws/src/my_first_agv/config/my_nav_params.yaml
```

2. 修改以下参数：
```yaml
CorridorFollow:
  obstacle_detect_range_max: 1.0  # 从 0.5 改为 1.0
  obstacle_warning_duration: 30.0  # 从 60.0 改为 30.0
```

3. 重新编译：
```bash
cd /home/ztz/demo01_ws
colcon build --packages-select my_first_agv
source install/setup.bash
```

4. 重新启动系统并测试

**预期结果**：
- ✅ 障碍物检测距离变为 1.0 米
- ✅ 警告超时时间变为 30 秒

---

## 常见问题排查

### 问题 1：插件未加载

**症状**：
```
Failed to create controller. Is the plugin registered?
```

**解决方案**：
```bash
# 检查插件是否注册
ros2 plugin list | grep my_first_agv

# 如果没有输出，重新编译
cd /home/ztz/demo01_ws
colcon build --packages-select my_first_agv --force-cmake-configure
source install/setup.bash
```

---

### 问题 2：走廊约束不生效

**症状**：机器人可以移动到走廊外

**可能原因**：
1. `corridor_enabled` 参数未设置为 `true`
2. `corridor_waypoints` 参数格式错误

**解决方案**：
```bash
# 检查参数
ros2 param get /global_costmap/global_costmap canyon_layer.corridor_enabled
ros2 param get /global_costmap/global_costmap canyon_layer.corridor_waypoints

# 预期输出：
# Boolean value is: true
# Double array is: [-2.0, 0.0, 8.0, 0.0]
```

---

### 问题 3：障碍物检测不灵敏

**症状**：机器人在障碍物很近时才停车

**可能原因**：
1. `obstacle_detect_range_max` 设置过小
2. costmap 分辨率不够

**解决方案**：
```bash
# 调整检测距离
# 编辑 my_nav_params.yaml，增加 obstacle_detect_range_max
obstacle_detect_range_max: 0.8  # 从 0.5 改为 0.8
```

---

### 问题 4：警告消息未发布

**症状**：障碍物持续 60 秒后没有收到警告

**可能原因**：
1. 状态机未进入 WARNING 状态
2. 发布者未正确初始化

**解决方案**：
```bash
# 检查终端日志，看是否有 "进入警告状态" 的输出
# 检查话题是否存在
ros2 topic list | grep obstacle_warning

# 检查话题类型
ros2 topic info /obstacle_warning
# 预期输出：Type: my_first_agv/msg/ObstacleWarning
```

---

### 问题 5：Gazebo 模型位置不对

**症状**：走廊墙壁与 costmap 中的虚拟墙不对齐

**可能原因**：
1. Gazebo 世界文件中的墙壁位置与 `corridor_waypoints` 不匹配
2. 机器人初始位置不在走廊内

**解决方案**：
```bash
# 检查 corridor.world 中的墙壁位置
# 左墙：y = -0.825
# 右墙：y = 0.825
# 走廊中心：y = 0.0

# 检查 my_nav_params.yaml 中的走廊航点
corridor_waypoints: [-2.0, 0.0, 8.0, 0.0]
# 走廊中心线：y = 0.0，宽度 1.5m
```

---

## 调试技巧

### 1. 查看 costmap 数据
```bash
# 查看全局 costmap
ros2 topic echo /global_costmap/costmap

# 在 RViz2 中添加 Costmap 显示
# Display -> Add -> By topic -> /global_costmap/costmap -> Map
```

### 2. 查看机器人速度
```bash
ros2 topic echo /cmd_vel
```

### 3. 查看控制器状态
```bash
# 查看终端日志，观察状态转换
# NORMAL -> STOPPED -> WARNING
```

### 4. 手动发布速度指令
```bash
# 测试机器人运动
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

### 5. 查看 TF 变换
```bash
ros2 run tf2_ros tf2_echo map base_link
```

---

## 性能优化建议

### 1. 调整控制器频率
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # 可以调整为 10.0-30.0
```

### 2. 调整 costmap 更新频率
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0  # 可以调整为 2.0-5.0
```

### 3. 调整障碍物检测步长
在 `corridor_controller.cpp` 中：
```cpp
double angle_step = 0.1;  // 角度步长（弧度），可以调整为 0.05-0.2
double dist_step = resolution;  // 距离步长，与 costmap 分辨率相同
```

---

## 总结

恭喜你完成了受限走廊导航系统的开发和测试！

### 你学到的关键概念：
1. **Costmap 插件开发**：如何创建自定义 costmap 层
2. **Controller 插件开发**：如何实现自定义控制器
3. **状态机设计**：如何实现 NORMAL -> STOPPED -> WARNING 状态转换
4. **Pure Pursuit 算法**：路径跟踪的经典算法
5. **Nav2 架构**：理解 Nav2 各组件的交互流程

### 下一步可以尝试：
1. 创建弯曲走廊（修改 `corridor_waypoints`）
2. 添加多个障碍物
3. 实现更复杂的警告逻辑（例如分级警告）
4. 集成其他传感器（例如深度相机）

祝你学习愉快！🚀
