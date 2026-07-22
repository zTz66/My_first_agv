# 受限走廊导航系统 - 对话接力文档

> 生成时间：2026-07-22  
> 工作空间：`/home/ztz/demo01_ws`  
> 目标包：`my_first_agv`

---

## 1. 项目目标

开发一套完整的"受限走廊导航系统"，核心要求：

1. 机器人在 **1.5 米宽**的虚拟走廊内沿 **X 轴方向**来回运行
2. 使用**自定义 Nav2 Controller 插件**实现核心逻辑
3. **遇障即停，不绕行**：遇到走廊中间的移动障碍物必须立即停车
4. 障碍物持续超过 **60 秒**后，通过 `/obstacle_warning` 发布警告
5. 障碍物移走后，机器人自动继续前进
6. 当目标点在机器人身后时，必须能够**原地旋转 180°**后再直线前进，不能在狭窄走廊中绕弧线掉头撞墙
7. 由于电脑配置不足，**Gazebo 单独启动**，其他导航节点放在一起启动

---

## 2. 当前已实现内容

| 功能 | 文件 | 状态 |
|------|------|------|
| 走廊 Gazebo 世界 | `worlds/corridor.world` | ✅ 沿 X 轴，1.5m 宽，含可移动红色障碍物 |
| 走廊地图生成 | `maps/generate_corridor_map.py` | ✅ 生成 pgm + yaml |
| CanyonLayer 虚拟墙 | `src/canyon_layer.cpp` | ✅ 通过 costmap 限制机器人在走廊内 |
| 自定义 Controller | `src/corridor_controller.cpp` | ✅ 继承 `nav2_core::Controller` |
| 障碍物状态机 | `src/corridor_controller.cpp` | ✅ NORMAL / STOPPED / WARNING |
| 激光雷达障碍物检测 | `src/corridor_controller.cpp` | ✅ 直接订阅 `/scan`，不依赖 costmap 清除 |
| 警告消息发布 | `msg/ObstacleWarning.msg` | ✅ 自定义消息类型 |
| Gazebo 分离启动 | `launch/corridor_gazebo_launch.py` | ✅ 单独启动 Gazebo + 机器人 |
| 导航启动 | `launch/corridor_nav_launch.py` | ✅ 启动 map_server / AMCL / planner / controller / RViz |
| 行为树 | `config/corridor_bt.xml` | ✅ `PipelineSequence`，无 RecoveryNode |
| Rotation Shim | `src/corridor_controller.cpp` | 🔄 刚加入，参考 Nav2 RotationShimController，待验证 |

---

## 3. 关键文件清单

```
/home/ztz/demo01_ws/src/my_first_agv/
├── src/corridor_controller.cpp          # 核心控制器实现
├── include/my_first_agv/corridor_controller.hpp
├── config/my_nav_params.yaml            # 导航参数
├── config/corridor_bt.xml               # 行为树
├── worlds/corridor.world                # Gazebo 走廊世界
├── maps/generate_corridor_map.py        # 地图生成脚本
├── launch/corridor_gazebo_launch.py     # Gazebo 启动
├── launch/corridor_nav_launch.py        # 导航启动
├── msg/ObstacleWarning.msg              # 警告消息
└── CMakeLists.txt                       # 构建配置
```

---

## 4. 最近修改（需要重点关注的部分）

### 4.1 Rotation Shim 原地旋转

参考 Nav2 `RotationShimController` 实现：

- `getSampledPathPt(pose)`：找到路径上距离机器人当前位置最近的点，再向前 0.5m 取采样点
- `computeRotateToHeadingCommand()`：输出纯旋转速度，线速度为 0
- 收到新路径时（`setPlan`）设置 `path_updated_ = true`
- 每次 `computeVelocityCommands` 时，如果 `path_updated_ == true`：
  - 计算机器人朝向与路径采样点的角度差（机器人坐标系）
  - 使用滞环：
    - 进入旋转：偏差 > 45° (`angular_dist_threshold`)
    - 退出旋转：偏差 < 5° (`angular_disengage_threshold`)
  - 需要旋转时输出纯角速度；旋转完成后才进入 Pure Pursuit

### 4.2 障碍物检测

- 订阅 `/scan`
- 检测路径前方扇形区域：
  - 角度范围：`obstacle_detect_angle = 0.5 rad`（±0.25 rad ≈ ±14°）
  - 距离范围：`[0.20, 0.30] m`
- 检测方向跟随 `lookahead_point`，不是固定机器人正前方
- 大角度转身时暂停检测，避免误检侧面墙壁

### 4.3 关键参数（my_nav_params.yaml 中 CorridorFollow 部分）

```yaml
CorridorFollow:
  plugin: "my_first_agv::CorridorController"
  lookahead_dist: 0.5
  max_linear_speed: 0.22
  min_linear_speed: 0.05
  max_angular_speed: 1.0

  # 旋转 shim 参数
  angular_dist_threshold: 0.785      # 45°，进入旋转
  angular_disengage_threshold: 0.087 # 5°，退出旋转
  forward_sampling_distance: 0.5
  rotate_to_heading_angular_vel: 0.8

  # 障碍物检测参数
  obstacle_detect_range_min: 0.20
  obstacle_detect_range_max: 0.30
  obstacle_detect_angle: 0.5
  obstacle_warning_duration: 30.0
```

---

## 5. 当前待验证 / 待解决问题

### 5.1 主要问题（本轮最后讨论的问题）

向身后目标点发送导航目标时：
- 期望：机器人原地旋转 180°，然后直线前进到达目标点
- 实际：可以原地旋转掉头，但是不能直线前进，最终还是撞墙

### 5.2 可能原因

1. Rotation Shim 退出阈值之前是 30°，刚改为 5°，已验证
2. Pure Pursuit 在走廊内可能产生过大横向偏移
3. 原地旋转时机器人 footprint 是否会与墙碰撞（理论上 1.5m 宽够转）
4. 激光检测扇形角度或距离设置可能需要调整

### 5.3 需要观察的日志

```bash
# 正常应该看到
原地旋转中：路径方向与当前朝向偏差 175.X°
原地旋转完成：偏差 3.X°，开始路径跟踪
激光路径前方无障碍：机器人 (x.x, y.y)，路径方向 x.x°，扇形内最小距离 x.xx m
```

---

## 6. 标准测试流程

### 6.1 启动系统

```bash
# 终端 1：清理并启动 Gazebo
pkill -9 gazebo
pkill -9 gzserver
pkill -9 gzclient
source /home/ztz/demo01_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch my_first_agv corridor_gazebo_launch.py

# 终端 2：等待 Gazebo 完全加载后，启动导航
source /home/ztz/demo01_ws/install/setup.bash
ros2 launch my_first_agv corridor_nav_launch.py
```

### 6.2 RViz 操作

1. 设置 **2D Pose Estimate**：在机器人当前位置点击并向上拖动（+X 方向）
2. 设置第一个 **Nav2 Goal**：走廊前方，例如 x=6, y=0
3. 机器人到达后，设置第二个 **Nav2 Goal**：机器人身后，例如 x=1, y=0

### 6.3 障碍物测试

1. 在 Gazebo 中拖动红色障碍物到机器人前方
2. 机器人应停车，终端显示：`检测到前方障碍物，停车等待`
3. 拖动障碍物离开
4. 机器人应继续前进，终端显示：`障碍物已消失，从 STOPPED 恢复正常行驶`

---

## 7. 重新编译命令

修改源码后必须重新编译：

```bash
cd /home/ztz/demo01_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_first_agv
source install/setup.bash
```

---

## 8. 给新模型的初始提示词

```markdown
我正在开发 ROS2 Humble 的受限走廊导航系统，工作空间在 `/home/ztz/demo01_ws`。

请先阅读 `/home/ztz/demo01_ws/src/my_first_agv/HANDOFF.md` 了解项目上下文。

当前核心问题是：向身后目标点发送导航目标时，机器人旋转 180° 后没有直线朝着目标点前进，而是撞墙。

关键文件：
- 控制器实现：src/my_first_agv/src/corridor_controller.cpp
- 控制器头文件：src/my_first_agv/include/my_first_agv/corridor_controller.hpp
- 参数：src/my_first_agv/config/my_nav_params.yaml
- 行为树：src/my_first_agv/config/corridor_bt.xml

请帮我分析原因并修复。修改前先问我任何不确定的地方。用中文回复。
```

---

## 9. 备注

- 用户是 ROS2 初学者，希望逐步教学
- 用户没有实体机器人，所有验证通过 Gazebo 仿真完成
- 当前 TurtleBot3 模型为 `burger`
- 走廊沿 X 轴，y 范围约为 [-0.75, 0.75]
- 地图原点 origin = [-5.0, -5.0, 0.0]，分辨率 0.05
- 我用的是ubuntu 22.04 +  ros2 humble + turtlebot3=waffle仿真
- 先阅读相关源码文件，理解当前实现
- 修改前如果信息不足，先向我提问确认
- 每次只修改必要文件，修改后需要 colcon build
- 用中文回复我
- gazebo单独启动，导航单独启动
- 我会给你一些必要的代码文件，你先完整读一遍
