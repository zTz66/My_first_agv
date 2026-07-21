# 障碍物警告消息
# 用于在检测到长时间障碍物阻塞时发布警告信息

# 标准消息头，包含时间戳和坐标系信息
std_msgs/Header header

# 障碍物在地图坐标系中的位置
geometry_msgs/Point obstacle_position

# 障碍物持续阻塞的时间（单位：秒）
float64 duration

# 建议采取的操作（例如："等待障碍物移开"、"重新规划路径"等）
string suggested_action

# 标识机器人是否仍被障碍物阻塞
# true: 仍在阻塞中
# false: 障碍物已消失
bool is_blocking
