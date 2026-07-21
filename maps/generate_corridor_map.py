#!/usr/bin/env python3
"""
走廊地图生成脚本

功能：根据 corridor.world 中的走廊结构，生成对应的 OccupancyGrid 地图文件
输出：corridor_map.pgm（地图图像）和 corridor_map.yaml（地图配置）

地图坐标系说明：
- 原点 origin = [-5.0, -5.0, 0.0]  表示地图左下角在世界坐标 (x=-5, y=-5) 处
- 分辨率 resolution = 0.05          每个像素代表 0.05 米
- 地图尺寸：10m x 10m               即 200 x 200 像素

颜色规则：
- 白色 (255) = 自由空间（机器人可通行）
- 黑色 (0)   = 障碍物（墙壁）
"""

import numpy as np
from PIL import Image
import os

# ==========================================
# 地图参数配置（与 corridor.world 对应）
# ==========================================
MAP_SIZE_M = 10.0          # 地图边长（米）
RESOLUTION = 0.05          # 分辨率（米/像素）
ORIGIN_X = -5.0            # 地图原点 X 坐标（世界坐标系）
ORIGIN_Y = -5.0            # 地图原点 Y 坐标（世界坐标系）

# 走廊参数（与 corridor.world 完全一致）
CORRIDOR_LENGTH = 10.0     # 走廊长度（米）
CORRIDOR_WIDTH = 1.5       # 走廊宽度（米）
WALL_THICKNESS = 0.15      # 墙壁厚度（米）
CORRIDOR_CENTER_X = 0.0    # 走廊中心 X 坐标
CORRIDOR_CENTER_Y = 3.0    # 走廊中心 Y 坐标

# 障碍物参数（初始位置）
OBSTACLE_X = 0.0
OBSTACLE_Y = 2.0
OBSTACLE_SIZE = 0.3 

# ==========================================
# 计算地图尺寸（像素）
# ==========================================
MAP_PIXELS = int(MAP_SIZE_M / RESOLUTION)  # 200 像素
print(f"地图尺寸: {MAP_PIXELS} x {MAP_PIXELS} 像素")
print(f"实际范围: {MAP_SIZE_M} x {MAP_SIZE_M} 米")

# ==========================================
# 创建空白地图（全白 = 自由空间）
# ==========================================
map_data = np.full((MAP_PIXELS, MAP_PIXELS), 255, dtype=np.uint8)

# ==========================================
# 坐标转换函数
# ==========================================
def world_to_pixel(wx, wy):
    """
    将世界坐标转换为地图像素坐标
    注意：地图图像的 Y 轴是向下增长的，而世界坐标的 Y 轴是向上增长的
    所以需要翻转 Y 轴
    """
    px = int((wx - ORIGIN_X) / RESOLUTION)
    py = MAP_PIXELS - int((wy - ORIGIN_Y) / RESOLUTION)  # 翻转 Y 轴
    return px, py

# ==========================================
# 计算墙壁的世界坐标范围
# ==========================================
# 左墙：y 范围 [-0.9, -0.75]
left_wall_y_min = -CORRIDOR_CENTER_Y - CORRIDOR_WIDTH/2 - WALL_THICKNESS
left_wall_y_max = -CORRIDOR_CENTER_Y - CORRIDOR_WIDTH/2
# 右墙：y 范围 [0.75, 0.9]
right_wall_y_min = CORRIDOR_CENTER_Y + CORRIDOR_WIDTH/2
right_wall_y_max = CORRIDOR_CENTER_Y + CORRIDOR_WIDTH/2 + WALL_THICKNESS

# 走廊 X 范围（墙的长度）
wall_x_min = CORRIDOR_CENTER_X - CORRIDOR_LENGTH/2  # -2.0
wall_x_max = CORRIDOR_CENTER_X + CORRIDOR_LENGTH/2  # 8.0

# ==========================================
# 绘制墙壁的函数
# ==========================================
def draw_wall(map_array, x_min, x_max, y_min, y_max):
    """在地图上绘制一个矩形墙壁"""
    px_min, py_min = world_to_pixel(x_min, y_min)
    px_max, py_max = world_to_pixel(x_max, y_max)
    
    # 确保坐标顺序正确（min < max）
    px_min, px_max = min(px_min, px_max), max(px_min, px_max)
    py_min, py_max = min(py_min, py_max), max(py_min, py_max)
    
    # 在地图上绘制黑色墙壁（值为 0）
    map_array[py_min:py_max, px_min:px_max] = 0

# ==========================================
# 绘制走廊墙壁
# ==========================================
print("绘制左墙...")
draw_wall(map_data, left_wall_x_min, left_wall_x_max, wall_y_min, wall_y_max)

print("绘制右墙...")
draw_wall(map_data, right_wall_x_min, right_wall_x_max, wall_y_min, wall_y_max)

# ==========================================
# 绘制障碍物（盒子）
# ==========================================
print("绘制障碍物...")
draw_wall(map_data, 
          OBSTACLE_X - OBSTACLE_SIZE/2, OBSTACLE_X + OBSTACLE_SIZE/2,
          OBSTACLE_Y - OBSTACLE_SIZE/2, OBSTACLE_Y + OBSTACLE_SIZE/2)

# ==========================================
# 在地图边缘绘制边界墙（模拟房间外墙）
# ==========================================
print("绘制边界...")
draw_wall(map_data, -5.0, 5.0, 4.9, 5.0)    # 上边界
draw_wall(map_data, -5.0, 5.0, -5.0, -4.9)  # 下边界
draw_wall(map_data, -5.0, -4.9, -5.0, 5.0)  # 左边界
draw_wall(map_data, 4.9, 5.0, -5.0, 5.0)    # 右边界

# ==========================================
# 保存地图图像（PGM 格式）
# ==========================================
output_dir = os.path.dirname(os.path.abspath(__file__))
pgm_path = os.path.join(output_dir, 'corridor_map.pgm')

img = Image.fromarray(map_data, mode='L')
img.save(pgm_path)
print(f"\n地图图像已保存: {pgm_path}")

# ==========================================
# 保存地图配置文件（YAML）
# ==========================================
yaml_content = f"""# 走廊导航地图配置文件
# 由 generate_corridor_map.py 自动生成
image: corridor_map.pgm
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""

yaml_path = os.path.join(output_dir, 'corridor_map.yaml')
with open(yaml_path, 'w') as f:
    f.write(yaml_content)
print(f"地图配置已保存: {yaml_path}")

print("\n✅ 地图生成完成！")
print(f"   地图尺寸: {MAP_SIZE_M}m x {MAP_SIZE_M}m")
print(f"   分辨率: {RESOLUTION}m/像素")
print(f"   走廊宽度: {CORRIDOR_WIDTH}m")
print(f"   走廊长度: {CORRIDOR_LENGTH}m")