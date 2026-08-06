#!/usr/bin/env python3
"""
走廊地图生成脚本

功能：根据 corridor.world 中的走廊结构，生成对应的 OccupancyGrid 地图文件
输出：corridor_map.pgm（地图图像）和 corridor_map.yaml（地图配置）

地图坐标系说明：
- 原点 origin = [-9.5, -7.5, 0.0]  表示地图左下角在世界坐标 (x=-9.5, y=-7.5) 处
- 分辨率 resolution = 0.05          每个像素代表 0.05 米
- 地图尺寸：15m x 15m               即 300 x 300 像素

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
MAP_SIZE_M = 15.0          # 地图边长（米）
RESOLUTION = 0.05          # 分辨率（米/像素）
ORIGIN_X = -9.5            # 地图原点 X 坐标（世界坐标系），走廊居中
ORIGIN_Y = -7.5            # 地图原点 Y 坐标（世界坐标系）

# 走廊参数（与 corridor.world 完全一致）
CORRIDOR_WIDTH = 1.5       # 走廊宽度（米）
WALL_THICKNESS = 0.15      # 墙壁厚度（米）

# Z 形走廊三段中心线端点（与 corridor.world 完全一致）
# 段1：(-5,0) → (-2,0)   段2：(-2,0) → (-2,-3)   段3：(-2,-3) → (1,-3)
SEGMENTS = [
    [(-5.0, 0.0), (-2.0, 0.0)],   # 段1 水平
    [(-2.0, 0.0), (-2.0, -3.0)],  # 段2 垂直
    [(-2.0, -3.0), (1.0, -3.0)],  # 段3 水平
]


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
# 计算 Z 形走廊各段侧墙的世界坐标范围
# 半宽 0.75，半墙厚 0.075（与 corridor.world 完全一致）
# ==========================================
HALF = CORRIDOR_WIDTH / 2.0    # 0.75
TH = WALL_THICKNESS / 2.0      # 0.075

# 段1（水平，y=0，x∈[-5,-2.75]）：上下墙（南墙东端缩到 x=-2.75 与段2西墙齐平；北墙东延到 x=-1.175 与段2东墙连接）
seg1_bottom = (-5.0, -2.0 - HALF, -HALF - TH, -HALF)      # x∈[-5,-2.75], y∈[-0.825,-0.75]
seg1_top    = (-5.0, -2.0 + HALF + TH, HALF, HALF + TH)   # x∈[-5,-1.175], y∈[0.75,0.825]
# 段2（垂直，x=-2，y∈[-3,0]）：左右墙（西墙顶部缩到 y=-0.75 与段1南墙齐平、底部延到 y=-3.825 与段3南墙连接；东墙顶部延到 y=0.825 与段1北墙连接）
seg2_left   = (-2.0 - HALF - TH, -2.0 - HALF, -3.0 - HALF - TH, -HALF)  # x∈[-2.825,-2.75], y∈[-3.825,-0.75]
seg2_right  = (-2.0 + HALF, -2.0 + HALF + TH, -2.25, HALF + TH)  # x∈[-1.25,-1.175], y∈[-2.25,0.825]
# 段3（水平，y=-3，x∈[-2.825,1]）：上下墙（南墙西延到 x=-2.825 与段2西墙连接；顶墙西端缩到 x=-1 给走廊2出口留通道）
seg3_bottom = (-2.0 - HALF - TH, 1.0, -3.0 - HALF - TH, -3.0 - HALF)  # x∈[-2.825,1], y∈[-3.825,-3.75]
seg3_top    = (-1.0, 1.0, -3.0 + HALF, -3.0 + HALF + TH)  # y∈[-2.25,-2.175]

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
# 绘制 Z 形走廊的 6 面墙
# ==========================================
print("绘制 Z 形走廊墙体（6 面墙）...")
walls = [seg1_bottom, seg1_top, seg2_left, seg2_right,
         seg3_bottom, seg3_top]
for w in walls:
    draw_wall(map_data, *w)


# ==========================================
# 在地图边缘绘制边界墙（模拟房间外墙）
# ==========================================
print("绘制边界...")
draw_wall(map_data, -9.5, 5.5, 7.4, 7.5)        # 上边界
draw_wall(map_data, -9.5, 5.5, -7.5, -7.4)       # 下边界
draw_wall(map_data, -9.5, -9.4, -7.5, 7.5)       # 左边界
draw_wall(map_data, 5.4, 5.5, -7.5, 7.5)         # 右边界

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
print(f"   走廊形状: Z 形（三段折线）")