cmake_minimum_required(VERSION 3.8)
project(my_first_agv)

# 默认 C++ 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 包含目录
include_directories(include)

# ==========================================
# 0. 生成自定义消息类型
# ==========================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleWarning.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ==========================================
# 1. 编译 canyon_layer 插件库
# ==========================================
add_library( canyon_layer SHARED src/canyon_layer.cpp)


target_include_directories(canyon_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(canyon_layer
  rclcpp
  nav2_costmap_2d
  nav2_common
  nav2_util
)

# ==========================================
# 2. 安装 canyon_layer 插件库（关键！）
# ==========================================
install(TARGETS
  canyon_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ==========================================
# 3. 导出插件描述文件（关键！）
# ==========================================
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins/canyon_layer.xml)

# ==========================================
# 4. 编译 corridor_controller 插件库
# ==========================================
add_library(corridor_controller SHARED src/corridor_controller.cpp)

target_include_directories(corridor_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(corridor_controller
  rclcpp
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# 链接自定义消息类型
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(corridor_controller "${cpp_typesupport_target}")

# ==========================================
# 5. 安装 corridor_controller 插件库
# ==========================================
install(TARGETS
  corridor_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ==========================================
# 6. 导出 corridor_controller 插件描述文件
# ==========================================
pluginlib_export_plugin_description_file(nav2_core plugins/corridor_controller.xml)

# ==========================================
# 7. 安装 Launch、Config、Worlds、Maps 文件
# ==========================================
install(DIRECTORY
  launch
  config
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# ==========================================
# 6. 测试（如果有）
# ==========================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
