cmake_minimum_required(VERSION 3.8)
project(laser_avoid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 添加可执行文件
add_executable(laser_obstacle_detector src/laser_avoid.cpp)

# 设置包含目录
target_include_directories(laser_obstacle_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# 链接库
ament_target_dependencies(laser_obstacle_detector
  rclcpp
  sensor_msgs
  std_msgs
)

# 添加可执行文件
add_executable(laser_obstacle_detector_2 src/laser_avoid_2.cpp)

# 设置包含目录
target_include_directories(laser_obstacle_detector_2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# 链接库
ament_target_dependencies(laser_obstacle_detector_2
  rclcpp
  sensor_msgs
  std_msgs
)

# 安装目标
install(TARGETS
  laser_obstacle_detector_2
  laser_obstacle_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装启动文件

install(DIRECTORY
  include/
  DESTINATION include
)
# 导出包
ament_package()
