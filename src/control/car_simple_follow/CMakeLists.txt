cmake_minimum_required(VERSION 3.8)
project(car_simple_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(car_msg REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 添加可执行文件
add_executable(simple_follow_node src/simple_follow.cpp)

# 设置包含目录
target_include_directories(simple_follow_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# 链接库
ament_target_dependencies(simple_follow_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  car_msg
)

# 安装目标
install(TARGETS
  simple_follow_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY
  include/
  DESTINATION include
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
)


ament_package()
