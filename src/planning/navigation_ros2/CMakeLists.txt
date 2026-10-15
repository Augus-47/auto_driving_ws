cmake_minimum_required(VERSION 3.8)
project(navigation_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(odom_to_path src/odom_to_path.cpp)
ament_target_dependencies(odom_to_path
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  odom_to_path
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  param
  map
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 