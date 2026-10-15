cmake_minimum_required(VERSION 3.8)
project(rs_to_velodyne)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
add_executable(rs_to_velodyne src/rs_to_velodyne.cpp)
ament_target_dependencies(
  rs_to_velodyne
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "message_filters"
  "pcl_conversions"
  "pcl_ros"
  "geometry_msgs"
  "tf2_geometry_msgs"
)

install(TARGETS 
  rs_to_velodyne
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件夹
install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

