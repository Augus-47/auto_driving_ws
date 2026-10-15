cmake_minimum_required(VERSION 3.8)
project(camera_raw)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4.2 REQUIRED)

find_package(cv_bridge REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(camera_node src/main.cpp
  src/camera_node.cpp
)
ament_target_dependencies(camera_node 
    rclcpp 
    sensor_msgs
    cv_bridge
    OpenCV)

install(TARGETS 
  camera_node
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
