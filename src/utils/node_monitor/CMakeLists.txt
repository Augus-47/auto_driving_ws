cmake_minimum_required(VERSION 3.8)
project(node_monitor)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

# 启用Qt自动代码生成 (MOC, UIC, RCC)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 包含目录
include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
)

# 将所有源文件添加到可执行文件
add_executable(monitor
  src/main.cpp
  src/node_monitor.cpp
  src/node_monitor_ui.cpp
  include/node_monitor/node_monitor.hpp
  include/node_monitor/node_monitor_ui.hpp
)

# 链接依赖库
ament_target_dependencies(monitor
  rclcpp
  std_msgs
)

target_link_libraries(monitor
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

# 安装可执行文件
install(TARGETS
  monitor
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
  DESTINATION share/${PROJECT_NAME}
)
ament_package()