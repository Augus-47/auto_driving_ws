cmake_minimum_required(VERSION 3.8)
project(costmap_generator)

# 查找依赖包
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(FLANN REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Qhull REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
ament_auto_find_build_dependencies()

# 查找GridMap包
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
set(GRID_MAP_INCLUDE_DIR ${grid_map_core_INCLUDE_DIRS} ${grid_map_ros_INCLUDE_DIRS})

# 包含头文件目录
include_directories(
  include
  SYSTEM
  ${GRID_MAP_INCLUDE_DIR}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(costmap_generator_lib SHARED
  src/utils/points_to_costmap.cpp
)
target_link_libraries(costmap_generator_lib
  ${PCL_LIBRARIES}
  FLANN::FLANN
)

if(${PCL_VERSION} GREATER_EQUAL 1.12.1)
  find_package(Qhull REQUIRED)
  target_link_libraries(costmap_generator_lib
    QHULL::QHULL
  )
endif()

generate_parameter_library(costmap_generator_node_parameters
  param/costmap_generator_node_parameters.yaml
)

ament_auto_add_library(costmap_generator_node SHARED
  src/costmap_generator.cpp
)
target_link_libraries(costmap_generator_node
  ${PCL_LIBRARIES}
  costmap_generator_lib
  costmap_generator_node_parameters
)
target_compile_options(costmap_generator_node PUBLIC
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(costmap_generator_node
  PLUGIN "CostmapGenerator"
  EXECUTABLE costmap_generator
)

# 安装目标和共享文件
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)    