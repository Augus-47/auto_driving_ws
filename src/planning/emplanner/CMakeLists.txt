cmake_minimum_required(VERSION 3.8)
project(emplanner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(nav_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(car_msg REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OsqpEigen_INCLUDE_DIR}
  ${nav2_map_server_INCLUDE_DIR}
)

# 原始测试示例
add_executable(hya_star_node src/hya_star_node.cpp
    src/planner/a_star.cpp
    src/local_plan/abstract_algorithm.cpp
    src/local_plan/astar_search.cpp
    src/local_plan/reeds_shepp.cpp
    src/common/vehicle_info.cpp
)
target_link_libraries(hya_star_node
  ${EIGEN3_LIBRARIES}
)
ament_target_dependencies(hya_star_node
  rclcpp
  Eigen3
  nav_msgs
  nav2_map_server
  yaml_cpp_vendor
  car_msg
  tf2_geometry_msgs
  visualization_msgs
)

add_executable(dp_sl_test src/main.cpp
    src/emplanner_node.cpp
    src/reference_line/reference_line.cpp
    src/reference_line/reference_line_provider.cpp
    src/reference_line/reference_line_smoother.cpp
    src/common/plot.cpp
    src/planner/emplanner.cpp
    src/planner/trajectory.cpp
    src/planner/path_time_graph.cpp
    src/planner/speed_time_graph.cpp
    src/planner/a_star.cpp
    src/common/planner_visualizer.cpp
)
target_link_libraries(dp_sl_test OsqpEigen::OsqpEigen
  Python3::Python Python3::Module
  ${EIGEN3_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  ${Python3_LIBRARIES}
)
target_include_directories(dp_sl_test PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

ament_target_dependencies(dp_sl_test
  rclcpp
  Eigen3
  OsqpEigen
  nav_msgs
  nav2_map_server
  yaml_cpp_vendor
  car_msg
  tf2_geometry_msgs
  visualization_msgs
)


# 安装目标
install(TARGETS dp_sl_test hya_star_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)
# 安装头文件
install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
