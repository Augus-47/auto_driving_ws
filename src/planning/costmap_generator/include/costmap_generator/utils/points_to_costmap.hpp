#ifndef COSTMAP_GENERATOR__UTILS__POINTS_TO_COSTMAP_HPP_
#define COSTMAP_GENERATOR__UTILS__POINTS_TO_COSTMAP_HPP_

#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <vector>

// PointsToCostmap 类用于将点云数据转换为代价地图
class PointsToCostmap
{
public:
    // 从点云数据中计算代价地图
    // 输入参数：
    // - maximum_height_thres: 点云数据的最大高度阈值
    // - minimum_height_thres: 点云数据的最小高度阈值
    // - grid_min_value: 代价地图的最小代价值
    // - grid_max_value: 代价地图的最大代价值
    // - gridmap: 基于网格地图的代价地图
    // - gridmap_layer_name: 网格地图层名称
    // - in_sensor_points: 订阅到的点云数据
    // 输出：
    // - 计算得到的代价地图，以 grid_map::Matrix 格式返回
    grid_map::Matrix makeCostmapFromPoints(
        const double maximum_height_thres, const double minimum_height_thres,
        const double grid_min_value, const double grid_max_value, const grid_map::GridMap & gridmap,
        const std::string & gridmap_layer_name,
        const pcl::PointCloud<pcl::PointXYZ> & in_sensor_points);

private:
    double grid_length_x_;  // 网格地图的 x 轴长度
    double grid_length_y_;  // 网格地图的 y 轴长度
    double grid_resolution_;  // 网格地图的分辨率
    double grid_position_x_;  // 网格地图的 x 轴位置
    double grid_position_y_;  // 网格地图的 y 轴位置

    // 初始化网格地图参数
    // 输入参数：
    // - gridmap: 要初始化的网格地图对象
    void initGridmapParam(const grid_map::GridMap & gridmap);

    // 检查网格索引是否有效
    // 输入参数：
    // - grid_ind: 对应点云的网格索引
    // 输出：
    // - 如果索引有效，返回 true
    bool isValidInd(const grid_map::Index & grid_ind);

    // 从点云中获取网格索引
    // 输入参数：
    // - point: 订阅到的点云中的一个点
    // 输出：
    // - 点对应的网格索引
    grid_map::Index fetchGridIndexFromPoint(const pcl::PointXYZ & point);

    // 将点云分配到网格地图的相应单元格中
    // 输入参数：
    // - in_sensor_points: 订阅到的点云数据
    // 输出：
    // - 一个大小为网格 x 长度 × 网格 y 长度的网格，其中填充了对应网格单元格中的点的高度
    std::vector<std::vector<std::vector<double>>> assignPoints2GridCell(
        const pcl::PointCloud<pcl::PointXYZ> & in_sensor_points);

    // 从订阅的点云数据中计算代价地图
    // 输入参数：
    // - maximum_height_thres: 点云数据的最大高度阈值
    // - minimum_height_thres: 点云数据的最小高度阈值
    // - grid_min_value: 代价地图的最小代价值
    // - grid_max_value: 代价地图的最大代价值
    // - gridmap: 基于网格地图的代价地图
    // - gridmap_layer_name: 网格地图层名称
    // - grid_vec: 一个大小为网格 x 长度 × 网格 y 长度的网格，其中填充了对应网格单元格中的点的高度
    // 输出：
    // - 计算得到的代价地图，以 grid_map::Matrix 格式返回
    grid_map::Matrix calculateCostmap(
        const double maximum_height_thres, const double minimum_height_thres,
        const double grid_min_value, const double grid_max_value, const grid_map::GridMap & gridmap,
        const std::string & gridmap_layer_name,
        const std::vector<std::vector<std::vector<double>>> grid_vec);
};

#endif  // AUTOWARE__COSTMAP_GENERATOR__UTILS__POINTS_TO_COSTMAP_HPP_