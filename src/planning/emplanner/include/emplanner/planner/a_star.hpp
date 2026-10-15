#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <vector>
#include <queue>
#include <set>
#include <memory>
#include <utility>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <functional>

// A*算法节点类
class AStarNode {
public:
    AStarNode(std::pair<int, int> position, std::shared_ptr<AStarNode> parent = nullptr)
        : position(position), parent(parent), g(0), h(0), f(0) {}

    std::pair<int, int> position;
    std::shared_ptr<AStarNode> parent;
    double g;
    double h;
    double f;

    bool operator<(const AStarNode& other) const {
        return f > other.f; // 优先队列默认是最大堆，所以反向比较
    }
};

// 地图坐标信息结构
struct MapInfo {
    int width;          // 地图宽度（像素）
    int height;         // 地图高度（像素）
    double resolution;  // 地图分辨率（米/像素）
    double origin_x;    // 地图原点X坐标（米）
    double origin_y;    // 地图原点Y坐标（米）
};

// A*路径规划器类
class AStarPlanner {
public:
    // 构造函数
    AStarPlanner();
    
    // 设置地图数据和地图信息
    void setMap(const std::vector<std::vector<int>>& map_data, const MapInfo& map_info);
    
    // 执行路径规划，返回路径点（栅格坐标）
    std::vector<std::pair<int, int>> planPath(
        const std::pair<int, int>& start, 
        const std::pair<int, int>& goal);
    
    // 执行路径规划，传入世界坐标（米），返回世界坐标路径
    std::vector<std::pair<double, double>> planPathWorld(
        double start_x, double start_y,
        double goal_x, double goal_y);
    
    // 世界坐标到栅格坐标的转换
    std::pair<int, int> worldToMap(double x, double y) const;
    
    // 栅格坐标到世界坐标的转换
    std::pair<double, double> mapToWorld(int map_x, int map_y) const;
    
    // 设置日志回调函数
    void setLogCallback(std::function<void(const std::string&)> callback);
    
private:
    // 地图数据
    std::vector<std::vector<int>> map_data_;
    
    // 地图信息
    MapInfo map_info_;
    
    // 日志回调函数
    std::function<void(const std::string&)> log_callback_;
    
    // 启发函数：曼哈顿距离
    double heuristic(const std::pair<int, int>& a, const std::pair<int, int>& b) const;
    
    // 日志输出
    void log(const std::string& message) const;
};

#endif // ASTAR_PLANNER_HPP