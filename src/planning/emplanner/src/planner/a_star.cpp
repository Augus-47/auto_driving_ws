#include "emplanner/planner/a_star.hpp"
#include <sstream>

AStarPlanner::AStarPlanner() {
    // 默认日志函数使用标准输出
    log_callback_ = [](const std::string& message) {
        std::cout << "[AStarPlanner] " << message << std::endl;
    };
}

// 设置地图数据和地图信息
void AStarPlanner::setMap(const std::vector<std::vector<int>>& map_data, const MapInfo& map_info) {
    map_data_ = map_data;
    map_info_ = map_info;
    
    std::stringstream ss;
    ss << "地图已设置: " << map_info_.width << "x" << map_info_.height 
       << ", 分辨率: " << map_info_.resolution 
       << ", 原点: (" << map_info_.origin_x << ", " << map_info_.origin_y << ")";
    log(ss.str());
}

void AStarPlanner::setLogCallback(std::function<void(const std::string&)> callback) {
    log_callback_ = callback;
}

void AStarPlanner::log(const std::string& message) const {
    if (log_callback_) {
        log_callback_(message);
    }
}

// 将世界坐标转换为栅格坐标
std::pair<int, int> AStarPlanner::worldToMap(double x, double y) const {
    int map_x = static_cast<int>((x - map_info_.origin_x) / map_info_.resolution);
    int map_y = static_cast<int>((y - map_info_.origin_y) / map_info_.resolution);

    // 限制索引不超出地图范围
    map_x = std::max(0, std::min(map_x, map_info_.width - 1));
    map_y = std::max(0, std::min(map_y, map_info_.height - 1));

    return {map_y, map_x}; // 注意：返回(行, 列)，即(y, x)
}

// 将栅格坐标转换为世界坐标
std::pair<double, double> AStarPlanner::mapToWorld(int map_x, int map_y) const {
    double world_x = map_x * map_info_.resolution + map_info_.origin_x;
    double world_y = map_y * map_info_.resolution + map_info_.origin_y;
    return {world_x, world_y};
}

// 启发函数：曼哈顿距离
double AStarPlanner::heuristic(const std::pair<int, int>& a, const std::pair<int, int>& b) const {
    // 曼哈顿距离
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

// 执行A*路径规划
std::vector<std::pair<int, int>> AStarPlanner::planPath(
    const std::pair<int, int>& start, 
    const std::pair<int, int>& goal) {
    
    if (map_data_.empty() || map_data_[0].empty()) {
        log("错误：地图数据为空");
        return {};
    }
    
    // 检查起点和终点是否在地图范围内
    if (start.first < 0 || start.first >= static_cast<int>(map_data_.size()) ||
        start.second < 0 || start.second >= static_cast<int>(map_data_[0].size())) {
        log("错误：起点超出地图范围");
        return {};
    }
    
    if (goal.first < 0 || goal.first >= static_cast<int>(map_data_.size()) ||
        goal.second < 0 || goal.second >= static_cast<int>(map_data_[0].size())) {
        log("错误：终点超出地图范围");
        return {};
    }
    
    // 检查起点和终点是否在障碍物上
    if (map_data_[start.first][start.second] != 0) {
        log("错误：起点位于障碍物上");
        return {};
    }
    
    if (map_data_[goal.first][goal.second] != 0) {
        log("错误：终点位于障碍物上");
        return {};
    }
    
    log("开始路径规划...");
    
    std::priority_queue<AStarNode> open_list;
    std::set<std::pair<int, int>> closed_set;
    
    auto start_node = std::make_shared<AStarNode>(start);
    
    open_list.push(*start_node);
    
    // 方向: 上、下、右、左
    const std::vector<std::pair<int, int>> directions = {
        {-1, 0}, {1, 0}, {0, 1}, {0, -1}
    };
    
    while (!open_list.empty()) {
        auto current_node = open_list.top();
        open_list.pop();
        
        // 如果当前节点已在闭集中，跳过
        if (closed_set.find(current_node.position) != closed_set.end()) {
            continue;
        }
        
        // 将当前节点加入闭集
        closed_set.insert(current_node.position);
        
        // 检查是否到达目标
        if (current_node.position == goal) {
            // 构建路径
            std::vector<std::pair<int, int>> path;
            auto node = std::make_shared<AStarNode>(current_node);
            
            while (node != nullptr) {
                path.push_back(node->position);
                node = node->parent;
            }
            
            std::reverse(path.begin(), path.end());
            
            std::stringstream ss;
            ss << "路径规划成功，路径长度: " << path.size();
            log(ss.str());
            
            return path;
        }
        
        // 探索邻居
        for (const auto& dir : directions) {
            std::pair<int, int> neighbor_pos = {
                current_node.position.first + dir.first,
                current_node.position.second + dir.second
            };
            
            // 检查边界和障碍物
            if (neighbor_pos.first < 0 || neighbor_pos.first >= static_cast<int>(map_data_.size()) ||
                neighbor_pos.second < 0 || neighbor_pos.second >= static_cast<int>(map_data_[0].size()) ||
                map_data_[neighbor_pos.first][neighbor_pos.second] != 0 ||
                closed_set.find(neighbor_pos) != closed_set.end()) {
                continue;
            }
            
            // 创建邻居节点
            auto neighbor_node = std::make_shared<AStarNode>(neighbor_pos);
            neighbor_node->parent = std::make_shared<AStarNode>(current_node);
            neighbor_node->g = current_node.g + 1;
            neighbor_node->h = heuristic(neighbor_pos, goal);
            neighbor_node->f = neighbor_node->g + neighbor_node->h;
            
            // 将邻居加入开集
            open_list.push(*neighbor_node);
        }
    }
    
    log("未找到从起点到终点的路径");
    return {};
}


// 执行A*路径规划，传入世界坐标（米），返回世界坐标路径
std::vector<std::pair<double, double>> AStarPlanner::planPathWorld(
    double start_x, double start_y, 
    double goal_x, double goal_y) {
    
    // 将世界坐标转换为栅格坐标
    auto start_grid = worldToMap(start_x, start_y);
    auto goal_grid = worldToMap(goal_x, goal_y);
    
    std::stringstream ss;
    ss << "世界坐标起点 (" << start_x << ", " << start_y << ") -> 栅格坐标 (" 
       << start_grid.first << ", " << start_grid.second << ")";
    log(ss.str());
    
    ss.str("");
    ss << "世界坐标终点 (" << goal_x << ", " << goal_y << ") -> 栅格坐标 (" 
       << goal_grid.first << ", " << goal_grid.second << ")";
    log(ss.str());
    
    // 使用栅格坐标进行路径规划
    auto grid_path = planPath(start_grid, goal_grid);
    
    // 将栅格坐标路径转换回世界坐标
    std::vector<std::pair<double, double>> world_path;
    for (const auto& grid_point : grid_path) {
        auto world_point = mapToWorld(grid_point.second, grid_point.first);  // 注意这里交换了x和y
        world_path.push_back(world_point);
    }
    
    return world_path;
}