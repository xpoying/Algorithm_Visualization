#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

// 地图定义（网格）
class MapDefinition
{
public:
    const int MapWidth = 500;
    const int MapHeight = 500;
    const std::vector<cv::Point> obstacles = {
        {50, 50},
        {50, 60},
        {50, 70},
        {50, 80},
        {50, 90},
        {50, 100},
        {60, 50},
        {70, 50},
        {80, 50},
        {90, 50},
        {100, 50},
        {50, 40},
        {50, 30},
        {50, 20},
        {50, 10},
        {40, 100},
        {30, 100},
        {20, 100}};
    cv::Point start = {10, 10};
    cv::Point end = {490, 490};
};

class AStar
{
public:
    bool IsValid(const cv::Point &p, const std::unique_ptr<MapDefinition> &map_);
    float heuristic(const cv::Point &current, const cv::Point &end);
    std::vector<cv::Point> AStarExecute(const std::unique_ptr<MapDefinition> &map_);

    struct PointHash
    {
        std::size_t operator()(const cv::Point &p) const
        {
            return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
        }
    };
};

float AStar::heuristic(const cv::Point &current, const cv::Point &end)
{
    return std::abs(current.x - end.x) + std::abs(current.y - end.y); // 曼哈顿距离
}

bool AStar::IsValid(const cv::Point &p, const std::unique_ptr<MapDefinition> &map_)
{
    // 边界检查
    if (p.x < 0 || p.x >= map_->MapWidth || p.y < 0 || p.y >= map_->MapHeight)
        return false;

    // 障碍物检查
    return std::find(map_->obstacles.begin(), map_->obstacles.end(), p) == map_->obstacles.end();
}

std::vector<cv::Point> AStar::AStarExecute(const std::unique_ptr<MapDefinition> &map_)
{
    auto start = map_->start;
    auto end = map_->end;

    // 优先队列 (f_cost, point)
    auto cmp = [](const std::pair<float, cv::Point> &a, const std::pair<float, cv::Point> &b)
    {
        return a.first > b.first;
    };
    std::priority_queue<std::pair<float, cv::Point>,
                        std::vector<std::pair<float, cv::Point>>,
                        decltype(cmp)>
        open_set(cmp);

    std::unordered_map<cv::Point, float, PointHash> g_cost;        // 实际代价
    std::unordered_map<cv::Point, cv::Point, PointHash> came_from; // 路径回溯
    std::unordered_set<cv::Point, PointHash> closed_set;           // 已处理节点

    // 初始化
    open_set.push({0, start});
    g_cost[start] = 0;

    // 可视化初始化
    cv::Mat vis_map(map_->MapHeight, map_->MapWidth, CV_8UC3, cv::Scalar(255, 255, 255));
    const int obstacleSize = 5;
    for (const auto &obs : map_->obstacles)
    {
        cv::rectangle(vis_map,
                      cv::Rect(obs.x - obstacleSize / 2, obs.y - obstacleSize / 2, obstacleSize, obstacleSize),
                      cv::Scalar(0, 0, 0), -1);
    }
    cv::circle(vis_map, start, 5, cv::Scalar(0, 255, 0), -1); // 起点绿色
    cv::circle(vis_map, end, 5, cv::Scalar(0, 0, 255), -1);   // 终点红色
    cv::imshow("A* Pathfinding", vis_map);
    cv::waitKey(1);

    // 方向向量（四连通）
    const std::vector<cv::Point> directions = {{10, 0}, {-10, 0}, {0, 10}, {0, -10}};

    while (!open_set.empty())
    {

        auto current = open_set.top().second;
        open_set.pop();

        // 跳过已处理节点
        if (closed_set.count(current))
            continue;
        closed_set.insert(current);

        // 绘制当前节点（蓝色）
        if (current != start && current != end)
        {
            cv::circle(vis_map, current, 2, cv::Scalar(255, 0, 0), -1);
            cv::imshow("A* Pathfinding", vis_map);
            cv::waitKey(10); // 延迟
        }

        // 找到路径
        if (current == end)
        {
            std::vector<cv::Point> path;
            while (current != start)
            {
                path.push_back(current);
                current = came_from[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());

            // 绘制最终路径
            for (size_t i = 1; i < path.size(); ++i)
            {
                cv::line(vis_map, path[i - 1], path[i], cv::Scalar(255, 0, 0), 2);
            }
            cv::imshow("A* Pathfinding", vis_map);
            cv::waitKey(0);
            return path;
        }

        // 探索邻居
        for (const auto &dir : directions)
        {
            cv::Point neighbor(current.x + dir.x, current.y + dir.y);

            if (!IsValid(neighbor, map_) || closed_set.count(neighbor))
                continue;

            float tentative_g = g_cost[current] + 10; // 移动代价
            if (!g_cost.count(neighbor) || tentative_g < g_cost[neighbor])
            {
                came_from[neighbor] = current;
                g_cost[neighbor] = tentative_g;
                float f_cost = tentative_g * 0.4 + heuristic(neighbor, end) * 0.6;
                open_set.push({f_cost, neighbor});

                // 绘制新发现的节点（黄色）
                cv::circle(vis_map, neighbor, 2, cv::Scalar(0, 255, 255), -1);
            }
        }
        cv::imshow("A* Pathfinding", vis_map);
        cv::waitKey(1);
    }
    return {}; // 无路径
}

int main()
{

    std::unique_ptr<MapDefinition> map_ =std::make_unique<MapDefinition>();
    std::unique_ptr<AStar> astar_ = std::make_unique<AStar>();
    std::vector<cv::Point> path = astar_->AStarExecute(map_);
    std::cout << "Path " << (path.empty() ? "not " : "") << "found!" << std::endl;

    return 0;
}