#include <vector>
#include "opencv2/opencv.hpp"
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <memory>
#include <limits>

class MapDefinition
{
public:
    int MAP_WIDTH = 500;
    int MAP_HEIGHT = 500;
    cv::Point2f start = cv::Point2f(10, 10);
    cv::Point2f end = cv::Point2f(490, 490);

    struct CircularObstacle
    {
        cv::Point2f center;
        float radius;
    };

    std::vector<CircularObstacle> circularObstacles;
};

class RRTStar
{
public:
    // 节点定义
    struct Node
    {
        cv::Point2f point;
        std::shared_ptr<Node> parent;
        Node(const cv::Point2f &p) : point(p), parent(nullptr) {}
    };
    using NodeList = std::vector<std::shared_ptr<Node>>;
    // 类初始化
    RRTStar(const std::shared_ptr<MapDefinition> &map) : map_(map) {}
    // 判断有效点位
    bool isValidPoint(const cv::Point2f &p) const;

    // 判断有效路径
    bool isValidPath(const std::shared_ptr<Node> &newNode, const std::shared_ptr<Node> &nearestNode) const;

    // 随机点生成 概率偏向终点
    cv::Point2f randomPoint(float biasProbability) const;
    // 查找最近节点
    std::shared_ptr<Node> nearestNode(const cv::Point2f &point, const NodeList &nodes) const;
    // 扩展树
    void extendTree(std::shared_ptr<Node> newNode, const std::shared_ptr<Node> &nearestNode, float maxStepLength);
    // 路径
    std::vector<cv::Point2f> extractPath(const std::shared_ptr<Node> &goalNode) const;
    // 执行
    void plan();

private:
    std::shared_ptr<MapDefinition> map_;
    NodeList tree_;
};
bool RRTStar::isValidPoint(const cv::Point2f &p) const
{
    if (p.x < 0 || p.x >= map_->MAP_WIDTH || p.y < 0 || p.y >= map_->MAP_HEIGHT)
    {
        return false;
    }

    for (const auto &obstacle : map_->circularObstacles)
    {
        float distance = cv::norm(p - obstacle.center);
        if (distance <= obstacle.radius)
        {
            return false;
        }
    }

    return true;
}
bool RRTStar::isValidPath(const std::shared_ptr<RRTStar::Node> &newNode, const std::shared_ptr<RRTStar::Node> &nearestNode) const
{
    int numSamples = 10;
    cv::Point2f direction = newNode->point - nearestNode->point;
    float distance = cv::norm(direction);
    float step = distance / numSamples;

    for (int i = 1; i <= numSamples; ++i)
    {
        cv::Point2f samplePoint = nearestNode->point + (direction / distance) * (i * step);
        if (!isValidPoint(samplePoint))
        {
            return false;
        }
    }

    return true;
}

cv::Point2f RRTStar::randomPoint(float biasProbability) const
{
    if (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) < biasProbability)
    {
        return map_->end;
    }
    else
    {
        cv::Point2f randomPoint;
        randomPoint.x = static_cast<float>(rand() % map_->MAP_WIDTH);
        randomPoint.y = static_cast<float>(rand() % map_->MAP_HEIGHT);
        return randomPoint;
    }
}

std::shared_ptr<RRTStar::Node> RRTStar::nearestNode(const cv::Point2f &point, const NodeList &nodes) const
{
    if (nodes.empty())
        return nullptr;

    auto nearest = nodes[0];
    float minDist = cv::norm(point - nearest->point);

    for (const auto &node : nodes)
    {
        float dist = cv::norm(point - node->point);
        if (dist < minDist)
        {
            minDist = dist;
            nearest = node;
        }
    }

    return nearest;
}

void RRTStar::extendTree(std::shared_ptr<RRTStar::Node> newNode, const std::shared_ptr<RRTStar::Node> &nearestNode,
                         float maxStepLength)
{
    cv::Point2f direction = newNode->point - nearestNode->point;
    float distance = cv::norm(direction);

    if (distance > maxStepLength)
    {
        direction = (direction / distance) * maxStepLength;
        newNode->point = nearestNode->point + direction;
    }

    if (isValidPath(newNode, nearestNode))
    {
        newNode->parent = nearestNode;
        tree_.push_back(newNode);
    }
}

std::vector<cv::Point2f> RRTStar::extractPath(const std::shared_ptr<RRTStar::Node> &goalNode) const
{
    std::vector<cv::Point2f> path;
    auto current = goalNode;
    while (current != nullptr)
    {
        path.push_back(current->point);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void RRTStar::plan()
{
    //初始化数值
    tree_.clear();
    auto startNode = std::make_shared<Node>(map_->start);
    tree_.push_back(startNode);
    const int maxIterations = 1000;
    const float biasProbability = 0.1f;
    const float maxStepLength = 15.0f;
    //可视化
    cv::Mat mapVis(map_->MAP_HEIGHT, map_->MAP_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::namedWindow("RRT Path Planning", cv::WINDOW_AUTOSIZE);
    for (const auto &obstacle : map_->circularObstacles)
    {
        cv::circle(mapVis, obstacle.center, static_cast<int>(obstacle.radius), cv::Scalar(0, 0, 0), -1);
    }
    cv::circle(mapVis, map_->start, 5, cv::Scalar(0, 255, 0), -1);
    cv::circle(mapVis, map_->end, 5, cv::Scalar(0, 0, 255), -1);
    cv::imshow("RRT Path Planning", mapVis);
    cv::waitKey(1000);

    //随机点采样
    for (int i = 0; i < maxIterations; ++i)
    {
        cv::Point2f randPoint = randomPoint(biasProbability);
        //判断是否合法
        if (!isValidPoint(randPoint))
        {
            continue;
        }
        auto nearest = nearestNode(randPoint, tree_);
        if (!nearest)
        {
            continue;
        }
        //建立节点
        auto new_node = std::make_shared<Node>(randPoint);
        extendTree(new_node, nearest, maxStepLength);

        if (new_node->parent)
        {
            //可视化
            cv::circle(mapVis, new_node->point, 3, cv::Scalar(255, 0, 0), -1);
            cv::line(mapVis, nearest->point, new_node->point, cv::Scalar(0, 0, 255), 1);
            cv::imshow("RRT Path Planning", mapVis);
            cv::waitKey(10);
            //与终点接近直接连线
            if (cv::norm(new_node->point - map_->end) < 10.0f)
            {
                auto path = extractPath(new_node);
                for (size_t j = 1; j < path.size(); ++j)
                {
                    cv::line(mapVis, path[j - 1], path[j], cv::Scalar(0, 255, 0), 2);
                }
                cv::imshow("RRT Path Planning", mapVis);
                cv::waitKey(0);
                return;
            }
        }
    }

    std::cout << "Path not found!" << std::endl;
    cv::waitKey(0);
}

int main()
{
    std::shared_ptr<MapDefinition> map = std::make_shared<MapDefinition>();
    map->start = cv::Point2f(10, 10);
    map->end = cv::Point2f(490, 490);

    map->circularObstacles.push_back({cv::Point2f(150, 150), 30.0f});
    map->circularObstacles.push_back({cv::Point2f(350, 350), 40.0f});
    map->circularObstacles.push_back({cv::Point2f(200, 350), 20.0f});

    RRTStar rrt(map);
    rrt.plan();

    return 0;
}