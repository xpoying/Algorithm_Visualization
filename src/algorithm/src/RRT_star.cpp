#include <vector>
#include "opencv2/opencv.hpp"

class MapDefinition
{
public:
    int MAP_WIDTH = 500;
    int MAP_HEIGHT = 500;
    std::vector<cv::Point2f> obstacles;
    cv::Point2f start;
    cv::Point2f end;
    struct cri_obstacles
    {
       cv::Point2f center;
       float radius;
    };
    
};

class RRT_star
{
public:
    // 初始化
    RRT_star(const std::shared_ptr<MapDefinition> &map) : map_(map) {};

    // 节点定义
    struct Node
    {
        cv::Point2f point;
        std::shared_ptr<Node> parent;
        Node(const cv::Point2f &p) : point(p), parent(nullptr) {};
    };
    using NodeList = std::vector<std::shared_ptr<Node>>;

    // 哈希
    struct PointHash
    {
        std::size_t operator()(const cv::Point2f &p)
        {
            return std::hash<int>()(static_cast<int>(p.x)) ^ (std::hash<int>()(static_cast<int>(p.y)) << 1);
        }
    };
    // 查看点位是否合法
    bool PointIsValid(const cv::Point2f &p);
    // 路径是否有效
    bool NodeIsValid(const std::shared_ptr<Node>&newNode,const std::shared_ptr<Node>&nearestNode);
    // 随机点位生成
    cv::Point2f RandomPoint(float biasProbability);
    // 树扩展
    void ExtendTree(std::shared_ptr<Node> node, const std::shared_ptr<Node> &nearestNode, float maxStepLength);
    // 寻找最近节点
    std::shared_ptr<Node> NearestNode(const cv::Point2f &point, const NodeList &nodes);
    // 提取路径
    std::vector<cv::Point2f> ExtractPath(const std::shared_ptr<Node> &goalNode) const;
    // 执行
    void RRT_Exected();

    NodeList tree_;
    std::shared_ptr<MapDefinition> map_;
};
// 寻找最近节点
std::shared_ptr<RRT_star::Node> RRT_star::NearestNode(const cv::Point2f &point, const NodeList &nodes)
{
    if (nodes.empty())
        return nullptr;

    auto nearest = nodes[0];
    float minDist = cv::norm(point - nearest->point);

    for (const auto &node : nodes)
    {
        float dis = cv::norm(point - node->point);
        if (dis < minDist)
        {
            minDist = dis;
            nearest = node;
        }
    }
    return nearest;
}

void RRT_star::ExtendTree(std::shared_ptr<Node> newNode, const std::shared_ptr<Node> &nearestNode, float maxStepLength)
{ // 方向向量
    cv::Point2f direction = newNode->point - nearestNode->point;
    float distance = cv::norm(direction);
    if (distance > maxStepLength)
    {
        direction = (direction / distance) * maxStepLength;
        newNode->point = nearestNode->point + direction;
    }
    newNode->parent = nearestNode;
    tree_.push_back(newNode);
}

std::vector<cv::Point2f> RRT_star::ExtractPath(const std::shared_ptr<Node> &goalNode) const
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

void RRT_star::RRT_Exected()
{
    // 函数初始化
    tree_.clear();
    auto startNode = std::make_shared<Node>(map_->start);
    tree_.push_back(startNode);
    const float biasProbaility = 0.3f;
    const int maxIterations = 1000;
    const float maxStepLength = 15;

    // 地图初始化
    cv::Mat val_map(map_->MAP_WIDTH, map_->MAP_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::namedWindow("rrt", cv::WINDOW_FREERATIO);
    for (const auto &obstacle : map_->obstacles)
    {
        cv::circle(val_map, obstacle, 5, cv::Scalar(0, 0, 0), -1);
    }
    cv::circle(val_map, map_->start, 5, cv::Scalar(0, 255, 0), -1);
    cv::circle(val_map, map_->end, 5, cv::Scalar(0, 0, 255), -1);

    cv::imshow("rrt", val_map);
    cv::waitKey(1000);

    // 随机采样  找到最近节点
    for (int i = 0; i < maxIterations; i++)
    {
        cv::Point2f randomPoint = RandomPoint(biasProbaility);

        if (!PointIsValid(randomPoint))
            continue;

        auto nearestNode = NearestNode(randomPoint, tree_);
        if (!nearestNode)
            continue;

        auto newNode = std::shared_ptr<Node>(nearestNode);
        ExtendTree(newNode, nearestNode, maxStepLength);

        // 绘制节点
        cv::circle(val_map, newNode->point, 3, cv::Scalar(255, 0, 0), -1);
        cv::line(val_map, nearestNode->point, newNode->point, cv::Scalar(0, 0, 255), 1);
        cv::imshow("rrt", val_map);
        cv::waitKey(10);

        if (cv::norm(newNode->point - map_->end) < 0.5f)
        {
            std::cout << "Path Found";
            // 绘制路径
            auto path = ExtractPath(newNode);

            for (size_t j = 1; j < path.size(); ++j)
            {
                cv::line(val_map, path[j - 1], path[j], cv::Scalar(0, 255, 0), 2);
            }

            cv::imshow("rrt", val_map);
            cv::waitKey(0);
            return;
        }
    }
    std::cout << "Path not found!" << std::endl;
    cv::waitKey(0);
}

int main() {
    std::shared_ptr<MapDefinition> map = std::make_shared<MapDefinition>();
    RRT_star rrt(map);
    rrt.RRT_Exected();
    return 0;
}