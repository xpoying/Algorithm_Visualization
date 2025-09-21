#include <algorithm>
#include <math.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include <cmath>

// 地图定义（网格）
class Map_Defintion
{
public:
   // 地图大小
   const int Map_Width = 1000;  // 地图宽度
   const int Map_Height = 1000; // 地图高度
   // 地图点位
   const std::vector<cv::Point2f> obstacles =
       {{20, 20}, {20, 30}, {20, 40}, {30, 20}, {30, 30}, {30, 40}, {60, 60}, {60, 70}, {70, 60}, {70, 70}}; // 地图障碍物
   cv::Point2f start = {10, 0};                                                                              // 起点
   cv::Point2f end = {490, 500};                                                                             // 终点
};

// A*定义
class Astart
{
public:
   bool IsValid(const cv::Point2f &p, const std::unique_ptr<Map_Defintion> &map_);      // 判断是否遇到障碍物或在地图内
   float distance(const cv::Point2f &p1, const cv::Point2f &p2);                        // 计算距离
   float heuristic(const cv::Point2f &current, const cv::Point2f &end);                 // 启发函数
   std::vector<cv::Point2f> Astart_exected(const std::unique_ptr<Map_Defintion> &map_); // 主函数
                                                                                        // 自定义哈希函数
   struct PointHash
   {
      std::size_t operator()(const cv::Point2f &p) const
      {
         return std::hash<int>()(static_cast<int>(p.x)) ^ std::hash<int>()(static_cast<int>(p.y));
      }
   };
   // 自定义比较函数
   struct PointEqual
   {
      bool operator()(const cv::Point2f &p1, const cv::Point2f &p2)
      {
         return p1.x == p2.x && p1.y == p2.y;
      }
   };
};

// 距离
float Astart::distance(const cv::Point2f &p1, const cv::Point2f &p2)
{
   return std::abs((p1.x - p2.x)) + std::abs((p1.y - p2.y));
}
// 启发函数
float Astart::heuristic(const cv::Point2f &current, const cv::Point2f &end)
{
   return distance(current, end);
}
// 探索点是否合法
bool Astart::IsValid(const cv::Point2f &p, const std::unique_ptr<Map_Defintion> &map_)
{
   return p.x < map_->Map_Width && p.y < map_->Map_Height && p.x > 0 && p.y > 0 &&
          std::find(map_->obstacles.begin(), map_->obstacles.end(), p) == map_->obstacles.end();
}
// 执行函数
std::vector<cv::Point2f> Astart::Astart_exected(const std::unique_ptr<Map_Defintion> &map_)
{
   auto start = map_->start; // 开始点
   auto end = map_->end;     // 终点

   auto cmp = [](const std::pair<cv::Point2f, float> &a, const std::pair<cv::Point2f, float> &b)
   {
      return a.second > b.second;
   };
   // 优先队列储存待探索节点，保证从队列中取出代价最小的节点
   std::priority_queue<std::pair<cv::Point2f, float>, std::vector<std::pair<cv::Point2f, float>>, decltype(cmp)> open_set(cmp);
   // 储存已经探索过的节点    f(x)= g(x) + h(x)
   std::unordered_map<cv::Point2f, float, PointHash, PointEqual> g_cost;
   // 建立节点关系
   std::unordered_map<cv::Point2f, cv::Point2f, PointHash, PointEqual> came_form;
   // 初始化
   open_set.push({start, 0});
   g_cost[start] = 0;

   // 可视化地图
   cv::Mat val_map(map_->Map_Height, map_->Map_Width, CV_8UC3, cv::Scalar(255, 255, 255));
   for (const auto &obstacle : map_->obstacles)
   {
      cv::rectangle(val_map, obstacle, obstacle, cv::Scalar(0, 0, 0), -1);
   }
   cv::namedWindow("A*", cv::WINDOW_AUTOSIZE);

   while (!open_set.empty())
   {
      cv::Point2f current = open_set.top().first;
      open_set.pop();
      // 如果到达节点，重建路径
      if (current == end)
      {
         std::vector<cv::Point2f> path;
         while (current != start)
         {
            path.push_back(current);
            current = came_form[current];
         }
         path.push_back(start);
         std::reverse(path.begin(), path.end());
         // 绘制起点和终点
         cv::circle(val_map, start, 5, cv::Scalar(0, 255, 0), -1); // 绿色，半径5
         cv::circle(val_map, end, 5, cv::Scalar(0, 0, 255), -1);   // 红色，半径5

         // 绘制路径
         for (size_t i = 1; i < path.size(); ++i)
         {
            cv::line(val_map, path[i - 1], path[i], cv::Scalar(255, 0, 0), 2); // 蓝色，线宽2
         }
         cv::imshow("A*", val_map);
         return path;
      }
      // 邻接点
      std::vector<cv::Point2f> neighbors = {
          {current.x - 10, current.y}, {current.x + 10, current.y}, {current.x, current.y - 10}, {current.x, current.y + 10}};
      for (const auto &neighbor : neighbors)
      {
         if (IsValid(neighbor, map_))
         {
            float tentative_g_cost = g_cost[current] + 10;
            if (g_cost.find(neighbor) == g_cost.end() || tentative_g_cost < g_cost[neighbor])
            {
               g_cost[neighbor] = tentative_g_cost;
               float f_cost = tentative_g_cost*0.4 + heuristic(neighbor, end)*0.6;
               open_set.push({neighbor, f_cost});
               came_form[neighbor] = current;
            }
            cv::rectangle(val_map, neighbor, neighbor, cv::Scalar(255, 255, 0), -1);
            cv::imshow("A*", val_map);
            cv::waitKey(500);
         }
      }
   }
   return {}; // 未找到路径
}
int main()
{
   std::unique_ptr<Astart> astart_;
   std::unique_ptr<Map_Defintion> map_;
   std::vector<cv::Point2f> path = astart_->Astart_exected(map_);
   ;

   if (!path.empty())
   {
      std::cout << "Path found!" << std::endl;
   }
   else
   {
      std::cout << "No path found." << std::endl;
   }
   return 0;
}