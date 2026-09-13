#pragma once

#include <opencv2/core.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <plan_interfaces/msg/dynamic_obstacle_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace simple_map
{

// 静态图片转换和动态障碍生成所需的地图参数。
struct Config
{
  std::string image_path;                         // 运行时地图图片路径
  int obstacle_gray_threshold = 128;              // 小于该灰度值的像素视为障碍
  std::vector<double> map_bounds_m;                // 地图边界，单位 m，格式沿用 cloud_bridge
  std::vector<double> image_origin_m;              // 图片逻辑原点在 map 中的位置，单位 m
  double image_resolution_m_per_pixel = 0.05;      // 图片分辨率，单位 m/pixel
  double map_resolution_m_per_cell = 0.05;         // 输出栅格分辨率，单位 m/cell
  bool generate_dynamic_obstacles = false;         // 是否叠加框架自带的动态障碍示例
};

// 将灰度地图图片转换为 OccupancyGrid，reason 返回输入错误字段。
bool BuildStaticMap(
  const Config & config,
  const cv::Mat & static_image,
  nav_msgs::msg::OccupancyGrid & output,
  std::string & reason);

// 读取静态图片并发布 /grid_map，可选叠加定时更新的动态障碍。
class SimpleMapNode : public rclcpp::Node
{
public:
  // 读取参数和静态图片，建立 /grid_map 发布器。
  explicit SimpleMapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 返回节点是否完成地图初始化。
  bool initialized() const {return initialized_;}

  // 在静态地图副本上写入当前时刻的多个移动矩形障碍。
  void updateDynamicObstacles(
    const rclcpp::Time & now,
    nav_msgs::msg::OccupancyGrid & map) const;

private:
  // 读取并检查地图参数。
  Config readConfig();
  bool validateConfig(std::string & reason) const;

  // 发布静态地图，或按定时器发布静态与动态障碍的合成地图。
  void publishStaticMap();
  void timerCallback();

  // 配置和两份地图数据。
  Config config_;
  cv::Mat static_image_;
  nav_msgs::msg::OccupancyGrid static_map_;
  nav_msgs::msg::OccupancyGrid combined_map_;

  // ROS 对象和动态障碍运动的时间基准。
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<plan_interfaces::msg::DynamicObstacleArray>::SharedPtr dynamic_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time dynamic_start_time_;
  bool initialized_ = false;
};

}  // namespace simple_map
