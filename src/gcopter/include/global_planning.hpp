#pragma once

#include "gcopter/grid_map_2d.hpp"
#include "gcopter/path_search.hpp"
#include "gcopter/traj_representation.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/visualizer.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <plan_interfaces/msg/minco_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

// global_planning 节点使用的 topic、规划模式和标称速度参数。
struct Config
{
  std::string mapTopic;                  // OccupancyGrid 输入
  std::string targetTopic;               // PoseStamped 起终点输入
  std::string odomTopic;                 // map 坐标 odom 输入或离线输出
  bool is_omni = true;                   // true 八邻接，false 四邻接
  bool is_plan_from_ego_pose = false;     // 起点是否取实时 odom
  bool publish_trajectory_odom = true;    // false 模式是否发布演示 odom
  double maxVelMag = 0.5;                // 基线时间分配使用的标称速度，单位 m/s

  // 从 ROS 参数服务器读取配置。
  explicit Config(const rclcpp::Node::SharedPtr & node)
  {
    node->declare_parameter("gcopter.MapTopic", std::string("/grid_map"));
    node->declare_parameter("gcopter.TargetTopic", std::string("/goal_pose"));
    node->declare_parameter("gcopter.OdomTopic", std::string("/Odometry_to_base_link"));
    node->declare_parameter("gcopter.is_omni", true);
    node->declare_parameter("gcopter.is_plan_from_ego_pose", false);
    node->declare_parameter("gcopter.PublishTrajectoryOdom", true);
    node->declare_parameter("gcopter.MaxVelMag", 0.5);

    mapTopic = node->get_parameter("gcopter.MapTopic").as_string();
    targetTopic = node->get_parameter("gcopter.TargetTopic").as_string();
    odomTopic = node->get_parameter("gcopter.OdomTopic").as_string();
    is_omni = node->get_parameter("gcopter.is_omni").as_bool();
    is_plan_from_ego_pose = node->get_parameter("gcopter.is_plan_from_ego_pose").as_bool();
    publish_trajectory_odom = node->get_parameter("gcopter.PublishTrajectoryOdom").as_bool();
    maxVelMag = node->get_parameter("gcopter.MaxVelMag").as_double();
  }

  // 检查 topic 名称和标称速度。
  bool valid() const
  {
    return !mapTopic.empty() && !targetTopic.empty() && !odomTopic.empty() &&
      std::isfinite(maxVelMag) && maxVelMag > 0.0;
  }
};

// 计算当前轨迹采样时刻，并限制在轨迹有效时间范围内。
inline double clampTrajectorySampleTime(double now, double trajectory_stamp, double total_duration)
{
  if (!std::isfinite(now) || !std::isfinite(trajectory_stamp) ||
    !std::isfinite(total_duration) || total_duration <= 0.0)
  {
    return 0.0;
  }
  return std::clamp(now - trajectory_stamp, 0.0, total_duration);
}

// 组织地图、目标、odom、路径搜索、轨迹生成和轨迹发布。
class GlobalPlanner
{
public:
  GlobalPlanner(const Config & config, const rclcpp::Node::SharedPtr & node);

  // 接收 map 坐标系下的里程计。
  void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);

  // 接收 OccupancyGrid 并转换为规划器内部地图。
  void mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  // 接收起终点或 ego 模式下的目标点。
  void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // 推进轨迹时间，并处理规划与重规划触发。
  void FSMCallBack_Timer();

  // 调用 PathSearch 生成离散路径。
  bool FrontSearch(
    const Eigen::Vector2d & start,
    const Eigen::Vector2d & goal,
    std::vector<Eigen::Vector2d> & route);

  // 执行一次完整规划并发布成功结果。
  bool plan(const Eigen::Vector2d & start, const Eigen::Vector2d & goal);

  // 根据后端数据生成可按时间查询的连续轨迹。
  bool buildContinuousTrajectory(
    const MincoTrajectoryData & trajectory_data,
    Trajectory<5, 2> & trajectory) const;

  // 检查轨迹指定时刻之后的采样点是否落入障碍。
  bool hasTrajectoryCollision(
    const Trajectory<5, 2> & trajectory,
    double start_time) const;

  // 填写连接点空间标志和过洞相关占位信息。
  void updateWaypointSpaciousFlags(MincoTrajectoryData & trajectory_data) const;

  // 将内部轨迹数据转换并发布为 MincoTrajectory 消息。
  void publishMincoTrajectory(const MincoTrajectoryData & trajectory_data);

  // 在连续轨迹上采样并生成 odom 消息。
  bool fillTrajectoryOdom(
    const Trajectory<5, 2> & source_trajectory,
    double elapsed_time,
    nav_msgs::msg::Odometry & odom_message);

  // 发布 false 模式使用的轨迹 odom。
  void publishTrajectoryOdom(double elapsed_time);

private:
  // 节点配置和 ROS 辅助对象。
  Config config_;
  rclcpp::Node::SharedPtr node_;
  Visualizer visualizer_;

  // ROS 输入、输出和定时器。
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  rclcpp::Publisher<plan_interfaces::msg::MincoTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr trajectory_odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 地图、规划结果和当前位置。
  GridMap2D grid_map_;
  PathSearch path_search_;
  MincoTrajectoryData initial_trajectory_data_;
  MincoTrajectoryData output_trajectory_data_;
  Trajectory<5, 2> continuous_trajectory_;
  std::vector<Eigen::Vector2d> startGoal_;
  nav_msgs::msg::Odometry odom_;

  // 节点状态。
  bool config_valid_ = false;             // 参数已经通过检查
  bool mapInitialized_ = false;           // 已收到合法地图
  bool odomInitialized_ = false;          // 已收到或生成合法 odom
  bool goal_changed_ = false;             // 已形成一组新的起终点
  bool map_changed_ = false;               // 已收到新地图，等待检查剩余轨迹
  bool have_plan_ = false;                 // 当前持有一条成功发布的轨迹
  double trajStamp_ = 0.0;                 // 当前轨迹起始 ROS 时间，单位 s
  double trajectory_odom_yaw_ = 0.0;       // 低速时保持的上一帧车体 yaw
};
