#pragma once

#include "gcopter/trajectory.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// 发布起终点、离散路径、连续轨迹和当前位置的 RViz Marker。
class Visualizer
{
public:
  explicit Visualizer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    rclcpp::QoS result_qos(rclcpp::KeepLast(1));
    result_qos.reliable();
    result_qos.transient_local();
    start_goal_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/visualizer/start_goal", result_qos);
    route_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/visualizer/route", result_qos);
    trajectory_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/visualizer/trajectory", result_qos);

    rclcpp::QoS current_qos(rclcpp::KeepLast(1));
    current_qos.reliable();
    current_position_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/visualizer/current_position", current_qos);
  }

  // 发布绿色起点和红色终点。
  void visualizeStartGoal(const Eigen::Vector2d & start, const Eigen::Vector2d & goal) const
  {
    auto marker = makeMarker("start_goal", visualization_msgs::msg::Marker::SPHERE_LIST);
    marker.scale.x = 0.24;
    marker.scale.y = 0.24;
    marker.scale.z = 0.24;
    marker.points.push_back(makePoint(start, 0.25));
    marker.points.push_back(makePoint(goal, 0.25));

    std_msgs::msg::ColorRGBA start_color;
    start_color.r = 0.1F;
    start_color.g = 0.9F;
    start_color.b = 0.2F;
    start_color.a = 1.0F;
    std_msgs::msg::ColorRGBA goal_color;
    goal_color.r = 0.95F;
    goal_color.g = 0.2F;
    goal_color.b = 0.15F;
    goal_color.a = 1.0F;
    marker.colors = {start_color, goal_color};
    start_goal_pub_->publish(marker);
  }

  // 发布前端搜索和采样后的离散路径。
  void visualizeRoute(const std::vector<Eigen::Vector2d> & route) const
  {
    auto marker = makeMarker("route", visualization_msgs::msg::Marker::LINE_STRIP);
    marker.scale.x = 0.04;
    marker.color.r = 0.95F;
    marker.color.g = 0.75F;
    marker.color.b = 0.1F;
    marker.color.a = 1.0F;
    marker.points.reserve(route.size());
    for (const Eigen::Vector2d & point : route) {
      marker.points.push_back(makePoint(point, 0.22));
    }
    route_pub_->publish(marker);
  }

  // 将连续轨迹按时间采样后发布为折线。
  void visualizeTrajectory(const Trajectory<5, 2> & trajectory) const
  {
    auto marker = makeMarker("trajectory", visualization_msgs::msg::Marker::LINE_STRIP);
    marker.scale.x = 0.07;
    marker.color.r = 0.0F;
    marker.color.g = 0.5F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    if (trajectory.getPieceNum() > 0) {
      constexpr double kSampleDt = 0.02;
      const double total = trajectory.getTotalDuration();
      for (double time = 0.0; time < total; time += kSampleDt) {
        marker.points.push_back(makePoint(trajectory.getPos(time).head<2>(), 0.28));
      }
      marker.points.push_back(makePoint(trajectory.getPos(total).head<2>(), 0.28));
    }
    trajectory_pub_->publish(marker);
  }

  // 发布 odom 或轨迹时间推进得到的当前位置。
  void visualizeCurrentPosition(const Eigen::Vector2d & position) const
  {
    auto marker = makeMarker("current_position", visualization_msgs::msg::Marker::SPHERE);
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = 0.34;
    marker.scale.x = 0.20;
    marker.scale.y = 0.20;
    marker.scale.z = 0.20;
    marker.color.r = 0.75F;
    marker.color.g = 0.1F;
    marker.color.b = 0.95F;
    marker.color.a = 1.0F;
    current_position_pub_->publish(marker);
  }

private:
  // 创建使用 map 坐标系的基础 Marker。
  visualization_msgs::msg::Marker makeMarker(const std::string & name, int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = node_->now();
    marker.header.frame_id = "map";
    marker.ns = "planner_forge/" + name;
    marker.id = 0;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  // 将二维坐标转换为带显示高度的 Point 消息。
  static geometry_msgs::msg::Point makePoint(const Eigen::Vector2d & point, double height)
  {
    geometry_msgs::msg::Point message;
    message.x = point.x();
    message.y = point.y();
    message.z = height;
    return message;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr route_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr current_position_pub_;
};
