#include "global_planning.hpp"

#include "gcopter/minco.hpp"
#include "gcopter/trajectory_generation.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

// 检查 OccupancyGrid 是否使用有限、轴对齐的地图原点。
bool isIdentityMapOrigin(const geometry_msgs::msg::Pose & origin)
{
  const auto & q = origin.orientation;
  if (!std::isfinite(origin.position.x) || !std::isfinite(origin.position.y) ||
    !std::isfinite(origin.position.z) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
    !std::isfinite(q.z) || !std::isfinite(q.w))
  {
    return false;
  }
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (std::abs(norm - 1.0) > 1.0e-6 || std::abs(q.x) > 1.0e-6 || std::abs(q.y) > 1.0e-6) {
    return false;
  }
  tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
  return std::abs(tf2::getYaw(quaternion)) <= 1.0e-6;
}

// 根据轨迹速度计算车体 yaw，静止时沿用上一帧方向。
double trajectoryYawToBodyYaw(const Eigen::Vector2d & velocity, double fallback_yaw)
{
  if (velocity.squaredNorm() < 1.0e-8) {
    return fallback_yaw;
  }
  return std::atan2(velocity.y(), velocity.x()) - 0.5 * std::acos(-1.0);
}

}  // namespace

GlobalPlanner::GlobalPlanner(const Config & config, const rclcpp::Node::SharedPtr & node)
: config_(config), node_(node), visualizer_(node)
{
  config_valid_ = config_.valid();
  if (!config_valid_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid PlannerForge config: topics must be non-empty and gcopter.MaxVelMag must be positive");
    return;
  }

  rclcpp::QoS target_qos(rclcpp::KeepLast(1));
  target_qos.reliable();
  target_qos.durability_volatile();
  targetSub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    config_.targetTopic, target_qos,
    std::bind(&GlobalPlanner::targetCallBack, this, std::placeholders::_1));

  odomSub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    config_.odomTopic, rclcpp::SensorDataQoS(),
    std::bind(&GlobalPlanner::odomCallBack, this, std::placeholders::_1));

  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.reliable();
  map_qos.transient_local();
  mapSub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    config_.mapTopic, map_qos,
    std::bind(&GlobalPlanner::mapCallBack, this, std::placeholders::_1));

  rclcpp::QoS trajectory_qos(rclcpp::KeepLast(20));
  trajectory_qos.reliable();
  trajectory_qos.durability_volatile();
  traj_pub_ = node_->create_publisher<plan_interfaces::msg::MincoTrajectory>(
    "/minco_trajectory", trajectory_qos);

  if (!config_.is_plan_from_ego_pose && config_.publish_trajectory_odom) {
    trajectory_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      config_.odomTopic, rclcpp::SensorDataQoS());
    RCLCPP_WARN(
      node_->get_logger(),
      "Offline trajectory odom is enabled on %s; disable it before running a real odom source",
      config_.odomTopic.c_str());
  }

  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&GlobalPlanner::FSMCallBack_Timer, this));

  RCLCPP_INFO(
    node_->get_logger(),
    "PlannerForge ready: map=%s target=%s odom=%s is_omni=%s ego_start=%s speed=%.3f m/s",
    config_.mapTopic.c_str(), config_.targetTopic.c_str(), config_.odomTopic.c_str(),
    config_.is_omni ? "true" : "false",
    config_.is_plan_from_ego_pose ? "true" : "false", config_.maxVelMag);
}

void GlobalPlanner::odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (!msg->header.frame_id.empty() && msg->header.frame_id != "map") {
    RCLCPP_ERROR(
      node_->get_logger(), "Rejected odom in frame '%s'; PlannerForge baseline expects map",
      msg->header.frame_id.c_str());
    return;
  }
  if (!std::isfinite(msg->pose.pose.position.x) || !std::isfinite(msg->pose.pose.position.y)) {
    RCLCPP_ERROR(node_->get_logger(), "Rejected odom with non-finite position");
    return;
  }
  odom_ = *msg;
  odomInitialized_ = true;
}

void GlobalPlanner::mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (msg->header.frame_id != "map") {
    RCLCPP_ERROR(
      node_->get_logger(), "Rejected grid map frame '%s'; expected map",
      msg->header.frame_id.c_str());
    return;
  }
  if (!std::isfinite(msg->info.resolution) || msg->info.resolution <= 0.0F ||
    msg->info.width == 0U || msg->info.height == 0U)
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected grid map with invalid dimensions or resolution");
    return;
  }
  const size_t expected_size =
    static_cast<size_t>(msg->info.width) * static_cast<size_t>(msg->info.height);
  if (msg->data.size() != expected_size) {
    RCLCPP_ERROR(
      node_->get_logger(), "Rejected grid map data size: expected %zu, received %zu",
      expected_size, msg->data.size());
    return;
  }
  if (msg->info.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
    msg->info.height > static_cast<uint32_t>(std::numeric_limits<int>::max()))
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected grid map dimensions above int range");
    return;
  }
  if (!isIdentityMapOrigin(msg->info.origin)) {
    RCLCPP_ERROR(node_->get_logger(), "Rejected rotated or invalid OccupancyGrid origin");
    return;
  }










  // ============================================================
  // TODO(阶段① 地图信息传输)
  // 在这里接入膨胀地图、代价地图、SDF 或自定义地图消息的转换结果。
  // 当前基线将 OccupancyGrid 中所有非零值转换为占据栅格。
  // ============================================================










  GridMap2D candidate;
  candidate.frame_id = msg->header.frame_id;
  candidate.resolution = msg->info.resolution;
  candidate.origin = Eigen::Vector2d(
    msg->info.origin.position.x, msg->info.origin.position.y);
  candidate.width = static_cast<int>(msg->info.width);
  candidate.height = static_cast<int>(msg->info.height);
  candidate.data.resize(expected_size);
  std::transform(msg->data.begin(), msg->data.end(), candidate.data.begin(), [](int8_t value) {
    return static_cast<uint8_t>(value == 0 ? 0 : 100);
  });
  if (!candidate.valid()) {
    RCLCPP_ERROR(node_->get_logger(), "Rejected grid map after internal validation");
    return;
  }

  const bool first_map = !mapInitialized_;
  grid_map_ = std::move(candidate);
  mapInitialized_ = true;
  map_changed_ = true;
  if (first_map) {
    RCLCPP_INFO(
      node_->get_logger(), "Grid map initialized: %dx%d, resolution %.3f m/cell",
      grid_map_.width, grid_map_.height, grid_map_.resolution);
  }
}

void GlobalPlanner::targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (!msg->header.frame_id.empty() && msg->header.frame_id != "map") {
    RCLCPP_ERROR(
      node_->get_logger(), "Rejected goal in frame '%s'; PlannerForge baseline does not apply TF",
      msg->header.frame_id.c_str());
    return;
  }
  const Eigen::Vector2d received(msg->pose.position.x, msg->pose.position.y);
  if (!received.allFinite()) {
    RCLCPP_ERROR(node_->get_logger(), "Rejected goal with non-finite position");
    return;
  }

  if (config_.is_plan_from_ego_pose) {
    if (!odomInitialized_) {
      RCLCPP_ERROR(node_->get_logger(), "Rejected goal because map-frame odom is not initialized");
      return;
    }
    startGoal_.clear();
    startGoal_.emplace_back(odom_.pose.pose.position.x, odom_.pose.pose.position.y);
    startGoal_.push_back(received);
    goal_changed_ = true;
    visualizer_.visualizeStartGoal(startGoal_[0], startGoal_[1]);
    return;
  }

  if (startGoal_.size() >= 2) {
    startGoal_.clear();
  }
  startGoal_.push_back(received);
  if (startGoal_.size() == 2) {
    goal_changed_ = true;
    visualizer_.visualizeStartGoal(startGoal_[0], startGoal_[1]);
  } else {
    RCLCPP_INFO(
      node_->get_logger(), "Stored false-mode start point (%.3f, %.3f); waiting for goal",
      received.x(), received.y());
  }
}

bool GlobalPlanner::FrontSearch(
  const Eigen::Vector2d & start,
  const Eigen::Vector2d & goal,
  std::vector<Eigen::Vector2d> & route)
{
  return path_search_.search(grid_map_, start, goal, config_.is_omni, route);
}

bool GlobalPlanner::plan(const Eigen::Vector2d & start, const Eigen::Vector2d & goal)
{
  if (!config_valid_ || !mapInitialized_) {
    return false;
  }

  std::vector<Eigen::Vector2d> route_raw;
  std::vector<Eigen::Vector2d> route_sampled;
  MincoTrajectoryData initial_candidate;
  MincoTrajectoryData output_candidate;
  Trajectory<5, 2> trajectory_candidate;
  std::string reason;

  if (!FrontSearch(start, goal, route_raw)) {
    RCLCPP_WARN(node_->get_logger(), "Planning failed at FrontSearch");
    return false;
  }
  if (!SamplePath(grid_map_, route_raw, route_sampled)) {
    RCLCPP_WARN(node_->get_logger(), "Planning failed at SamplePath");
    return false;
  }
  if (!GenerateTrajectory(route_sampled, config_.maxVelMag, initial_candidate)) {
    RCLCPP_WARN(node_->get_logger(), "Planning failed at GenerateTrajectory");
    return false;
  }
  updateWaypointSpaciousFlags(initial_candidate);
  if (!OptimizeTrajectory(grid_map_, initial_candidate, output_candidate)) {
    RCLCPP_WARN(node_->get_logger(), "Planning failed at OptimizeTrajectory");
    return false;
  }
  if (!ValidateTrajectoryData(output_candidate, reason)) {
    RCLCPP_WARN(
      node_->get_logger(), "Planning rejected backend output: %s", reason.c_str());
    return false;
  }
  if (!buildContinuousTrajectory(output_candidate, trajectory_candidate)) {
    RCLCPP_WARN(node_->get_logger(), "Planning failed while constructing continuous trajectory");
    return false;
  }
  if (hasTrajectoryCollision(trajectory_candidate, 0.0)) {
    RCLCPP_WARN(node_->get_logger(), "Planning rejected continuous trajectory collision");
    return false;
  }

  publishMincoTrajectory(output_candidate);
  initial_trajectory_data_ = initial_candidate;
  output_trajectory_data_ = output_candidate;
  continuous_trajectory_ = trajectory_candidate;
  have_plan_ = true;
  trajStamp_ = node_->now().seconds();
  visualizer_.visualizeStartGoal(start, goal);
  visualizer_.visualizeRoute(route_sampled);
  visualizer_.visualizeTrajectory(continuous_trajectory_);
  RCLCPP_INFO(
    node_->get_logger(), "Plan published: route_points=%zu segments=%d duration=%.3f s",
    route_sampled.size(), output_candidate.segment_count,
    continuous_trajectory_.getTotalDuration());
  return true;
}

bool GlobalPlanner::buildContinuousTrajectory(
  const MincoTrajectoryData & trajectory_data,
  Trajectory<5, 2> & trajectory) const
{
  trajectory.clear();
  std::string reason;
  if (!ValidateTrajectoryData(trajectory_data, reason)) {
    return false;
  }







  // ============================================================
  // TODO(阶段② 具体轨迹解算)
  // 在这里接入连续轨迹求解器，并生成可按时间查询的位置和各阶导数。
  // 当前基线使用 MINCO_S3NU 解算五次多项式轨迹。
  // ============================================================










  minco::MINCO_S3NU minco;
  minco.setConditions(
    trajectory_data.start_pva, trajectory_data.end_pva, trajectory_data.segment_count);
  minco.setParameters(
    trajectory_data.intermediate_positions, trajectory_data.segment_durations);
  minco.getTrajectory(trajectory);
  const double total_duration = trajectory.getTotalDuration();
  return trajectory.getPieceNum() == trajectory_data.segment_count &&
    std::isfinite(total_duration) && total_duration > 0.0;
}

bool GlobalPlanner::hasTrajectoryCollision(
  const Trajectory<5, 2> & trajectory,
  double start_time) const
{
  // 以固定时间间隔查询连续轨迹位置。
  constexpr double kCheckDt = 0.02;
  constexpr size_t kMaxCollisionSamples = 1000000U;
  if (!grid_map_.valid() || trajectory.getPieceNum() <= 0 || !std::isfinite(start_time)) {
    return true;
  }
  const double total_duration = trajectory.getTotalDuration();
  if (!std::isfinite(total_duration) || total_duration <= 0.0) {
    return true;
  }
  const double begin = std::clamp(start_time, 0.0, total_duration);
  const double remaining = total_duration - begin;
  const double requested_samples = std::ceil(remaining / kCheckDt) + 1.0;
  if (!std::isfinite(requested_samples) || requested_samples > kMaxCollisionSamples) {
    return true;
  }

  const size_t sample_count = static_cast<size_t>(requested_samples);
  for (size_t index = 0; index < sample_count; ++index) {
    const double time = std::min(total_duration, begin + static_cast<double>(index) * kCheckDt);
    const Eigen::Vector2d position = trajectory.getPos(time).head<2>();
    if (!position.allFinite() || grid_map_.isOccupied(grid_map_.worldToGrid(position))) {
      return true;
    }
  }
  const Eigen::Vector2d final_position = trajectory.getPos(total_duration).head<2>();
  return !final_position.allFinite() ||
    grid_map_.isOccupied(grid_map_.worldToGrid(final_position));
}

void GlobalPlanner::updateWaypointSpaciousFlags(MincoTrajectoryData & trajectory_data) const
{









  // ============================================================
  // TODO(阶段③ 25 cm 狗洞标志)
  // 在这里设计考虑过洞问题，当一段轨迹真正要过洞时，合理标记轨迹使其
  // 当前基线只将连接点标记为宽阔区域。
  // ============================================================












  
  trajectory_data.waypoint_is_spacious.resize(
    trajectory_data.intermediate_positions.cols());
  trajectory_data.waypoint_is_spacious.setOnes();
}

void GlobalPlanner::publishMincoTrajectory(const MincoTrajectoryData & trajectory_data)
{
  std::string reason;
  if (!ValidateTrajectoryData(trajectory_data, reason)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Refused to publish invalid trajectory: %s", reason.c_str());
    return;
  }

  plan_interfaces::msg::MincoTrajectory message;
  message.header.stamp = node_->now();
  message.header.frame_id = "map";
  for (int axis = 0; axis < 2; ++axis) {
    message.head_position[static_cast<size_t>(axis)] = trajectory_data.start_pva(axis, 0);
    message.head_velocity[static_cast<size_t>(axis)] = trajectory_data.start_pva(axis, 1);
    message.head_acceleration[static_cast<size_t>(axis)] = trajectory_data.start_pva(axis, 2);
    message.tail_position[static_cast<size_t>(axis)] = trajectory_data.end_pva(axis, 0);
    message.tail_velocity[static_cast<size_t>(axis)] = trajectory_data.end_pva(axis, 1);
    message.tail_acceleration[static_cast<size_t>(axis)] = trajectory_data.end_pva(axis, 2);
  }
  message.head_position[2] = 0.0;
  message.head_velocity[2] = 0.0;
  message.head_acceleration[2] = 0.0;
  message.tail_position[2] = 0.0;
  message.tail_velocity[2] = 0.0;
  message.tail_acceleration[2] = 0.0;
  message.piece_n = trajectory_data.segment_count;

  message.points.reserve(static_cast<size_t>(trajectory_data.intermediate_positions.cols()));
  for (Eigen::Index index = 0; index < trajectory_data.intermediate_positions.cols(); ++index) {
    geometry_msgs::msg::Point point;
    point.x = trajectory_data.intermediate_positions(0, index);
    point.y = trajectory_data.intermediate_positions(1, index);
    point.z = 0.0;
    message.points.push_back(point);
  }
  message.times.reserve(static_cast<size_t>(trajectory_data.segment_durations.size()));
  for (Eigen::Index index = 0; index < trajectory_data.segment_durations.size(); ++index) {
    message.times.push_back(trajectory_data.segment_durations(index));
  }
  message.is_point_spacious.reserve(
    static_cast<size_t>(trajectory_data.waypoint_is_spacious.size()));
  for (Eigen::Index index = 0; index < trajectory_data.waypoint_is_spacious.size(); ++index) {
    message.is_point_spacious.push_back(
      trajectory_data.waypoint_is_spacious(index) == 0 ? 0 : 1);
  }

  for (const MincoTrajectoryData::CrossHoleInterval & interval :
    trajectory_data.cross_hole_intervals)
  {
    message.cross_hole_enter_times.push_back(interval.enter_time);
    message.cross_hole_exit_times.push_back(interval.exit_time);
    message.cross_hole_active_start_times.push_back(interval.active_start_time);
    message.cross_hole_active_end_times.push_back(interval.active_end_time);
    message.cross_hole_black_pixel_counts.push_back(interval.black_pixel_count);
  }
  traj_pub_->publish(message);
}

bool GlobalPlanner::fillTrajectoryOdom(
  const Trajectory<5, 2> & source_trajectory,
  double elapsed_time,
  nav_msgs::msg::Odometry & odom_message)
{
  if (source_trajectory.getPieceNum() <= 0 || !std::isfinite(elapsed_time)) {
    return false;
  }
  const double total_duration = source_trajectory.getTotalDuration();
  if (!std::isfinite(total_duration) || total_duration <= 0.0) {
    return false;
  }
  const double sample_time = std::clamp(elapsed_time, 0.0, total_duration);
  const Eigen::Vector2d position = source_trajectory.getPos(sample_time).head<2>();
  const Eigen::Vector2d velocity = source_trajectory.getVel(sample_time).head<2>();
  const Eigen::Vector2d acceleration = source_trajectory.getAcc(sample_time).head<2>();
  if (!position.allFinite() || !velocity.allFinite() || !acceleration.allFinite()) {
    return false;
  }

  const double yaw = trajectoryYawToBodyYaw(velocity, trajectory_odom_yaw_);
  trajectory_odom_yaw_ = yaw;
  double yaw_rate = 0.0;
  const double speed_squared = velocity.squaredNorm();
  if (speed_squared > 1.0e-8) {
    yaw_rate =
      (velocity.x() * acceleration.y() - velocity.y() * acceleration.x()) / speed_squared;
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  odom_message.header.stamp = node_->now();
  odom_message.header.frame_id = "map";
  odom_message.child_frame_id = "base_link";
  odom_message.pose.pose.position.x = position.x();
  odom_message.pose.pose.position.y = position.y();
  odom_message.pose.pose.position.z = 0.0;
  odom_message.pose.pose.orientation = tf2::toMsg(orientation);
  // 将 map 坐标速度旋转到 odom 消息的 base_link 速度语义。
  odom_message.twist.twist.linear.x =
    velocity.x() * std::cos(yaw) + velocity.y() * std::sin(yaw);
  odom_message.twist.twist.linear.y =
    -velocity.x() * std::sin(yaw) + velocity.y() * std::cos(yaw);
  odom_message.twist.twist.linear.z = 0.0;
  odom_message.twist.twist.angular.x = 0.0;
  odom_message.twist.twist.angular.y = 0.0;
  odom_message.twist.twist.angular.z = yaw_rate;
  return true;
}

void GlobalPlanner::publishTrajectoryOdom(double elapsed_time)
{
  if (config_.is_plan_from_ego_pose || !config_.publish_trajectory_odom ||
    !trajectory_odom_pub_)
  {
    return;
  }
  nav_msgs::msg::Odometry trajectory_odom;
  if (fillTrajectoryOdom(continuous_trajectory_, elapsed_time, trajectory_odom)) {
    trajectory_odom_pub_->publish(trajectory_odom);
  }
}

void GlobalPlanner::FSMCallBack_Timer()
{
  const double now_seconds = node_->now().seconds();
  const double elapsed_time = have_plan_ ? now_seconds - trajStamp_ : 0.0;
  double current_sample_time = 0.0;
  if (have_plan_) {
    current_sample_time = clampTrajectorySampleTime(
      now_seconds, trajStamp_, continuous_trajectory_.getTotalDuration());
  }

  if (!config_.is_plan_from_ego_pose && have_plan_) {
    // false 模式按当前 ROS 时间推进轨迹，并生成同一时刻的位置、速度和加速度。
    if (fillTrajectoryOdom(continuous_trajectory_, elapsed_time, odom_)) {
      odomInitialized_ = true;
      publishTrajectoryOdom(elapsed_time);
    }
  }
  if (odomInitialized_) {
    visualizer_.visualizeCurrentPosition(
      Eigen::Vector2d(odom_.pose.pose.position.x, odom_.pose.pose.position.y));
  }

  if (!config_valid_) {
    return;
  }
  const bool ready = mapInitialized_ && startGoal_.size() == 2 &&
    (!config_.is_plan_from_ego_pose || odomInitialized_);
  if (!ready) {
    if (map_changed_ && !have_plan_) {
      map_changed_ = false;
    }
    return;
  }







  // ============================================================
  // TODO(阶段④ 动态障碍与实时重规划)
  // 在这里扩展有限状态机、动态障碍预测、重规划滞回和规划失败处理。
  // 当前基线在新地图与剩余轨迹冲突时触发一次重新规划。
  // ============================================================








  
  const bool goal_event = goal_changed_;
  bool map_requires_replan = false;
  if (map_changed_ && have_plan_) {
    map_requires_replan = hasTrajectoryCollision(continuous_trajectory_, current_sample_time);
  }
  if (map_changed_) {
    map_changed_ = false;
  }
  if (!goal_event && !map_requires_replan) {
    return;
  }

  Eigen::Vector2d planning_start = startGoal_[0];
  if (!goal_event && map_requires_replan) {
    planning_start = Eigen::Vector2d(
      odom_.pose.pose.position.x, odom_.pose.pose.position.y);
  }
  const Eigen::Vector2d planning_goal = startGoal_[1];
  goal_changed_ = false;
  plan(planning_start, planning_goal);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("global_planning_node");
  auto planner = std::make_shared<GlobalPlanner>(Config(node), node);
  (void)planner;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
