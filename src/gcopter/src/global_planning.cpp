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

  dynamicSub_ = node_->create_subscription<plan_interfaces::msg::DynamicObstacleArray>(
    "/dynamic_obstacles", rclcpp::SensorDataQoS(),
    std::bind(&GlobalPlanner::dynamicObstacleCallBack, this, std::placeholders::_1));

  rclcpp::QoS trajectory_qos(rclcpp::KeepLast(20));
  trajectory_qos.reliable();
  trajectory_qos.durability_volatile();
  traj_pub_ = node_->create_publisher<plan_interfaces::msg::MincoTrajectory>(
    "/minco_trajectory", trajectory_qos);

  rclcpp::QoS planning_result_qos(rclcpp::KeepLast(1));
  planning_result_qos.reliable();
  planning_result_qos.transient_local();
  planning_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "/planner/path", planning_result_qos);
  planning_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/planner/status", planning_result_qos);

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
    config_.is_plan_from_ego_pose ? "true" : "false", config_.nominal_velocity);
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
  // Derive hard footprint layers, clearance, and configured semantic regions.
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
  candidate.raw_occupancy.resize(expected_size);
  std::transform(msg->data.begin(), msg->data.end(), candidate.raw_occupancy.begin(), [](int8_t value) {
    return static_cast<uint8_t>(value == 0 ? 0 : 100);
  });
  candidate.semantic.assign(expected_size, static_cast<uint8_t>(MapSemantic::NORMAL));
  for (size_t region = 0; region + 3 < config_.cross_hole_regions.size(); region += 4) {
    const double min_x = std::min(config_.cross_hole_regions[region], config_.cross_hole_regions[region + 1]);
    const double max_x = std::max(config_.cross_hole_regions[region], config_.cross_hole_regions[region + 1]);
    const double min_y = std::min(config_.cross_hole_regions[region + 2], config_.cross_hole_regions[region + 3]);
    const double max_y = std::max(config_.cross_hole_regions[region + 2], config_.cross_hole_regions[region + 3]);
    for (int y = 0; y < candidate.height; ++y) {
      for (int x = 0; x < candidate.width; ++x) {
        const Eigen::Vector2d point = candidate.gridToWorld({x, y});
        if (point.x() >= min_x && point.x() <= max_x && point.y() >= min_y && point.y() <= max_y) {
          candidate.semantic[static_cast<size_t>(candidate.index({x, y}))] =
            static_cast<uint8_t>(MapSemantic::CROSS_HOLE);
        }
      }
    }
  }
  if (!candidate.buildDerivedLayers(
      config_.robot_radius_normal + config_.static_safety_margin,
      config_.robot_radius_compact + config_.static_safety_margin))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to derive inflated map layers");
    return;
  }
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
    pending_goal_retry_ = true;
    pending_goal_retry_count_ = 0;
    visualizer_.visualizeStartGoal(startGoal_[0], startGoal_[1]);
    return;
  }

  if (startGoal_.size() >= 2) {
    startGoal_.clear();
  }
  startGoal_.push_back(received);
  if (startGoal_.size() == 2) {
    goal_changed_ = true;
    pending_goal_retry_ = true;
    pending_goal_retry_count_ = 0;
    visualizer_.visualizeStartGoal(startGoal_[0], startGoal_[1]);
  } else {
    RCLCPP_INFO(
      node_->get_logger(), "Stored false-mode start point (%.3f, %.3f); waiting for goal",
      received.x(), received.y());
  }
}

void GlobalPlanner::dynamicObstacleCallBack(
  const plan_interfaces::msg::DynamicObstacleArray::SharedPtr msg)
{
  if (!msg || (!msg->header.frame_id.empty() && msg->header.frame_id != "map")) {return;}
  for (const auto & obstacle : msg->obstacles) {
    if (!std::isfinite(obstacle.position.x) || !std::isfinite(obstacle.position.y) ||
      !std::isfinite(obstacle.velocity.x) || !std::isfinite(obstacle.velocity.y) ||
      !std::isfinite(obstacle.size.x) || !std::isfinite(obstacle.size.y) ||
      !std::isfinite(obstacle.radius) || !std::isfinite(obstacle.valid_for) ||
      obstacle.size.x < 0.0 || obstacle.size.y < 0.0 || obstacle.radius < 0.0 ||
      obstacle.valid_for < 0.0)
    {
      RCLCPP_WARN(node_->get_logger(), "Rejected invalid dynamic obstacle array");
      return;
    }
  }
  dynamic_obstacles_ = *msg;
  dynamic_obstacles_changed_ = true;
  visualizer_.visualizeDynamicObstacles(dynamic_obstacles_);
}

bool GlobalPlanner::FrontSearch(
  const Eigen::Vector2d & start,
  const Eigen::Vector2d & goal,
  std::vector<Eigen::Vector2d> & route,
  std::string * failure_reason)
{
  return path_search_.search(
    grid_map_, start, goal, config_.is_omni, config_.search, route, failure_reason);
}

bool GlobalPlanner::plan(
  const Eigen::Vector2d & start, const Eigen::Vector2d & goal,
  bool preserve_current_state)
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
  std::vector<ConvexCorridor2D> corridors;
  const auto total_begin = std::chrono::steady_clock::now();
  front_end_path_available_ = false;

  const auto search_begin = std::chrono::steady_clock::now();
  if (!FrontSearch(start, goal, route_raw, &reason)) {
    publishPlanningPath({});
    const bool waiting_for_dynamic_clearance =
      !dynamic_obstacles_.obstacles.empty() &&
      (reason == "start_occupied" || reason == "goal_occupied" ||
      reason == "search_exhausted");
    if (waiting_for_dynamic_clearance) {
      publishPlanningStatus(
        "WAITING_DYNAMIC_CLEARANCE", false,
        "前端暂不可达（" + reason + "），保留原起终点并等待动态障碍物让开后自动重试");
      RCLCPP_WARN(
        node_->get_logger(), "前端暂不可达：%s；将自动重试原起终点", reason.c_str());
    } else {
      publishPlanningStatus("FAILED_NO_PATH", false, "前端 A* 失败：" + reason);
      RCLCPP_WARN(node_->get_logger(), "规划失败：前端 A*：%s", reason.c_str());
    }
    if (!preserve_current_state) {
      have_plan_ = false;
      continuous_trajectory_.clear();
      visualizer_.clearTrajectory();
    }
    return false;
  }
  front_end_path_available_ = true;
  const double search_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - search_begin).count();
  // 前端一成功就立即发布。后端即使耗时或失败，用户和上层仍能取得一条
  // 经过硬膨胀地图验证的可行离散路径。
  visualizer_.visualizeStartGoal(start, goal);
  visualizer_.visualizeRoute(route_raw);
  publishPlanningPath(route_raw);
  publishPlanningStatus("FRONTEND_PATH_READY", false, "前端路径已发布，后端继续处理中");
  if (!SamplePath(grid_map_, route_raw, route_sampled)) {
    route_sampled = route_raw;
    RCLCPP_WARN(node_->get_logger(), "路径稀疏化失败，保留完整前端 A* 路径继续处理");
  }
  visualizer_.visualizeRoute(route_sampled);
  publishPlanningPath(route_sampled);
  const auto corridor_begin = std::chrono::steady_clock::now();
  if (!GenerateCorridor(grid_map_, route_sampled, config_.corridor, corridors, reason)) {
    // Conservative recovery: restore the front-end grid points and retry without
    // changing the selected topological route or rerunning A*.
    route_sampled = route_raw;
    visualizer_.visualizeRoute(route_sampled);
    publishPlanningPath(route_sampled);
    if (!GenerateCorridor(grid_map_, route_sampled, config_.corridor, corridors, reason)) {
      publishPlanningStatus(
        "FRONTEND_PATH_ONLY", false, "corridor 生成失败：" + reason);
      RCLCPP_WARN(
        node_->get_logger(),
        "已降级为仅发布前端路径：corridor 生成失败：%s", reason.c_str());
      if (!preserve_current_state) {
        have_plan_ = false;
        continuous_trajectory_.clear();
        visualizer_.clearTrajectory();
      }
      return false;
    }
  }
  const double corridor_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - corridor_begin).count();
  publishPlanningStatus(
    "CORRIDOR_PATH_READY", false, "前端路径与安全 corridor 已生成，轨迹后端继续处理中");

  Eigen::Matrix<double, 2, 3> start_pva = Eigen::Matrix<double, 2, 3>::Zero();
  if (preserve_current_state && have_plan_) {
    const double sample_time = clampTrajectorySampleTime(
      node_->now().seconds(), trajStamp_, continuous_trajectory_.getTotalDuration());
    start_pva.col(0) = continuous_trajectory_.getPos(sample_time).head<2>();
    start_pva.col(1) = continuous_trajectory_.getVel(sample_time).head<2>();
    start_pva.col(2) = continuous_trajectory_.getAcc(sample_time).head<2>();
  }
  start_pva.col(0) = start;
  if (!GenerateTrajectory(route_sampled, corridors, config_.nominal_velocity,
      config_.min_segment_duration, start_pva, initial_candidate))
  {
    publishPlanningStatus(
      "CORRIDOR_PATH_ONLY", false, "初始 MINCO 数据生成失败，保留前端路径与 corridor");
    RCLCPP_WARN(node_->get_logger(), "已降级为前端路径 + corridor：初始 MINCO 数据生成失败");
    if (!preserve_current_state) {
      have_plan_ = false;
      continuous_trajectory_.clear();
      visualizer_.clearTrajectory();
    }
    return false;
  }
  updateWaypointSpaciousFlags(initial_candidate);
  const auto minco_begin = std::chrono::steady_clock::now();
  MincoTrajectoryData safe_initial_candidate;
  Trajectory<5, 2> safe_initial_trajectory;
  std::string initial_reason;
  bool safe_initial_available = PrepareSafeInitialTrajectory(
    grid_map_, initial_candidate, config_.optimization, safe_initial_candidate, &initial_reason);
  if (safe_initial_available) {
    safe_initial_available = buildContinuousTrajectory(
      safe_initial_candidate, safe_initial_trajectory) &&
      !hasTrajectoryCollision(safe_initial_trajectory, 0.0);
    if (!safe_initial_available) {
      initial_reason = "保底初始轨迹的连续解算或硬碰撞检查失败";
    }
  }

  const bool optimization_succeeded = OptimizeTrajectory(
    grid_map_, initial_candidate, config_.optimization, output_candidate, &reason);
  const double minco_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - minco_begin).count();
  std::string result_level = "OPTIMIZED_MINCO";
  std::string result_detail = "优化后的 MINCO 轨迹已通过全部检查";
  bool optimized_usable = optimization_succeeded;
  if (optimized_usable && !ValidateTrajectoryData(output_candidate, reason)) {
    optimized_usable = false;
    reason = "优化结果数据无效：" + reason;
  }
  if (optimized_usable && !buildContinuousTrajectory(output_candidate, trajectory_candidate)) {
    optimized_usable = false;
    reason = "优化结果无法构造连续轨迹";
  }
  if (optimized_usable && hasTrajectoryCollision(trajectory_candidate, 0.0)) {
    optimized_usable = false;
    reason = "优化后的连续轨迹未通过硬碰撞检查";
  }

  if (!optimized_usable && safe_initial_available) {
    output_candidate = safe_initial_candidate;
    trajectory_candidate = safe_initial_trajectory;
    result_level = "SAFE_INITIAL_MINCO";
    result_detail = "优化阶段失败，已发布满足硬安全与运动包络的未优化 MINCO 轨迹：" + reason;
    RCLCPP_WARN(node_->get_logger(), "%s", result_detail.c_str());
  } else if (!optimized_usable) {
    const std::string detail = "MINCO 后端失败（" + reason +
      "），保底初始轨迹也不可用（" + initial_reason + "）";
    publishPlanningStatus("CORRIDOR_PATH_ONLY", false, detail);
    RCLCPP_WARN(node_->get_logger(), "已降级为前端路径 + corridor：%s", detail.c_str());
    if (!preserve_current_state) {
      have_plan_ = false;
      continuous_trajectory_.clear();
      visualizer_.clearTrajectory();
    }
    return false;
  }

  extractCrossHoleIntervals(trajectory_candidate, output_candidate);
  updateWaypointSpaciousFlags(output_candidate);

  publishMincoTrajectory(output_candidate);
  initial_trajectory_data_ = initial_candidate;
  output_trajectory_data_ = output_candidate;
  continuous_trajectory_ = trajectory_candidate;
  have_plan_ = true;
  trajStamp_ = node_->now().seconds();
  visualizer_.visualizeStartGoal(start, goal);
  visualizer_.visualizeRoute(route_sampled);
  visualizer_.visualizeTrajectory(continuous_trajectory_);
  publishPlanningPath(route_sampled);
  publishPlanningStatus(result_level, true, result_detail);
  double path_length = 0.0;
  for (size_t i = 1; i < route_raw.size(); ++i) {path_length += (route_raw[i] - route_raw[i - 1]).norm();}
  double max_velocity = 0.0;
  double max_acceleration = 0.0;
  double minimum_clearance = std::numeric_limits<double>::infinity();
  const double duration = continuous_trajectory_.getTotalDuration();
  for (double time = 0.0; time <= duration; time += 0.02) {
    const Eigen::Vector2d position = continuous_trajectory_.getPos(std::min(time, duration)).head<2>();
    max_velocity = std::max(max_velocity, continuous_trajectory_.getVel(std::min(time, duration)).norm());
    max_acceleration = std::max(max_acceleration, continuous_trajectory_.getAcc(std::min(time, duration)).norm());
    minimum_clearance = std::min(minimum_clearance,
      static_cast<double>(grid_map_.clearanceAt(grid_map_.worldToGrid(position))));
  }
  const double total_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - total_begin).count();
  RCLCPP_INFO(
    node_->get_logger(), "规划结果已发布：level=%s route_points=%zu segments=%d duration=%.3f s",
    result_level.c_str(), route_sampled.size(), output_candidate.segment_count,
    continuous_trajectory_.getTotalDuration());
  RCLCPP_INFO(node_->get_logger(),
    "metrics search=%.2fms corridor=%.2fms minco=%.2fms total=%.2fms path=%.3fm clearance=%.3fm vmax=%.3fm/s amax=%.3fm/s2",
    search_ms, corridor_ms, minco_ms, total_ms, path_length, minimum_clearance,
    max_velocity, max_acceleration);
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
  // Build the continuous quintic trajectory with the existing MINCO_S3NU solver.
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
  double start_time,
  double horizon) const
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
  const double end = std::min(total_duration, begin + std::max(0.0, horizon));
  const double remaining = end - begin;
  const double requested_samples = std::ceil(remaining / kCheckDt) + 1.0;
  if (!std::isfinite(requested_samples) || requested_samples > kMaxCollisionSamples) {
    return true;
  }

  const size_t sample_count = static_cast<size_t>(requested_samples);
  for (size_t index = 0; index < sample_count; ++index) {
    const double time = std::min(end, begin + static_cast<double>(index) * kCheckDt);
    const Eigen::Vector2d position = trajectory.getPos(time).head<2>();
    if (!position.allFinite() || grid_map_.isOccupied(grid_map_.worldToGrid(position))) {
      return true;
    }
  }
  const Eigen::Vector2d final_position = trajectory.getPos(end).head<2>();
  return !final_position.allFinite() ||
    grid_map_.isOccupied(grid_map_.worldToGrid(final_position));
}

bool GlobalPlanner::hasDynamicObstacleConflict(
  const Trajectory<5, 2> & trajectory,
  double start_time,
  double now_seconds) const
{
  if (trajectory.getPieceNum() <= 0 || !std::isfinite(start_time) ||
    !std::isfinite(now_seconds))
  {return true;}
  const double message_stamp = rclcpp::Time(dynamic_obstacles_.header.stamp).seconds();
  const double age = std::max(0.0, now_seconds - message_stamp);
  const double duration = trajectory.getTotalDuration();
  const double end = std::min(duration, start_time + config_.prediction_horizon);
  for (const auto & obstacle : dynamic_obstacles_.obstacles) {
    const double validity = obstacle.valid_for > 0.0 ?
      std::min(config_.dynamic_obstacle_timeout, obstacle.valid_for) :
      config_.dynamic_obstacle_timeout;
    if (age > validity) {continue;}
    const double obstacle_radius = obstacle.shape ==
      plan_interfaces::msg::DynamicObstacle::SHAPE_BOX ?
      0.5 * std::hypot(obstacle.size.x, obstacle.size.y) : obstacle.radius;
    const double safety_radius = obstacle_radius + config_.robot_radius_normal +
      config_.static_safety_margin + config_.dynamic_safety_margin;
    for (double trajectory_time = start_time; trajectory_time <= end; trajectory_time += 0.02) {
      const double future = age + trajectory_time - start_time;
      const Eigen::Vector2d predicted(
        obstacle.position.x + obstacle.velocity.x * future,
        obstacle.position.y + obstacle.velocity.y * future);
      const Eigen::Vector2d robot = trajectory.getPos(trajectory_time).head<2>();
      if (!predicted.allFinite() || !robot.allFinite() ||
        (predicted - robot).squaredNorm() <= safety_radius * safety_radius)
      {return true;}
    }
  }
  return false;
}

void GlobalPlanner::updateWaypointSpaciousFlags(MincoTrajectoryData & trajectory_data) const
{









  // ============================================================
  // Mark intermediate points using the cross-hole semantic layer.
  // 在这里设计考虑过洞问题，当一段轨迹真正要过洞时，合理标记轨迹使其
  // 当前基线只将连接点标记为宽阔区域。
  // ============================================================












  
  trajectory_data.waypoint_is_spacious.resize(trajectory_data.intermediate_positions.cols());
  for (Eigen::Index index = 0; index < trajectory_data.intermediate_positions.cols(); ++index) {
    const Eigen::Vector2d point = trajectory_data.intermediate_positions.col(index);
    trajectory_data.waypoint_is_spacious(index) =
      grid_map_.semanticAt(grid_map_.worldToGrid(point)) == MapSemantic::CROSS_HOLE ? 0 : 1;
  }
}

void GlobalPlanner::extractCrossHoleIntervals(
  const Trajectory<5, 2> & trajectory,
  MincoTrajectoryData & trajectory_data) const
{
  trajectory_data.cross_hole_intervals.clear();
  if (!grid_map_.valid() || trajectory.getPieceNum() <= 0) {return;}
  const double duration = trajectory.getTotalDuration();
  const double dt = std::min(0.02, std::max(0.002, grid_map_.resolution /
    std::max(config_.optimization.max_velocity, 1.0e-3) * 0.5));
  bool inside = false;
  double enter_time = 0.0;
  for (double time = 0.0; time <= duration + 0.5 * dt; time += dt) {
    const double sample_time = std::min(time, duration);
    const Eigen::Vector2d position = trajectory.getPos(sample_time).head<2>();
    const bool current_inside = grid_map_.semanticAt(grid_map_.worldToGrid(position)) ==
      MapSemantic::CROSS_HOLE;
    if (current_inside && !inside) {
      enter_time = sample_time;
    } else if (!current_inside && inside) {
      MincoTrajectoryData::CrossHoleInterval interval;
      interval.enter_time = enter_time;
      interval.exit_time = sample_time;
      interval.active_start_time = std::max(0.0,
        enter_time - config_.cross_hole_prepare_time - config_.cross_hole_time_margin);
      interval.active_end_time = std::min(duration,
        sample_time + config_.cross_hole_recovery_time + config_.cross_hole_time_margin);
      interval.black_pixel_count = 0;
      if (interval.exit_time > interval.enter_time &&
        interval.active_end_time > interval.active_start_time)
      {
        trajectory_data.cross_hole_intervals.push_back(interval);
      }
    }
    inside = current_inside;
  }
  if (inside) {
    MincoTrajectoryData::CrossHoleInterval interval;
    interval.enter_time = enter_time;
    interval.exit_time = duration;
    interval.active_start_time = std::max(0.0,
      enter_time - config_.cross_hole_prepare_time - config_.cross_hole_time_margin);
    interval.active_end_time = duration;
    if (interval.exit_time > interval.enter_time &&
      interval.active_end_time > interval.active_start_time)
    {
      trajectory_data.cross_hole_intervals.push_back(interval);
    }
  }
}

void GlobalPlanner::publishPlanningPath(const std::vector<Eigen::Vector2d> & route)
{
  nav_msgs::msg::Path message;
  message.header.stamp = node_->now();
  message.header.frame_id = "map";
  message.poses.reserve(route.size());
  for (const Eigen::Vector2d & point : route) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = message.header;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.orientation.w = 1.0;
    message.poses.push_back(pose);
  }
  planning_path_pub_->publish(message);
}

void GlobalPlanner::publishPlanningStatus(
  const std::string & level, bool executable, const std::string & detail)
{
  std_msgs::msg::String message;
  message.data = "level=" + level + "; executable=" +
    (executable ? std::string("true") : std::string("false")) + "; detail=" + detail;
  planning_status_pub_->publish(message);
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
  // Replan only when a changed map or predicted obstacle conflicts with the horizon.
  // 在这里扩展有限状态机、动态障碍预测、重规划滞回和规划失败处理。
  // 当前基线在新地图与剩余轨迹冲突时触发一次重新规划。
  // ============================================================








  
  const bool goal_event = goal_changed_;
  const bool pending_retry_due = !goal_event && pending_goal_retry_ && !have_plan_ &&
    map_changed_ && pending_goal_retry_count_ < config_.planning_retry_limit &&
    (now_seconds - last_plan_attempt_time_) * 1000.0 >= config_.planning_retry_interval_ms;
  bool map_requires_replan = false;
  if (map_changed_ && have_plan_) {
    const bool cooldown_elapsed =
      (now_seconds - last_replan_time_) * 1000.0 >= config_.replan_cooldown_ms;
    map_requires_replan = cooldown_elapsed && hasTrajectoryCollision(
      continuous_trajectory_, current_sample_time, config_.prediction_horizon);
  }
  if (map_changed_) {
    map_changed_ = false;
  }
  if (dynamic_obstacles_changed_ && have_plan_) {
    const bool cooldown_elapsed =
      (now_seconds - last_replan_time_) * 1000.0 >= config_.replan_cooldown_ms;
    map_requires_replan = map_requires_replan || (cooldown_elapsed &&
      hasDynamicObstacleConflict(continuous_trajectory_, current_sample_time, now_seconds));
  }
  dynamic_obstacles_changed_ = false;
  if (!goal_event && !map_requires_replan && !pending_retry_due) {
    return;
  }

  Eigen::Vector2d planning_start = startGoal_[0];
  if (!goal_event && map_requires_replan) {
    planning_start = Eigen::Vector2d(
      odom_.pose.pose.position.x, odom_.pose.pose.position.y);
  }
  const Eigen::Vector2d planning_goal = startGoal_[1];
  goal_changed_ = false;
  last_plan_attempt_time_ = now_seconds;
  if (pending_retry_due) {
    RCLCPP_INFO(
      node_->get_logger(), "动态净空自动重试 %d/%d",
      pending_goal_retry_count_ + 1, config_.planning_retry_limit);
  }
  if (plan(planning_start, planning_goal, !goal_event && map_requires_replan)) {
    last_replan_time_ = now_seconds;
    pending_goal_retry_ = false;
    pending_goal_retry_count_ = 0;
  } else if ((goal_event || pending_retry_due) && !front_end_path_available_) {
    ++pending_goal_retry_count_;
    if (pending_goal_retry_count_ >= config_.planning_retry_limit) {
      pending_goal_retry_ = false;
      publishPlanningStatus(
        "FAILED_NO_PATH_RETRY_EXHAUSTED", false,
        "动态净空自动重试次数已用完，请重新选择起终点");
      RCLCPP_WARN(
        node_->get_logger(), "动态净空自动重试已达到上限 %d", config_.planning_retry_limit);
    }
  } else if (goal_event || pending_retry_due) {
    // 已经发布了前端保底路径，不再因后端降级反复执行整条规划流水线。
    pending_goal_retry_ = false;
    pending_goal_retry_count_ = 0;
  } else if (map_requires_replan &&
    (hasTrajectoryCollision(continuous_trajectory_, current_sample_time, config_.prediction_horizon) ||
    hasDynamicObstacleConflict(continuous_trajectory_, current_sample_time, now_seconds)))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Replanning failed and the retained trajectory is unsafe; controller must stop");
    have_plan_ = false;
    continuous_trajectory_.clear();
    visualizer_.clearTrajectory();
    publishPlanningStatus(
      "PATH_ONLY_STOP_REQUIRED", false,
      "动态重规划只得到离散路径，旧轨迹已不安全，控制器必须停车");
  }
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
