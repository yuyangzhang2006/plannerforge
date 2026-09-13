#pragma once

#include "gcopter/corridor_generator.hpp"
#include "gcopter/grid_map_2d.hpp"
#include "gcopter/local_replan.hpp"
#include "gcopter/path_search.hpp"
#include "gcopter/traj_representation.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/trajectory_generation.hpp"
#include "gcopter/visualizer.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <plan_interfaces/msg/minco_trajectory.hpp>
#include <plan_interfaces/msg/dynamic_obstacle_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

struct Config
{
  std::string mapTopic;
  std::string targetTopic;
  std::string odomTopic;
  bool is_omni = true;
  bool is_plan_from_ego_pose = false;
  bool publish_trajectory_odom = true;
  double nominal_velocity = 0.5;
  double robot_radius_normal = 0.30;
  double robot_radius_compact = 0.15;
  double static_safety_margin = 0.05;
  double min_segment_duration = 0.15;
  double cross_hole_prepare_time = 0.5;
  double cross_hole_recovery_time = 0.5;
  double cross_hole_time_margin = 0.1;
  double prediction_horizon = 2.0;
  double dynamic_safety_margin = 0.15;
  double dynamic_obstacle_timeout = 0.5;
  double replan_cooldown_ms = 500.0;
  double planning_retry_interval_ms = 1000.0;
  int planning_retry_limit = 20;
  LocalReplanOptions local_replan;
  std::vector<double> cross_hole_regions;  // xmin, xmax, ymin, ymax tuples
  PathSearch::Options search;
  CorridorOptions corridor;
  OptimizationOptions optimization;

  explicit Config(const rclcpp::Node::SharedPtr & node)
  {
    node->declare_parameter("gcopter.MapTopic", std::string("/grid_map"));
    node->declare_parameter("gcopter.TargetTopic", std::string("/goal_pose"));
    node->declare_parameter("gcopter.OdomTopic", std::string("/Odometry_to_base_link"));
    node->declare_parameter("gcopter.is_omni", true);
    node->declare_parameter("gcopter.is_plan_from_ego_pose", false);
    node->declare_parameter("gcopter.PublishTrajectoryOdom", true);
    node->declare_parameter("gcopter.MaxVelMag", 0.5);
    node->declare_parameter("gcopter.NominalVel", 0.5);
    node->declare_parameter("gcopter.RobotRadiusNormal", 0.30);
    node->declare_parameter("gcopter.RobotRadiusCompact", 0.15);
    node->declare_parameter("gcopter.StaticSafetyMargin", 0.05);
    node->declare_parameter("gcopter.TurnCostWeight", 0.0);
    node->declare_parameter("gcopter.SafetyCostWeight", 0.0);
    node->declare_parameter("gcopter.SafetyCostDistance", 0.5);
    node->declare_parameter("gcopter.SpecialRegionCostWeight", 0.0);
    node->declare_parameter("gcopter.CrossHoleRegions", std::vector<double>{});
    node->declare_parameter("gcopter.CorridorObstacleSearchRadius", 1.5);
    node->declare_parameter("gcopter.CorridorFiriIterations", 4);
    node->declare_parameter("gcopter.CorridorMinOverlap", 0.08);
    node->declare_parameter("gcopter.CorridorMergeTolerance", 1.0e-4);
    node->declare_parameter("gcopter.MaxPlanVel", 1.0);
    node->declare_parameter("gcopter.MaxPlanAcc", 1.5);
    node->declare_parameter("gcopter.MinSegmentDuration", 0.15);
    node->declare_parameter("gcopter.WeightTime", 1.0);
    node->declare_parameter("gcopter.WeightVelPenalty", 10.0);
    node->declare_parameter("gcopter.WeightAccPenalty", 10.0);
    node->declare_parameter("gcopter.OptimizationMaxIterations", 30);
    node->declare_parameter("gcopter.OptimizationTimeBudgetMs", 30.0);
    node->declare_parameter("gcopter.PredictionHorizonSec", 2.0);
    node->declare_parameter("gcopter.DynamicSafetyMargin", 0.15);
    node->declare_parameter("gcopter.DynamicObstacleTimeoutSec", 0.5);
    node->declare_parameter("gcopter.ReplanCooldownMs", 500.0);
    node->declare_parameter("gcopter.PlanningRetryIntervalMs", 1000.0);
    node->declare_parameter("gcopter.PlanningRetryLimit", 20);
    node->declare_parameter("gcopter.LocalReplanMinLookaheadDistance", 1.5);
    node->declare_parameter("gcopter.LocalReplanMaxLookaheadDistance", 6.0);
    node->declare_parameter("gcopter.LocalReplanMaxPathDistance", 9.0);
    node->declare_parameter("gcopter.LocalReplanMaxExtraDistance", 3.0);
    node->declare_parameter("gcopter.CrossHoleShapeTime", 0.5);
    node->declare_parameter("gcopter.CrossHoleRecoveryTime", 0.5);
    node->declare_parameter("gcopter.CrossHoleTimeMargin", 0.1);

    mapTopic = node->get_parameter("gcopter.MapTopic").as_string();
    targetTopic = node->get_parameter("gcopter.TargetTopic").as_string();
    odomTopic = node->get_parameter("gcopter.OdomTopic").as_string();
    is_omni = node->get_parameter("gcopter.is_omni").as_bool();
    is_plan_from_ego_pose = node->get_parameter("gcopter.is_plan_from_ego_pose").as_bool();
    publish_trajectory_odom = node->get_parameter("gcopter.PublishTrajectoryOdom").as_bool();
    nominal_velocity = node->get_parameter("gcopter.NominalVel").as_double();
    const double legacy_velocity = node->get_parameter("gcopter.MaxVelMag").as_double();
    if (nominal_velocity == 0.5 && legacy_velocity != 0.5) {nominal_velocity = legacy_velocity;}
    robot_radius_normal = node->get_parameter("gcopter.RobotRadiusNormal").as_double();
    robot_radius_compact = node->get_parameter("gcopter.RobotRadiusCompact").as_double();
    static_safety_margin = node->get_parameter("gcopter.StaticSafetyMargin").as_double();
    search.turn_cost_weight = node->get_parameter("gcopter.TurnCostWeight").as_double();
    search.safety_cost_weight = node->get_parameter("gcopter.SafetyCostWeight").as_double();
    search.safety_cost_distance = node->get_parameter("gcopter.SafetyCostDistance").as_double();
    search.special_region_cost_weight =
      node->get_parameter("gcopter.SpecialRegionCostWeight").as_double();
    cross_hole_regions = node->get_parameter("gcopter.CrossHoleRegions").as_double_array();
    corridor.obstacle_search_radius =
      node->get_parameter("gcopter.CorridorObstacleSearchRadius").as_double();
    corridor.firi_iterations =
      static_cast<int>(node->get_parameter("gcopter.CorridorFiriIterations").as_int());
    corridor.minimum_overlap = node->get_parameter("gcopter.CorridorMinOverlap").as_double();
    corridor.merge_tolerance = node->get_parameter("gcopter.CorridorMergeTolerance").as_double();
    optimization.max_velocity = node->get_parameter("gcopter.MaxPlanVel").as_double();
    optimization.max_acceleration = node->get_parameter("gcopter.MaxPlanAcc").as_double();
    min_segment_duration = node->get_parameter("gcopter.MinSegmentDuration").as_double();
    optimization.minimum_segment_duration = min_segment_duration;
    optimization.weight_time = node->get_parameter("gcopter.WeightTime").as_double();
    optimization.weight_velocity = node->get_parameter("gcopter.WeightVelPenalty").as_double();
    optimization.weight_acceleration = node->get_parameter("gcopter.WeightAccPenalty").as_double();
    optimization.maximum_iterations =
      static_cast<int>(node->get_parameter("gcopter.OptimizationMaxIterations").as_int());
    optimization.time_budget_ms = node->get_parameter("gcopter.OptimizationTimeBudgetMs").as_double();
    prediction_horizon = node->get_parameter("gcopter.PredictionHorizonSec").as_double();
    dynamic_safety_margin = node->get_parameter("gcopter.DynamicSafetyMargin").as_double();
    dynamic_obstacle_timeout = node->get_parameter("gcopter.DynamicObstacleTimeoutSec").as_double();
    replan_cooldown_ms = node->get_parameter("gcopter.ReplanCooldownMs").as_double();
    planning_retry_interval_ms =
      node->get_parameter("gcopter.PlanningRetryIntervalMs").as_double();
    planning_retry_limit =
      static_cast<int>(node->get_parameter("gcopter.PlanningRetryLimit").as_int());
    local_replan.minimum_lookahead_distance =
      node->get_parameter("gcopter.LocalReplanMinLookaheadDistance").as_double();
    local_replan.maximum_lookahead_distance =
      node->get_parameter("gcopter.LocalReplanMaxLookaheadDistance").as_double();
    local_replan.maximum_local_path_distance =
      node->get_parameter("gcopter.LocalReplanMaxPathDistance").as_double();
    local_replan.maximum_extra_distance =
      node->get_parameter("gcopter.LocalReplanMaxExtraDistance").as_double();
    cross_hole_prepare_time = node->get_parameter("gcopter.CrossHoleShapeTime").as_double();
    cross_hole_recovery_time = node->get_parameter("gcopter.CrossHoleRecoveryTime").as_double();
    cross_hole_time_margin = node->get_parameter("gcopter.CrossHoleTimeMargin").as_double();
  }

  bool valid() const
  {
    return !mapTopic.empty() && !targetTopic.empty() && !odomTopic.empty() &&
      std::isfinite(nominal_velocity) && nominal_velocity > 0.0 &&
      std::isfinite(robot_radius_normal) && std::isfinite(robot_radius_compact) &&
      std::isfinite(static_safety_margin) && robot_radius_normal >= robot_radius_compact &&
      robot_radius_compact >= 0.0 && static_safety_margin >= 0.0 &&
      cross_hole_regions.size() % 4 == 0 && optimization.max_velocity > 0.0 &&
      optimization.max_acceleration > 0.0 && min_segment_duration > 0.0 &&
      std::isfinite(planning_retry_interval_ms) && planning_retry_interval_ms > 0.0 &&
      planning_retry_limit >= 0 &&
      std::isfinite(local_replan.minimum_lookahead_distance) &&
      std::isfinite(local_replan.maximum_lookahead_distance) &&
      std::isfinite(local_replan.maximum_local_path_distance) &&
      std::isfinite(local_replan.maximum_extra_distance) &&
      local_replan.minimum_lookahead_distance >= 0.0 &&
      local_replan.maximum_lookahead_distance >= local_replan.minimum_lookahead_distance &&
      local_replan.maximum_local_path_distance > 0.0 &&
      local_replan.maximum_extra_distance >= 0.0;
  }
};

inline double clampTrajectorySampleTime(double now, double stamp, double duration)
{
  if (!std::isfinite(now) || !std::isfinite(stamp) || !std::isfinite(duration) || duration <= 0.0) {
    return 0.0;
  }
  return std::clamp(now - stamp, 0.0, duration);
}

class GlobalPlanner
{
public:
  GlobalPlanner(const Config & config, const rclcpp::Node::SharedPtr & node);
  void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
  void mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void dynamicObstacleCallBack(
    const plan_interfaces::msg::DynamicObstacleArray::SharedPtr msg);
  void FSMCallBack_Timer();
  bool FrontSearch(const Eigen::Vector2d & start, const Eigen::Vector2d & goal,
    std::vector<Eigen::Vector2d> & route, std::string * failure_reason = nullptr);
  bool plan(const Eigen::Vector2d & start, const Eigen::Vector2d & goal,
    bool preserve_current_state = false,
    const std::vector<Eigen::Vector2d> * prescribed_route = nullptr);
  bool buildContinuousTrajectory(const MincoTrajectoryData & data,
    Trajectory<5, 2> & trajectory) const;
  bool hasTrajectoryCollision(const Trajectory<5, 2> & trajectory, double start_time,
    double horizon = std::numeric_limits<double>::infinity()) const;
  bool hasDynamicObstacleConflict(
    const Trajectory<5, 2> & trajectory, double start_time, double now_seconds) const;
  void updateWaypointSpaciousFlags(MincoTrajectoryData & data) const;
  void extractCrossHoleIntervals(const Trajectory<5, 2> & trajectory,
    MincoTrajectoryData & data) const;
  void publishPlanningPath(const std::vector<Eigen::Vector2d> & route);
  void publishPlanningStatus(
    const std::string & level, bool executable, const std::string & detail);
  void publishMincoTrajectory(const MincoTrajectoryData & data);
  bool fillTrajectoryOdom(const Trajectory<5, 2> & trajectory, double elapsed_time,
    nav_msgs::msg::Odometry & odom_message);
  void publishTrajectoryOdom(double elapsed_time);

private:
  Config config_;
  rclcpp::Node::SharedPtr node_;
  Visualizer visualizer_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  rclcpp::Subscription<plan_interfaces::msg::DynamicObstacleArray>::SharedPtr dynamicSub_;
  rclcpp::Publisher<plan_interfaces::msg::MincoTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planning_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr trajectory_odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  GridMap2D grid_map_;
  PathSearch path_search_;
  MincoTrajectoryData initial_trajectory_data_;
  MincoTrajectoryData output_trajectory_data_;
  Trajectory<5, 2> continuous_trajectory_;
  std::vector<Eigen::Vector2d> active_route_;
  std::vector<Eigen::Vector2d> startGoal_;
  nav_msgs::msg::Odometry odom_;
  plan_interfaces::msg::DynamicObstacleArray dynamic_obstacles_;
  bool config_valid_ = false;
  bool mapInitialized_ = false;
  bool odomInitialized_ = false;
  bool goal_changed_ = false;
  bool map_changed_ = false;
  bool dynamic_obstacles_changed_ = false;
  bool have_plan_ = false;
  bool pending_goal_retry_ = false;
  bool front_end_path_available_ = false;
  int pending_goal_retry_count_ = 0;
  double trajStamp_ = 0.0;
  double trajectory_odom_yaw_ = 0.0;
  double last_replan_time_ = -std::numeric_limits<double>::infinity();
  double last_plan_attempt_time_ = -std::numeric_limits<double>::infinity();
};
