#pragma once

#include "gcopter/grid_map_2d.hpp"
#include "gcopter/traj_representation.hpp"

#include <Eigen/Eigen>

#include <string>
#include <vector>

// 对搜索路径进行简化、插值或重新采样。
bool SamplePath(
  const GridMap2D & map,
  const std::vector<Eigen::Vector2d> & input,
  std::vector<Eigen::Vector2d> & output);

// 根据采样路径生成后端所需的初始轨迹数据。
bool GenerateTrajectory(
  const std::vector<Eigen::Vector2d> & route,
  const std::vector<ConvexCorridor2D> & corridors,
  double nominal_speed,
  double minimum_segment_duration,
  const Eigen::Matrix<double, 2, 3> & start_pva,
  MincoTrajectoryData & output);

struct OptimizationOptions
{
  double max_velocity = 1.0;
  double max_acceleration = 1.0;
  double minimum_segment_duration = 0.1;
  double weight_time = 1.0;
  double weight_velocity = 10.0;
  double weight_acceleration = 10.0;
  int maximum_iterations = 30;
  double time_budget_ms = 20.0;
};

// 优化轨迹位置、时间和约束，结果写入 output。
bool OptimizeTrajectory(
  const GridMap2D & map,
  const MincoTrajectoryData & input,
  const OptimizationOptions & options,
  MincoTrajectoryData & output);

// 检查轨迹字段的尺寸、数值和时间区间。
bool ValidateTrajectoryData(
  const MincoTrajectoryData & input,
  std::string & reason);
