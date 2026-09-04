#include "gcopter/trajectory_generation.hpp"

#include <cmath>
#include <string>
#include <vector>










// ============================================================
// TODO(阶段② 路径采样)
// 在这里实现路径简化、插值、等距采样或安全走廊采样。
// 当前基线在检查输入后直接复制路径点。
// ============================================================












bool SamplePath(
  const GridMap2D & map,
  const std::vector<Eigen::Vector2d> & input,
  std::vector<Eigen::Vector2d> & output)
{
  output.clear();
  if (!map.valid() || input.size() < 2) {
    return false;
  }
  for (const Eigen::Vector2d & point : input) {
    if (!point.allFinite()) {
      return false;
    }
  }
  output = input;
  return true;
}










// ============================================================
// TODO(阶段② 初始轨迹与时间分配)
// 在这里生成边界状态、连接点和每段时间，为后端优化提供初值。
// 当前基线按 segment_length / nominal_speed 分配时间。
// ============================================================










bool GenerateTrajectory(
  const std::vector<Eigen::Vector2d> & route,
  double nominal_speed,
  MincoTrajectoryData & output)
{
  output.clear();
  if (!std::isfinite(nominal_speed) || nominal_speed <= 0.0 || route.size() < 2) {
    return false;
  }
  for (const Eigen::Vector2d & point : route) {
    if (!point.allFinite()) {
      return false;
    }
  }

  // 连续重复点会产生零时长轨迹段，生成前先合并。
  constexpr double kDuplicateEps = 1.0e-6;
  std::vector<Eigen::Vector2d> cleaned;
  cleaned.reserve(route.size());
  cleaned.push_back(route.front());
  for (size_t index = 1; index < route.size(); ++index) {
    if ((route[index] - cleaned.back()).norm() > kDuplicateEps) {
      cleaned.push_back(route[index]);
    }
  }
  if (cleaned.size() < 2) {
    return false;
  }

  const int point_count = static_cast<int>(cleaned.size());
  const int segment_count = point_count - 1;
  const int via_count = point_count - 2;
  output.start_pva.setZero();
  output.end_pva.setZero();
  output.segment_count = segment_count;
  output.intermediate_positions.resize(2, via_count);
  output.segment_durations.resize(segment_count);
  output.waypoint_is_spacious.resize(via_count);
  output.waypoint_is_spacious.setOnes();
  output.cross_hole_intervals.clear();
  output.start_pva.col(0) = cleaned.front();
  output.end_pva.col(0) = cleaned.back();

  for (int index = 0; index < via_count; ++index) {
    output.intermediate_positions.col(index) = cleaned[static_cast<size_t>(index + 1)];
  }
  for (int index = 0; index < segment_count; ++index) {
    const double segment_length =
      (cleaned[static_cast<size_t>(index + 1)] - cleaned[static_cast<size_t>(index)]).norm();
    const double segment_time = segment_length / nominal_speed;
    if (segment_length <= kDuplicateEps || !std::isfinite(segment_time) || segment_time <= 0.0) {
      output.clear();
      return false;
    }
    output.segment_durations(index) = segment_time;
  }

  std::string reason;
  if (!ValidateTrajectoryData(output, reason)) {
    output.clear();
    return false;
  }
  return true;
}









// ============================================================
// TODO(阶段② 后端优化)
// 在这里加入轨迹平滑、运动学约束、时间优化和障碍距离约束。
// 当前基线直接复制 input，接入算法后由 output 返回完整结果。
// ============================================================










bool OptimizeTrajectory(
  const GridMap2D & map,
  const MincoTrajectoryData & input,
  MincoTrajectoryData & output)
{
  (void)map;
  output = input;
  return true;
}

bool ValidateTrajectoryData(
  const MincoTrajectoryData & input,
  std::string & reason)
{
  reason.clear();
  if (input.segment_count <= 0) {
    reason = "segment_count";
    return false;
  }
  if (!input.start_pva.allFinite()) {
    reason = "start_pva";
    return false;
  }
  if (!input.end_pva.allFinite()) {
    reason = "end_pva";
    return false;
  }
  if (input.intermediate_positions.rows() != 2 ||
    input.intermediate_positions.cols() != input.segment_count - 1)
  {
    reason = "intermediate_positions_shape";
    return false;
  }
  if (!input.intermediate_positions.allFinite()) {
    reason = "intermediate_positions_value";
    return false;
  }
  if (input.segment_durations.size() != input.segment_count) {
    reason = "segment_durations_shape";
    return false;
  }
  if (!input.segment_durations.allFinite() ||
    (input.segment_durations.array() <= 0.0).any())
  {
    reason = "segment_durations_value";
    return false;
  }
  if (input.waypoint_is_spacious.size() != input.segment_count - 1) {
    reason = "waypoint_spacious_shape";
    return false;
  }
  for (const MincoTrajectoryData::CrossHoleInterval & interval : input.cross_hole_intervals) {
    if (!std::isfinite(interval.enter_time) || !std::isfinite(interval.exit_time) ||
      !std::isfinite(interval.active_start_time) || !std::isfinite(interval.active_end_time) ||
      interval.enter_time < 0.0 || interval.exit_time <= interval.enter_time ||
      interval.active_start_time < 0.0 || interval.active_end_time <= interval.active_start_time)
    {
      reason = "cross_hole_interval";
      return false;
    }
  }
  return true;
}
