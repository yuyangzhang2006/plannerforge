#include "gcopter/trajectory_generation.hpp"

#include "gcopter/minco.hpp"
#include "gcopter/trajectory.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace
{

Eigen::Vector2i discreteDirection(
  const GridMap2D & map, const Eigen::Vector2d & from, const Eigen::Vector2d & to)
{
  const Eigen::Vector2i delta = map.worldToGrid(to) - map.worldToGrid(from);
  return {delta.x() == 0 ? 0 : (delta.x() > 0 ? 1 : -1),
    delta.y() == 0 ? 0 : (delta.y() > 0 ? 1 : -1)};
}

bool projectToCorridors(
  Eigen::Vector2d & point, const ConvexCorridor2D & lhs, const ConvexCorridor2D & rhs)
{
  for (int pass = 0; pass < 12; ++pass) {
    bool changed = false;
    for (const ConvexCorridor2D * corridor : {&lhs, &rhs}) {
      for (Eigen::Index row = 0; row < corridor->A.rows(); ++row) {
        const Eigen::Vector2d normal = corridor->A.row(row).transpose();
        const double violation = normal.dot(point) - corridor->b(row);
        if (violation > 0.0) {
          const double norm_squared = normal.squaredNorm();
          if (norm_squared <= 1.0e-12) {return false;}
          point -= (violation + 1.0e-8) * normal / norm_squared;
          changed = true;
        }
      }
    }
    if (!changed) {break;}
  }
  return lhs.contains(point, 1.0e-6) && rhs.contains(point, 1.0e-6);
}

struct Evaluation
{
  double objective = std::numeric_limits<double>::infinity();
  double maximum_velocity = 0.0;
  double maximum_acceleration = 0.0;
  double maximum_corridor_violation = 0.0;
  int occupied_samples = 0;
  bool corridor_safe = false;
};

Evaluation evaluate(
  const GridMap2D & map, const MincoTrajectoryData & data,
  const OptimizationOptions & options)
{
  Evaluation result;
  minco::MINCO_S3NU minco;
  minco.setConditions(data.start_pva, data.end_pva, data.segment_count);
  minco.setParameters(data.intermediate_positions, data.segment_durations);
  Trajectory<5, 2> trajectory;
  minco.getTrajectory(trajectory);
  double jerk_energy = 0.0;
  minco.getEnergy(jerk_energy);
  if (!std::isfinite(jerk_energy) || trajectory.getPieceNum() != data.segment_count) {
    return result;
  }
  result.corridor_safe = true;

  double velocity_penalty = 0.0;
  double acceleration_penalty = 0.0;
  double corridor_penalty = 0.0;
  constexpr int kSamplesPerPiece = 16;
  for (int piece = 0; piece < data.segment_count; ++piece) {
    const double duration = data.segment_durations(piece);
    for (int sample = 0; sample <= kSamplesPerPiece; ++sample) {
      const double local_time = duration * static_cast<double>(sample) / kSamplesPerPiece;
      const Eigen::Vector2d position = trajectory[piece].getPos(local_time).head<2>();
      const double velocity = trajectory[piece].getVel(local_time).head<2>().norm();
      const double acceleration = trajectory[piece].getAcc(local_time).head<2>().norm();
      if (!position.allFinite() || !std::isfinite(velocity) || !std::isfinite(acceleration)) {
        return result;
      }
      result.maximum_velocity = std::max(result.maximum_velocity, velocity);
      result.maximum_acceleration = std::max(result.maximum_acceleration, acceleration);
      const double velocity_excess = std::max(0.0, velocity - options.max_velocity);
      const double acceleration_excess = std::max(0.0, acceleration - options.max_acceleration);
      velocity_penalty += velocity_excess * velocity_excess * duration / kSamplesPerPiece;
      acceleration_penalty += acceleration_excess * acceleration_excess * duration / kSamplesPerPiece;
      if (map.isOccupied(map.worldToGrid(position))) {
        corridor_penalty += 1.0;
        ++result.occupied_samples;
        result.corridor_safe = false;
      }
      if (piece >= static_cast<int>(data.corridors.size())) {
        corridor_penalty += 1.0;
        result.corridor_safe = false;
      } else {
        const ConvexCorridor2D & corridor = data.corridors[static_cast<size_t>(piece)];
        const double violation = std::max(0.0, (corridor.A * position - corridor.b).maxCoeff());
        result.maximum_corridor_violation = std::max(
          result.maximum_corridor_violation, violation);
        if (violation > 1.0e-6) {
          // A continuous metric gives coordinate descent a useful direction;
          // the former binary sample count was flat almost everywhere.
          const double normalized = violation / std::max(map.resolution, 1.0e-6);
          corridor_penalty += normalized * normalized;
          result.corridor_safe = false;
        }
      }
    }
  }
  result.objective = jerk_energy + options.weight_time * data.segment_durations.sum() +
    options.weight_velocity * velocity_penalty +
    options.weight_acceleration * acceleration_penalty + 1.0e8 * corridor_penalty;
  return result;
}

}  // namespace

bool SamplePath(
  const GridMap2D & map,
  const std::vector<Eigen::Vector2d> & input,
  std::vector<Eigen::Vector2d> & output)
{
  output.clear();
  if (!map.valid() || input.size() < 2) {return false;}
  for (const Eigen::Vector2d & point : input) {
    if (!point.allFinite() || !map.isInside(map.worldToGrid(point))) {return false;}
  }
  std::vector<uint8_t> keep(input.size(), 0U);
  keep.front() = 1U;
  keep.back() = 1U;
  Eigen::Vector2i previous_direction = discreteDirection(map, input[0], input[1]);
  for (size_t index = 1; index + 1 < input.size(); ++index) {
    const Eigen::Vector2i next_direction = discreteDirection(map, input[index], input[index + 1]);
    const MapSemantic before = map.semanticAt(map.worldToGrid(input[index - 1]));
    const MapSemantic current = map.semanticAt(map.worldToGrid(input[index]));
    const MapSemantic after = map.semanticAt(map.worldToGrid(input[index + 1]));
    if (next_direction != previous_direction || current != before || current != after) {
      keep[index] = 1U;
    }
    previous_direction = next_direction;
  }

  // Bound the distance between retained seeds. Very long straight pieces next
  // to a short corner piece create extreme duration ratios and can make the
  // global minimum-jerk interpolant overshoot by metres. Ten grid steps keep
  // the representation sparse while giving MINCO well-scaled local support.
  constexpr size_t kMaximumSeedSteps = 10;
  size_t last_kept = 0;
  for (size_t index = 1; index < input.size(); ++index) {
    if (keep[index] == 0U) {continue;}
    while (index - last_kept > kMaximumSeedSteps) {
      last_kept += kMaximumSeedSteps;
      keep[last_kept] = 1U;
    }
    last_kept = index;
  }
  output.reserve(input.size());
  for (size_t index = 0; index < input.size(); ++index) {
    if (keep[index] != 0U &&
      (output.empty() || (input[index] - output.back()).norm() > 1.0e-8))
    {
      output.push_back(input[index]);
    }
  }
  return output.size() >= 2;
}

bool GenerateTrajectory(
  const std::vector<Eigen::Vector2d> & route,
  const std::vector<ConvexCorridor2D> & corridors,
  double nominal_speed,
  double minimum_segment_duration,
  const Eigen::Matrix<double, 2, 3> & start_pva,
  MincoTrajectoryData & output)
{
  output.clear();
  if (!std::isfinite(nominal_speed) || nominal_speed <= 0.0 ||
    !std::isfinite(minimum_segment_duration) || minimum_segment_duration <= 0.0 ||
    !start_pva.allFinite() || route.size() < 2 || corridors.size() != route.size() - 1)
  {
    return false;
  }
  constexpr double kDuplicateEps = 1.0e-8;
  for (const Eigen::Vector2d & point : route) {
    if (!point.allFinite()) {return false;}
  }
  const int segment_count = static_cast<int>(route.size()) - 1;
  output.start_pva = start_pva;
  output.start_pva.col(0) = route.front();
  output.end_pva.setZero();
  output.end_pva.col(0) = route.back();
  output.segment_count = segment_count;
  output.intermediate_positions.resize(2, segment_count - 1);
  output.segment_durations.resize(segment_count);
  output.waypoint_is_spacious.setOnes(segment_count - 1);
  output.corridors = corridors;
  for (int index = 0; index < segment_count - 1; ++index) {
    output.intermediate_positions.col(index) = route[static_cast<size_t>(index + 1)];
  }
  for (int index = 0; index < segment_count; ++index) {
    const double length =
      (route[static_cast<size_t>(index + 1)] - route[static_cast<size_t>(index)]).norm();
    if (length <= kDuplicateEps) {output.clear(); return false;}
    output.segment_durations(index) = std::max(minimum_segment_duration, length / nominal_speed);
  }
  // Prevent a long straight piece beside a grid-sized corner piece from
  // dominating the global quintic derivative solution. Only lengthen the
  // shorter duration, which slows the vehicle near turns without weakening
  // the nominal-speed or minimum-duration constraints.
  constexpr double kMaximumAdjacentDurationRatio = 2.0;
  for (int pass = 0; pass < segment_count; ++pass) {
    bool changed = false;
    for (int index = 1; index < segment_count; ++index) {
      double & previous = output.segment_durations(index - 1);
      double & current = output.segment_durations(index);
      if (current > kMaximumAdjacentDurationRatio * previous) {
        previous = current / kMaximumAdjacentDurationRatio;
        changed = true;
      } else if (previous > kMaximumAdjacentDurationRatio * current) {
        current = previous / kMaximumAdjacentDurationRatio;
        changed = true;
      }
    }
    if (!changed) {break;}
  }
  std::string reason;
  return ValidateTrajectoryData(output, reason);
}

bool OptimizeTrajectory(
  const GridMap2D & map,
  const MincoTrajectoryData & input,
  const OptimizationOptions & options,
  MincoTrajectoryData & output,
  std::string * failure_reason)
{
  output.clear();
  if (failure_reason != nullptr) {failure_reason->clear();}
  std::string reason;
  if (!map.valid() || !ValidateTrajectoryData(input, reason) ||
    input.corridors.size() != static_cast<size_t>(input.segment_count) ||
    !std::isfinite(options.max_velocity) || options.max_velocity <= 0.0 ||
    !std::isfinite(options.max_acceleration) || options.max_acceleration <= 0.0 ||
    !std::isfinite(options.minimum_segment_duration) || options.minimum_segment_duration <= 0.0 ||
    options.maximum_iterations <= 0 || !std::isfinite(options.time_budget_ms) ||
    options.time_budget_ms <= 0.0)
  {
    if (failure_reason != nullptr) {*failure_reason = "invalid_input";}
    return false;
  }
  output = input;
  // Establish a feasible planning envelope first. Uniform time scaling preserves
  // the spatial curve and monotonically reduces velocity and acceleration. Run
  // this bounded mandatory phase before the discretionary optimization budget.
  Evaluation current = evaluate(map, output, options);
  for (int pass = 0; pass < 8; ++pass) {
    const double scale = std::max({1.0,
      current.maximum_velocity / options.max_velocity,
      std::sqrt(current.maximum_acceleration / options.max_acceleration)});
    if (!std::isfinite(scale)) {
      if (failure_reason != nullptr) {*failure_reason = "nonfinite_time_scale";}
      return false;
    }
    if (scale <= 1.001) {break;}
    output.segment_durations *= std::min(4.0, scale * 1.01);
    current = evaluate(map, output, options);
  }
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double, std::milli>(options.time_budget_ms);

  // Bounded projected coordinate descent. Every intermediate point remains in
  // the ordered overlap of its adjacent corridors; durations remain positive.
  double spatial_step = std::max(0.25 * map.resolution, 1.0e-3);
  for (int iteration = 0;
    iteration < options.maximum_iterations && std::chrono::steady_clock::now() < deadline;
    ++iteration)
  {
    bool improved = false;
    for (int waypoint = 0; waypoint < output.segment_count - 1; ++waypoint) {
      for (int axis = 0; axis < 2; ++axis) {
        MincoTrajectoryData candidate = output;
        candidate.intermediate_positions(axis, waypoint) += spatial_step;
        Eigen::Vector2d projected = candidate.intermediate_positions.col(waypoint);
        if (!projectToCorridors(projected, candidate.corridors[static_cast<size_t>(waypoint)],
          candidate.corridors[static_cast<size_t>(waypoint + 1)]))
        {
          continue;
        }
        candidate.intermediate_positions.col(waypoint) = projected;
        Evaluation trial = evaluate(map, candidate, options);
        if (trial.objective + 1.0e-9 < current.objective) {
          output = std::move(candidate); current = trial; improved = true; continue;
        }
        candidate = output;
        candidate.intermediate_positions(axis, waypoint) -= spatial_step;
        projected = candidate.intermediate_positions.col(waypoint);
        if (projectToCorridors(projected, candidate.corridors[static_cast<size_t>(waypoint)],
          candidate.corridors[static_cast<size_t>(waypoint + 1)]))
        {
          candidate.intermediate_positions.col(waypoint) = projected;
          trial = evaluate(map, candidate, options);
          if (trial.objective + 1.0e-9 < current.objective) {
            output = std::move(candidate); current = trial; improved = true;
          }
        }
      }
    }
    for (int segment = 0; segment < output.segment_count &&
      std::chrono::steady_clock::now() < deadline; ++segment)
    {
      for (const double factor : {0.95, 1.05}) {
        MincoTrajectoryData candidate = output;
        candidate.segment_durations(segment) = std::max(options.minimum_segment_duration,
          factor * candidate.segment_durations(segment));
        if (std::abs(candidate.segment_durations(segment) - output.segment_durations(segment)) >
          1.0e-9)
        {
          const Evaluation trial = evaluate(map, candidate, options);
          const bool limits_ok = trial.maximum_velocity <= options.max_velocity * 1.01 &&
            trial.maximum_acceleration <= options.max_acceleration * 1.01;
          if (trial.objective + 1.0e-9 < current.objective &&
            (!current.corridor_safe || (trial.corridor_safe && limits_ok)))
          {
            output = std::move(candidate);
            current = trial;
            improved = true;
          }
        }
      }
    }
    if (!improved) {spatial_step *= 0.5;}
    if (spatial_step < 0.02 * map.resolution) {break;}
  }

  current = evaluate(map, output, options);
  // Spatial waypoint updates can improve jerk while raising derivatives again.
  // Restore the hard planning envelope once more; uniform scaling leaves the
  // already verified spatial curve and corridor membership unchanged.
  for (int pass = 0; pass < 8; ++pass) {
    const double scale = std::max({1.0,
      current.maximum_velocity / options.max_velocity,
      std::sqrt(current.maximum_acceleration / options.max_acceleration)});
    if (!std::isfinite(scale)) {
      if (failure_reason != nullptr) {*failure_reason = "nonfinite_final_time_scale";}
      return false;
    }
    if (scale <= 1.001) {break;}
    output.segment_durations *= std::min(4.0, scale * 1.01);
    current = evaluate(map, output, options);
  }
  const bool envelope_ok = current.maximum_velocity <= options.max_velocity * 1.01 &&
    current.maximum_acceleration <= options.max_acceleration * 1.01;
  const bool valid = ValidateTrajectoryData(output, reason);
  const bool success = std::isfinite(current.objective) && current.corridor_safe &&
    envelope_ok && valid;
  if (!success && failure_reason != nullptr) {
    std::ostringstream stream;
    stream << "safe=" << (current.corridor_safe ? "true" : "false")
           << " max_corridor_violation=" << current.maximum_corridor_violation
           << " occupied_samples=" << current.occupied_samples
           << " vmax=" << current.maximum_velocity
           << " amax=" << current.maximum_acceleration;
    if (!valid) {stream << " validation=" << reason;}
    *failure_reason = stream.str();
  }
  return success;
}

bool PrepareSafeInitialTrajectory(
  const GridMap2D & map,
  const MincoTrajectoryData & input,
  const OptimizationOptions & options,
  MincoTrajectoryData & output,
  std::string * failure_reason)
{
  output.clear();
  if (failure_reason != nullptr) {failure_reason->clear();}
  std::string validation_reason;
  if (!map.valid() || !ValidateTrajectoryData(input, validation_reason) ||
    input.corridors.size() != static_cast<size_t>(input.segment_count) ||
    !std::isfinite(options.max_velocity) || options.max_velocity <= 0.0 ||
    !std::isfinite(options.max_acceleration) || options.max_acceleration <= 0.0)
  {
    if (failure_reason != nullptr) {
      *failure_reason = validation_reason.empty() ? "invalid_input" : validation_reason;
    }
    return false;
  }

  output = input;
  Evaluation evaluation = evaluate(map, output, options);
  // 统一延长全部分段只改变时间参数，不改变空间曲线，因此不会破坏已经满足的
  // corridor 和碰撞条件，同时会单调降低速度与加速度。
  for (int pass = 0; pass < 8; ++pass) {
    const double scale = std::max({1.0,
      evaluation.maximum_velocity / options.max_velocity,
      std::sqrt(evaluation.maximum_acceleration / options.max_acceleration)});
    if (!std::isfinite(scale)) {break;}
    if (scale <= 1.001) {break;}
    output.segment_durations *= std::min(4.0, scale * 1.01);
    evaluation = evaluate(map, output, options);
  }

  const bool envelope_ok = evaluation.maximum_velocity <= options.max_velocity * 1.01 &&
    evaluation.maximum_acceleration <= options.max_acceleration * 1.01;
  const bool valid = ValidateTrajectoryData(output, validation_reason);
  const bool success = std::isfinite(evaluation.objective) && evaluation.corridor_safe &&
    envelope_ok && valid;
  if (!success && failure_reason != nullptr) {
    std::ostringstream stream;
    stream << "safe=" << (evaluation.corridor_safe ? "true" : "false")
           << " max_corridor_violation=" << evaluation.maximum_corridor_violation
           << " occupied_samples=" << evaluation.occupied_samples
           << " vmax=" << evaluation.maximum_velocity
           << " amax=" << evaluation.maximum_acceleration;
    if (!valid) {stream << " validation=" << validation_reason;}
    *failure_reason = stream.str();
  }
  return success;
}

bool ValidateTrajectoryData(const MincoTrajectoryData & input, std::string & reason)
{
  reason.clear();
  if (input.segment_count <= 0) {reason = "segment_count"; return false;}
  if (!input.start_pva.allFinite()) {reason = "start_pva"; return false;}
  if (!input.end_pva.allFinite()) {reason = "end_pva"; return false;}
  if (input.intermediate_positions.rows() != 2 ||
    input.intermediate_positions.cols() != input.segment_count - 1 ||
    !input.intermediate_positions.allFinite())
  {reason = "intermediate_positions"; return false;}
  if (input.segment_durations.size() != input.segment_count ||
    !input.segment_durations.allFinite() || (input.segment_durations.array() <= 0.0).any())
  {reason = "segment_durations"; return false;}
  if (input.waypoint_is_spacious.size() != input.segment_count - 1) {
    reason = "waypoint_spacious"; return false;
  }
  if (!input.corridors.empty() &&
    input.corridors.size() != static_cast<size_t>(input.segment_count))
  {reason = "corridors"; return false;}
  for (const ConvexCorridor2D & corridor : input.corridors) {
    if (!corridor.valid()) {reason = "corridor_value"; return false;}
  }
  for (const MincoTrajectoryData::CrossHoleInterval & interval : input.cross_hole_intervals) {
    if (!std::isfinite(interval.enter_time) || !std::isfinite(interval.exit_time) ||
      !std::isfinite(interval.active_start_time) || !std::isfinite(interval.active_end_time) ||
      interval.enter_time < 0.0 || interval.exit_time <= interval.enter_time ||
      interval.active_start_time < 0.0 || interval.active_end_time <= interval.active_start_time)
    {reason = "cross_hole_interval"; return false;}
  }
  return true;
}
