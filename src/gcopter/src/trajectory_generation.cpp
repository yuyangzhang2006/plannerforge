#include "gcopter/trajectory_generation.hpp"

#include "gcopter/minco.hpp"
#include "gcopter/trajectory.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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
      if (map.isOccupied(map.worldToGrid(position))) {corridor_penalty += 1.0;}
      if (piece >= static_cast<int>(data.corridors.size()) ||
        !data.corridors[static_cast<size_t>(piece)].contains(position, 1.0e-6))
      {
        corridor_penalty += 1.0;
      }
    }
  }
  result.corridor_safe = corridor_penalty == 0.0;
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
  output.push_back(input.front());
  Eigen::Vector2i previous_direction = discreteDirection(map, input[0], input[1]);
  for (size_t index = 1; index + 1 < input.size(); ++index) {
    const Eigen::Vector2i next_direction = discreteDirection(map, input[index], input[index + 1]);
    const MapSemantic before = map.semanticAt(map.worldToGrid(input[index - 1]));
    const MapSemantic current = map.semanticAt(map.worldToGrid(input[index]));
    const MapSemantic after = map.semanticAt(map.worldToGrid(input[index + 1]));
    if (next_direction != previous_direction || current != before || current != after) {
      if ((input[index] - output.back()).norm() > 1.0e-8) {output.push_back(input[index]);}
    }
    previous_direction = next_direction;
  }
  if ((input.back() - output.back()).norm() > 1.0e-8) {output.push_back(input.back());}
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
  std::string reason;
  return ValidateTrajectoryData(output, reason);
}

bool OptimizeTrajectory(
  const GridMap2D & map,
  const MincoTrajectoryData & input,
  const OptimizationOptions & options,
  MincoTrajectoryData & output)
{
  output.clear();
  std::string reason;
  if (!map.valid() || !ValidateTrajectoryData(input, reason) ||
    input.corridors.size() != static_cast<size_t>(input.segment_count) ||
    !std::isfinite(options.max_velocity) || options.max_velocity <= 0.0 ||
    !std::isfinite(options.max_acceleration) || options.max_acceleration <= 0.0 ||
    !std::isfinite(options.minimum_segment_duration) || options.minimum_segment_duration <= 0.0 ||
    options.maximum_iterations <= 0 || !std::isfinite(options.time_budget_ms) ||
    options.time_budget_ms <= 0.0)
  {
    return false;
  }
  output = input;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double, std::milli>(options.time_budget_ms);

  // Establish a feasible planning envelope first. Uniform time scaling preserves
  // the spatial curve and monotonically reduces velocity and acceleration.
  Evaluation current = evaluate(map, output, options);
  for (int pass = 0; pass < 8 && std::chrono::steady_clock::now() < deadline; ++pass) {
    const double scale = std::max({1.0,
      current.maximum_velocity / options.max_velocity,
      std::sqrt(current.maximum_acceleration / options.max_acceleration)});
    if (!std::isfinite(scale)) {return false;}
    if (scale <= 1.001) {break;}
    output.segment_durations *= std::min(4.0, scale * 1.01);
    current = evaluate(map, output, options);
  }

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
      MincoTrajectoryData candidate = output;
      candidate.segment_durations(segment) = std::max(options.minimum_segment_duration,
        0.95 * candidate.segment_durations(segment));
      if (candidate.segment_durations(segment) < output.segment_durations(segment) - 1.0e-9) {
        const Evaluation trial = evaluate(map, candidate, options);
        if (trial.corridor_safe && trial.maximum_velocity <= options.max_velocity * 1.01 &&
          trial.maximum_acceleration <= options.max_acceleration * 1.01 &&
          trial.objective + 1.0e-9 < current.objective)
        {
          output = std::move(candidate);
          current = trial;
          improved = true;
        }
      }
    }
    if (!improved) {spatial_step *= 0.5;}
    if (spatial_step < 0.02 * map.resolution) {break;}
  }

  current = evaluate(map, output, options);
  const bool envelope_ok = current.maximum_velocity <= options.max_velocity * 1.01 &&
    current.maximum_acceleration <= options.max_acceleration * 1.01;
  return std::isfinite(current.objective) && current.corridor_safe && envelope_ok &&
    ValidateTrajectoryData(output, reason);
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
