#include "gcopter/corridor_generator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{

ConvexCorridor2D makeBox(
  const Eigen::Vector2d & start, const Eigen::Vector2d & end,
  double forward_extension, double backward_extension,
  double left_width, double right_width)
{
  const Eigen::Vector2d direction = (end - start).normalized();
  const Eigen::Vector2d normal(-direction.y(), direction.x());
  ConvexCorridor2D result;
  result.A.resize(4, 2);
  result.b.resize(4);
  result.A.row(0) = direction.transpose();
  result.A.row(1) = -direction.transpose();
  result.A.row(2) = normal.transpose();
  result.A.row(3) = -normal.transpose();
  result.b(0) = direction.dot(end) + forward_extension;
  result.b(1) = -direction.dot(start) + backward_extension;
  result.b(2) = normal.dot(start) + left_width;
  result.b(3) = -normal.dot(start) + right_width;
  return result;
}

bool boxIsFree(
  const GridMap2D & map, const Eigen::Vector2d & start, const Eigen::Vector2d & end,
  double forward_extension, double backward_extension,
  double left_width, double right_width)
{
  const double length = (end - start).norm();
  const Eigen::Vector2d direction = (end - start) / length;
  const Eigen::Vector2d normal(-direction.y(), direction.x());
  const double spacing = std::max(0.5 * map.resolution, 1.0e-3);
  const double longitudinal_span = length + forward_extension + backward_extension;
  const double lateral_span = left_width + right_width;
  const int longitudinal = std::max(
    1, static_cast<int>(std::ceil(longitudinal_span / spacing)));
  const int lateral = std::max(1, static_cast<int>(std::ceil(lateral_span / spacing)));
  for (int i = 0; i <= longitudinal; ++i) {
    const double along = -backward_extension + longitudinal_span *
      static_cast<double>(i) / static_cast<double>(longitudinal);
    for (int j = 0; j <= lateral; ++j) {
      const double across = -right_width + lateral_span *
        static_cast<double>(j) / static_cast<double>(lateral);
      if (map.isOccupied(map.worldToGrid(start + direction * along + normal * across))) {
        return false;
      }
    }
  }
  return true;
}

std::vector<Eigen::Vector2d> clipPolygon(
  const std::vector<Eigen::Vector2d> & polygon,
  const Eigen::Vector2d & normal, double bound, double tolerance)
{
  std::vector<Eigen::Vector2d> clipped;
  if (polygon.empty()) {return clipped;}
  clipped.reserve(polygon.size() + 1);
  Eigen::Vector2d previous = polygon.back();
  double previous_distance = normal.dot(previous) - bound;
  bool previous_inside = previous_distance <= tolerance;
  for (const Eigen::Vector2d & current : polygon) {
    const double current_distance = normal.dot(current) - bound;
    const bool current_inside = current_distance <= tolerance;
    if (previous_inside != current_inside) {
      const double denominator = previous_distance - current_distance;
      if (std::abs(denominator) > 1.0e-14) {
        const double ratio = std::clamp(previous_distance / denominator, 0.0, 1.0);
        clipped.push_back(previous + ratio * (current - previous));
      }
    }
    if (current_inside) {clipped.push_back(current);}
    previous = current;
    previous_distance = current_distance;
    previous_inside = current_inside;
  }
  return clipped;
}

double corridorIntersectionArea(
  const GridMap2D & map, const ConvexCorridor2D & lhs,
  const ConvexCorridor2D & rhs, double tolerance)
{
  const double min_x = map.origin.x();
  const double min_y = map.origin.y();
  const double max_x = min_x + static_cast<double>(map.width) * map.resolution;
  const double max_y = min_y + static_cast<double>(map.height) * map.resolution;
  std::vector<Eigen::Vector2d> polygon{{min_x, min_y}, {max_x, min_y},
    {max_x, max_y}, {min_x, max_y}};
  for (const ConvexCorridor2D * corridor : {&lhs, &rhs}) {
    for (Eigen::Index row = 0; row < corridor->A.rows() && !polygon.empty(); ++row) {
      polygon = clipPolygon(
        polygon, corridor->A.row(row).transpose(), corridor->b(row), tolerance);
    }
  }
  if (polygon.size() < 3) {return 0.0;}
  double twice_area = 0.0;
  for (size_t index = 0; index < polygon.size(); ++index) {
    const Eigen::Vector2d & current = polygon[index];
    const Eigen::Vector2d & next = polygon[(index + 1) % polygon.size()];
    twice_area += current.x() * next.y() - current.y() * next.x();
  }
  return 0.5 * std::abs(twice_area);
}

}  // namespace

bool ConvexCorridor2D::valid() const
{
  return A.rows() >= 3 && A.cols() == 2 && b.size() == A.rows() &&
    A.allFinite() && b.allFinite();
}

bool ConvexCorridor2D::contains(const Eigen::Vector2d & point, double tolerance) const
{
  return valid() && point.allFinite() &&
    (A * point - b).maxCoeff() <= tolerance;
}

bool GenerateCorridor(
  const GridMap2D & map,
  const std::vector<Eigen::Vector2d> & sparse_path,
  const CorridorOptions & options,
  std::vector<ConvexCorridor2D> & corridors,
  std::string & reason)
{
  corridors.clear();
  reason.clear();
  if (!map.valid() || sparse_path.size() < 2 ||
    !std::isfinite(options.obstacle_search_radius) || options.obstacle_search_radius <= 0.0 ||
    options.firi_iterations <= 0 || !std::isfinite(options.minimum_overlap) ||
    options.minimum_overlap < 0.0)
  {
    reason = "configuration";
    return false;
  }

  corridors.reserve(sparse_path.size() - 1);
  const double initial_width = std::max(0.2 * map.resolution, 1.0e-3);
  const double initial_extension = std::max(0.5 * options.minimum_overlap, initial_width);
  for (size_t segment = 0; segment + 1 < sparse_path.size(); ++segment) {
    const Eigen::Vector2d & start = sparse_path[segment];
    const Eigen::Vector2d & end = sparse_path[segment + 1];
    if (!start.allFinite() || !end.allFinite() || (end - start).norm() <= 1.0e-8) {
      reason = "degenerate_seed";
      return false;
    }
    if (map.isOccupied(map.worldToGrid(start)) || map.isOccupied(map.worldToGrid(end))) {
      reason = "occupied_seed";
      return false;
    }

    double forward_extension = 0.0;
    double backward_extension = 0.0;
    double left_width = initial_width;
    double right_width = initial_width;
    if (!boxIsFree(
        map, start, end, 0.0, 0.0, left_width, right_width))
    {
      reason = "seed_not_free";
      return false;
    }
    if (boxIsFree(
        map, start, end, initial_extension, backward_extension,
        left_width, right_width))
    {
      forward_extension = initial_extension;
    }
    if (boxIsFree(
        map, start, end, forward_extension, initial_extension,
        left_width, right_width))
    {
      backward_extension = initial_extension;
    }
    for (int iteration = 0; iteration < options.firi_iterations; ++iteration) {
      const double proposed_left = std::min(
        options.obstacle_search_radius,
        std::max(left_width * 2.0, left_width + map.resolution));
      if (boxIsFree(
          map, start, end, forward_extension, backward_extension,
          proposed_left, right_width))
      {
        left_width = proposed_left;
      }
      const double proposed_right = std::min(
        options.obstacle_search_radius,
        std::max(right_width * 2.0, right_width + map.resolution));
      if (boxIsFree(
          map, start, end, forward_extension, backward_extension,
          left_width, proposed_right))
      {
        right_width = proposed_right;
      }
      const double proposed_forward = std::min(
        options.obstacle_search_radius, forward_extension + map.resolution);
      if (boxIsFree(
          map, start, end, proposed_forward, backward_extension,
          left_width, right_width))
      {
        forward_extension = proposed_forward;
      }
      const double proposed_backward = std::min(
        options.obstacle_search_radius, backward_extension + map.resolution);
      if (boxIsFree(
          map, start, end, forward_extension, proposed_backward,
          left_width, right_width))
      {
        backward_extension = proposed_backward;
      }
    }
    // Refine the unused fraction of the final grid-sized growth step while
    // retaining the same hard collision test.
    const auto refine_extent = [&](double & value, const auto & is_free) {
        double lower = value;
        double upper = std::min(options.obstacle_search_radius, value + map.resolution);
        for (int refinement = 0; refinement < 4 && upper - lower > 1.0e-4; ++refinement) {
          const double middle = 0.5 * (lower + upper);
          if (is_free(middle)) {lower = middle;} else {upper = middle;}
        }
        value = lower;
      };
    refine_extent(left_width, [&](double candidate) {
      return boxIsFree(map, start, end, forward_extension, backward_extension,
        candidate, right_width);
    });
    refine_extent(right_width, [&](double candidate) {
      return boxIsFree(map, start, end, forward_extension, backward_extension,
        left_width, candidate);
    });
    refine_extent(forward_extension, [&](double candidate) {
      return boxIsFree(map, start, end, candidate, backward_extension,
        left_width, right_width);
    });
    refine_extent(backward_extension, [&](double candidate) {
      return boxIsFree(map, start, end, forward_extension, candidate,
        left_width, right_width);
    });
    ConvexCorridor2D corridor = makeBox(
      start, end, forward_extension, backward_extension, left_width, right_width);
    if (!corridor.contains(start) || !corridor.contains(end)) {
      reason = "seed_outside_corridor";
      return false;
    }
    corridors.push_back(std::move(corridor));
  }

  // Validate the actual polygon intersection. Requiring four axis-aligned
  // points around a turn is overly restrictive: at a legitimate corner it can
  // demand that the incoming corridor extend through the obstacle that caused
  // the route to turn. A positive area still rejects point-only contact while
  // permitting asymmetric overlap in the locally free quadrant.
  const double minimum_area = std::max(
    1.0e-12, options.merge_tolerance * options.merge_tolerance);
  for (size_t joint = 1; joint < sparse_path.size() - 1; ++joint) {
    if (corridorIntersectionArea(
        map, corridors[joint - 1], corridors[joint], options.merge_tolerance) <= minimum_area)
    {
      reason = "insufficient_overlap";
      return false;
    }
  }
  return true;
}
