#include "gcopter/corridor_generator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace
{

struct LocalObstacleSet
{
  const GridMap2D & map;
  Eigen::Vector2d start;
  Eigen::Vector2d end;
  Eigen::Vector2d direction;
  Eigen::Vector2d normal;
  double length = 0.0;
  std::vector<Eigen::Vector2d> occupied_centers;

  LocalObstacleSet(
    const GridMap2D & input_map, const Eigen::Vector2d & input_start,
    const Eigen::Vector2d & input_end, double search_radius)
  : map(input_map), start(input_start), end(input_end),
    direction((input_end - input_start).normalized()),
    normal(-direction.y(), direction.x()), length((input_end - input_start).norm())
  {
    // 所有候选 corridor 都包含在 seed 两端与 search_radius 构成的包围盒内。
    // 这里只扫描一次局部栅格，后续几十次扩张测试只访问其中的障碍格。
    const double padding = std::sqrt(2.0) * search_radius + map.resolution;
    const Eigen::Vector2d lower = start.cwiseMin(end) - Eigen::Vector2d::Constant(padding);
    const Eigen::Vector2d upper = start.cwiseMax(end) + Eigen::Vector2d::Constant(padding);
    Eigen::Vector2i minimum = map.worldToGrid(lower);
    Eigen::Vector2i maximum = map.worldToGrid(upper);
    minimum.x() = std::clamp(minimum.x(), 0, map.width - 1);
    minimum.y() = std::clamp(minimum.y(), 0, map.height - 1);
    maximum.x() = std::clamp(maximum.x(), 0, map.width - 1);
    maximum.y() = std::clamp(maximum.y(), 0, map.height - 1);
    for (int y = minimum.y(); y <= maximum.y(); ++y) {
      for (int x = minimum.x(); x <= maximum.x(); ++x) {
        const Eigen::Vector2i cell(x, y);
        if (map.isOccupied(cell)) {occupied_centers.push_back(map.gridToWorld(cell));}
      }
    }
  }

  bool isFree(
    double forward_extension, double backward_extension,
    double left_width, double right_width) const
  {
    const double along_min = direction.dot(start) - backward_extension;
    const double along_max = direction.dot(start) + length + forward_extension;
    const double lateral_origin = normal.dot(start);
    const double lateral_min = lateral_origin - right_width;
    const double lateral_max = lateral_origin + left_width;
    const std::array<Eigen::Vector2d, 4> corners = {
      start - direction * backward_extension - normal * right_width,
      start - direction * backward_extension + normal * left_width,
      end + direction * forward_extension - normal * right_width,
      end + direction * forward_extension + normal * left_width};
    const double map_min_x = map.origin.x();
    const double map_min_y = map.origin.y();
    const double map_max_x = map_min_x + static_cast<double>(map.width) * map.resolution;
    const double map_max_y = map_min_y + static_cast<double>(map.height) * map.resolution;
    for (const Eigen::Vector2d & corner : corners) {
      if (corner.x() < map_min_x || corner.x() > map_max_x ||
        corner.y() < map_min_y || corner.y() > map_max_y)
      {
        return false;
      }
    }

    double box_min_x = corners[0].x();
    double box_max_x = corners[0].x();
    double box_min_y = corners[0].y();
    double box_max_y = corners[0].y();
    for (size_t index = 1; index < corners.size(); ++index) {
      box_min_x = std::min(box_min_x, corners[index].x());
      box_max_x = std::max(box_max_x, corners[index].x());
      box_min_y = std::min(box_min_y, corners[index].y());
      box_max_y = std::max(box_max_y, corners[index].y());
    }
    const double half = 0.5 * map.resolution;
    const double along_radius = half * (std::abs(direction.x()) + std::abs(direction.y()));
    const double lateral_radius = half * (std::abs(normal.x()) + std::abs(normal.y()));
    for (const Eigen::Vector2d & center : occupied_centers) {
      // AABB 两个轴的快速排除。
      if (center.x() + half < box_min_x || center.x() - half > box_max_x ||
        center.y() + half < box_min_y || center.y() - half > box_max_y)
      {
        continue;
      }
      // 另外两个分离轴是 corridor 自身的纵向和横向轴。四轴均重叠时，
      // 有向 corridor 与占据栅格方块相交。
      const double cell_along = direction.dot(center);
      if (cell_along + along_radius < along_min || cell_along - along_radius > along_max) {
        continue;
      }
      const double cell_lateral = normal.dot(center);
      if (cell_lateral + lateral_radius < lateral_min ||
        cell_lateral - lateral_radius > lateral_max)
      {
        continue;
      }
      return false;
    }
    return true;
  }
};

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
    const LocalObstacleSet local_obstacles(
      map, start, end, options.obstacle_search_radius);
    if (!local_obstacles.isFree(0.0, 0.0, left_width, right_width))
    {
      reason = "seed_not_free";
      return false;
    }
    if (local_obstacles.isFree(
        initial_extension, backward_extension, left_width, right_width))
    {
      forward_extension = initial_extension;
    }
    if (local_obstacles.isFree(
        forward_extension, initial_extension, left_width, right_width))
    {
      backward_extension = initial_extension;
    }
    for (int iteration = 0; iteration < options.firi_iterations; ++iteration) {
      const double proposed_left = std::min(
        options.obstacle_search_radius,
        std::max(left_width * 2.0, left_width + map.resolution));
      if (local_obstacles.isFree(
          forward_extension, backward_extension, proposed_left, right_width))
      {
        left_width = proposed_left;
      }
      const double proposed_right = std::min(
        options.obstacle_search_radius,
        std::max(right_width * 2.0, right_width + map.resolution));
      if (local_obstacles.isFree(
          forward_extension, backward_extension, left_width, proposed_right))
      {
        right_width = proposed_right;
      }
      const double proposed_forward = std::min(
        options.obstacle_search_radius, forward_extension + map.resolution);
      if (local_obstacles.isFree(
          proposed_forward, backward_extension, left_width, right_width))
      {
        forward_extension = proposed_forward;
      }
      const double proposed_backward = std::min(
        options.obstacle_search_radius, backward_extension + map.resolution);
      if (local_obstacles.isFree(
          forward_extension, proposed_backward, left_width, right_width))
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
      return local_obstacles.isFree(
        forward_extension, backward_extension, candidate, right_width);
    });
    refine_extent(right_width, [&](double candidate) {
      return local_obstacles.isFree(
        forward_extension, backward_extension, left_width, candidate);
    });
    refine_extent(forward_extension, [&](double candidate) {
      return local_obstacles.isFree(candidate, backward_extension, left_width, right_width);
    });
    refine_extent(backward_extension, [&](double candidate) {
      return local_obstacles.isFree(forward_extension, candidate, left_width, right_width);
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
