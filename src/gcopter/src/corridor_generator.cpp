#include "gcopter/corridor_generator.hpp"

#include <algorithm>
#include <cmath>

namespace
{

ConvexCorridor2D makeBox(
  const Eigen::Vector2d & start, const Eigen::Vector2d & end,
  double extension, double half_width)
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
  result.b(0) = direction.dot(end) + extension;
  result.b(1) = -direction.dot(start) + extension;
  result.b(2) = std::max(normal.dot(start), normal.dot(end)) + half_width;
  result.b(3) = -std::min(normal.dot(start), normal.dot(end)) + half_width;
  return result;
}

bool boxIsFree(
  const GridMap2D & map, const Eigen::Vector2d & start, const Eigen::Vector2d & end,
  double extension, double half_width)
{
  const double length = (end - start).norm();
  const Eigen::Vector2d direction = (end - start) / length;
  const Eigen::Vector2d normal(-direction.y(), direction.x());
  const double spacing = std::max(0.25 * map.resolution, 1.0e-3);
  const int longitudinal = std::max(1, static_cast<int>(std::ceil((length + 2.0 * extension) / spacing)));
  const int lateral = std::max(1, static_cast<int>(std::ceil(2.0 * half_width / spacing)));
  for (int i = 0; i <= longitudinal; ++i) {
    const double along = -extension + (length + 2.0 * extension) *
      static_cast<double>(i) / static_cast<double>(longitudinal);
    for (int j = 0; j <= lateral; ++j) {
      const double across = -half_width + 2.0 * half_width *
        static_cast<double>(j) / static_cast<double>(lateral);
      if (map.isOccupied(map.worldToGrid(start + direction * along + normal * across))) {
        return false;
      }
    }
  }
  return true;
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

    double half_width = initial_width;
    double extension = initial_extension;
    if (!boxIsFree(map, start, end, 0.0, initial_width)) {
      reason = "seed_not_free";
      return false;
    }
    if (!boxIsFree(map, start, end, extension, half_width)) {
      extension = 0.0;
    }
    for (int iteration = 0; iteration < options.firi_iterations; ++iteration) {
      const double proposed_width = std::min(
        options.obstacle_search_radius, std::max(half_width * 2.0, half_width + map.resolution));
      if (boxIsFree(map, start, end, extension, proposed_width)) {
        half_width = proposed_width;
      }
      const double proposed_extension = std::min(
        options.obstacle_search_radius, extension + map.resolution);
      if (boxIsFree(map, start, end, proposed_extension, half_width)) {
        extension = proposed_extension;
      }
    }
    ConvexCorridor2D corridor = makeBox(start, end, extension, half_width);
    if (!corridor.contains(start) || !corridor.contains(end)) {
      reason = "seed_outside_corridor";
      return false;
    }
    corridors.push_back(std::move(corridor));
  }

  // The overlap seed is a disk represented by four axis points. This rejects
  // point-only contact while keeping the test independent of corridor orientation.
  const double overlap_radius = 0.5 * options.minimum_overlap;
  for (size_t joint = 1; joint < sparse_path.size() - 1 && overlap_radius > 0.0; ++joint) {
    const Eigen::Vector2d center = sparse_path[joint];
    const Eigen::Vector2d offsets[4] = {
      {overlap_radius, 0.0}, {-overlap_radius, 0.0},
      {0.0, overlap_radius}, {0.0, -overlap_radius}};
    for (const Eigen::Vector2d & offset : offsets) {
      if (!corridors[joint - 1].contains(center + offset, options.merge_tolerance) ||
        !corridors[joint].contains(center + offset, options.merge_tolerance))
      {
        reason = "insufficient_overlap";
        return false;
      }
    }
  }
  return true;
}
