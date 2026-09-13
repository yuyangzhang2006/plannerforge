#include "gcopter/local_replan.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

double routeLength(const std::vector<Eigen::Vector2d> & route)
{
  double length = 0.0;
  for (size_t index = 1; index < route.size(); ++index) {
    length += (route[index] - route[index - 1]).norm();
  }
  return length;
}

bool validOptions(const LocalReplanOptions & options)
{
  return std::isfinite(options.minimum_lookahead_distance) &&
    std::isfinite(options.maximum_lookahead_distance) &&
    std::isfinite(options.maximum_local_path_distance) &&
    std::isfinite(options.maximum_extra_distance) &&
    options.minimum_lookahead_distance >= 0.0 &&
    options.maximum_lookahead_distance >= options.minimum_lookahead_distance &&
    options.maximum_local_path_distance > 0.0 && options.maximum_extra_distance >= 0.0;
}

}  // namespace

bool BuildLocalRejoinRoute(
  const GridMap2D & map,
  const std::vector<Eigen::Vector2d> & active_route,
  const Eigen::Vector2d & current_position,
  bool is_omni,
  const PathSearch::Options & search_options,
  const LocalReplanOptions & options,
  std::vector<Eigen::Vector2d> & output,
  size_t & rejoin_index,
  std::string & reason)
{
  output.clear();
  rejoin_index = 0;
  reason.clear();
  if (!map.valid() || active_route.size() < 2 || !current_position.allFinite() ||
    !validOptions(options))
  {
    reason = "invalid_input";
    return false;
  }

  size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < active_route.size(); ++index) {
    if (!active_route[index].allFinite()) {
      reason = "nonfinite_active_route";
      return false;
    }
    const double distance = (active_route[index] - current_position).squaredNorm();
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = index;
    }
  }

  double lookahead = 0.0;
  PathSearch search;
  std::string search_reason;
  for (size_t candidate = nearest_index + 1; candidate < active_route.size(); ++candidate) {
    lookahead += (active_route[candidate] - active_route[candidate - 1]).norm();
    if (lookahead + 1.0e-9 < options.minimum_lookahead_distance) {continue;}
    if (lookahead > options.maximum_lookahead_distance + 1.0e-9) {break;}
    if (map.isOccupied(map.worldToGrid(active_route[candidate]))) {continue;}

    std::vector<Eigen::Vector2d> local_route;
    if (!search.search(
        map, current_position, active_route[candidate], is_omni,
        search_options, local_route, &search_reason))
    {
      continue;
    }
    const double local_length = routeLength(local_route);
    const double direct_distance = (active_route[candidate] - current_position).norm();
    if (local_length > options.maximum_local_path_distance + 1.0e-9 ||
      local_length - direct_distance > options.maximum_extra_distance + 1.0e-9)
    {
      continue;
    }

    output = std::move(local_route);
    output.reserve(output.size() + active_route.size() - candidate - 1);
    for (size_t suffix = candidate + 1; suffix < active_route.size(); ++suffix) {
      if ((active_route[suffix] - output.back()).norm() > 1.0e-8) {
        output.push_back(active_route[suffix]);
      }
    }
    rejoin_index = candidate;
    return output.size() >= 2;
  }

  reason = search_reason.empty() ? "no_rejoin_within_absolute_limits" : search_reason;
  return false;
}
