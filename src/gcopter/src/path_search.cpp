#include "gcopter/path_search.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

bool PathSearch::search(
  const GridMap2D & map,
  const Eigen::Vector2d & start,
  const Eigen::Vector2d & goal,
  bool is_omni,
  const Options & options,
  std::vector<Eigen::Vector2d> & path)
{
  path.clear();
  if (!map.valid() || !start.allFinite() || !goal.allFinite() ||
    !std::isfinite(options.turn_cost_weight) || options.turn_cost_weight < 0.0 ||
    !std::isfinite(options.safety_cost_weight) || options.safety_cost_weight < 0.0 ||
    !std::isfinite(options.safety_cost_distance) || options.safety_cost_distance < 0.0 ||
    !std::isfinite(options.special_region_cost_weight) ||
    options.special_region_cost_weight < 0.0)
  {
    return false;
  }

  const Eigen::Vector2i start_grid = map.worldToGrid(start);
  const Eigen::Vector2i goal_grid = map.worldToGrid(goal);
  if (!map.isInside(start_grid) || !map.isInside(goal_grid) ||
    map.isOccupied(start_grid) || map.isOccupied(goal_grid))
  {
    return false;
  }
  if (start_grid == goal_grid) {
    if ((start - goal).norm() <= 1.0e-9) {return false;}
    path = {start, goal};
    return true;
  }

  const std::array<Eigen::Vector2i, 8> directions = {
    Eigen::Vector2i(1, 0), Eigen::Vector2i(1, 1), Eigen::Vector2i(0, 1),
    Eigen::Vector2i(-1, 1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(-1, -1),
    Eigen::Vector2i(0, -1), Eigen::Vector2i(1, -1)};
  const int direction_count = is_omni ? 8 : 4;
  // Four-connected mode uses the cardinal entries from the same angular ordering.
  const std::array<int, 8> active_direction = is_omni ?
    std::array<int, 8>{0, 1, 2, 3, 4, 5, 6, 7} :
    std::array<int, 8>{0, 2, 4, 6, 0, 0, 0, 0};

  constexpr int kStartDirection = 8;
  constexpr int kDirectionStates = 9;
  const auto stateIndex = [](int cell, int direction) {
      return cell * kDirectionStates + direction;
    };
  const auto octile = [&](const Eigen::Vector2i & cell) {
      const int dx = std::abs(cell.x() - goal_grid.x());
      const int dy = std::abs(cell.y() - goal_grid.y());
      return map.resolution *
             (static_cast<double>(std::max(dx, dy)) +
             (std::sqrt(2.0) - 1.0) * static_cast<double>(std::min(dx, dy)));
    };

  struct QueueNode {double f; double g; int state;};
  const auto compare = [](const QueueNode & lhs, const QueueNode & rhs) {
      if (lhs.f != rhs.f) {return lhs.f > rhs.f;}
      if (lhs.g != rhs.g) {return lhs.g > rhs.g;}
      return lhs.state > rhs.state;
    };
  const size_t state_count = map.cellCount() * kDirectionStates;
  std::vector<double> best(state_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(state_count, -1);
  std::vector<uint8_t> closed(state_count, 0U);
  std::priority_queue<QueueNode, std::vector<QueueNode>, decltype(compare)> open(compare);

  const int start_state = stateIndex(map.index(start_grid), kStartDirection);
  best[static_cast<size_t>(start_state)] = 0.0;
  open.push({octile(start_grid), 0.0, start_state});
  int goal_state = -1;

  while (!open.empty()) {
    const QueueNode current = open.top();
    open.pop();
    if (closed[static_cast<size_t>(current.state)] != 0U ||
      current.g > best[static_cast<size_t>(current.state)] + 1.0e-12)
    {
      continue;
    }
    closed[static_cast<size_t>(current.state)] = 1U;
    const int current_cell = current.state / kDirectionStates;
    const int incoming = current.state % kDirectionStates;
    if (current_cell == map.index(goal_grid)) {
      goal_state = current.state;
      break;
    }
    const Eigen::Vector2i current_grid(current_cell % map.width, current_cell / map.width);
    for (int active = 0; active < direction_count; ++active) {
      const int direction = active_direction[static_cast<size_t>(active)];
      const Eigen::Vector2i step = directions[static_cast<size_t>(direction)];
      const Eigen::Vector2i next_grid = current_grid + step;
      if (map.isOccupied(next_grid)) {continue;}
      const bool diagonal = step.x() != 0 && step.y() != 0;
      if (diagonal && (map.isOccupied({current_grid.x() + step.x(), current_grid.y()}) ||
        map.isOccupied({current_grid.x(), current_grid.y() + step.y()})))
      {
        continue;
      }

      const double length = map.resolution * (diagonal ? std::sqrt(2.0) : 1.0);
      double turn = 0.0;
      if (incoming != kStartDirection) {
        int delta = std::abs(direction - incoming);
        delta = std::min(delta, 8 - delta);
        turn = options.turn_cost_weight * static_cast<double>(delta) *
          (std::acos(-1.0) / 4.0);
      }
      double safety = 0.0;
      if (options.safety_cost_weight > 0.0 && options.safety_cost_distance > 0.0) {
        const double clearance = map.clearanceAt(next_grid);
        if (clearance < options.safety_cost_distance) {
          const double ratio = 1.0 - clearance / options.safety_cost_distance;
          safety = options.safety_cost_weight * length * ratio * ratio;
        }
      }
      const double semantic = map.semanticAt(next_grid) == MapSemantic::CROSS_HOLE ?
        options.special_region_cost_weight * length : 0.0;
      const double next_g = current.g + length + turn + safety + semantic;
      const int next_state = stateIndex(map.index(next_grid), direction);
      if (next_g + 1.0e-12 < best[static_cast<size_t>(next_state)]) {
        best[static_cast<size_t>(next_state)] = next_g;
        parent[static_cast<size_t>(next_state)] = current.state;
        open.push({next_g + octile(next_grid), next_g, next_state});
      }
    }
  }
  if (goal_state < 0) {return false;}

  std::vector<int> reverse_cells;
  for (int state = goal_state; state >= 0; state = parent[static_cast<size_t>(state)]) {
    reverse_cells.push_back(state / kDirectionStates);
    if (state == start_state) {break;}
    if (reverse_cells.size() > state_count) {return false;}
  }
  if (reverse_cells.empty() || reverse_cells.back() != map.index(start_grid)) {return false;}
  std::reverse(reverse_cells.begin(), reverse_cells.end());
  path.reserve(reverse_cells.size());
  for (const int cell : reverse_cells) {
    path.push_back(map.gridToWorld({cell % map.width, cell / map.width}));
  }
  path.front() = start;
  path.back() = goal;
  return path.size() >= 2;
}
