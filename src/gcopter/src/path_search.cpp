#include "gcopter/path_search.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

void PathSearch::prepare(std::size_t cell_count)
{
  constexpr size_t kDirectionStates = 9U;
  if (cell_count > std::numeric_limits<size_t>::max() / kDirectionStates) {return;}
  const size_t state_count = cell_count * kDirectionStates;
  if (nodes_.size() != state_count) {
    nodes_.clear();
    nodes_.resize(state_count);
    generation_ = 0U;
  }
  open_heap_.clear();
  open_heap_.reserve(std::min(state_count, cell_count * 2U));
}

bool PathSearch::search(
  const GridMap2D & map,
  const Eigen::Vector2d & start,
  const Eigen::Vector2d & goal,
  bool is_omni,
  const Options & options,
  std::vector<Eigen::Vector2d> & path,
  std::string * failure_reason)
{
  path.clear();
  if (failure_reason != nullptr) {failure_reason->clear();}
  if (!map.valid() || !start.allFinite() || !goal.allFinite() ||
    !std::isfinite(options.turn_cost_weight) || options.turn_cost_weight < 0.0 ||
    !std::isfinite(options.safety_cost_weight) || options.safety_cost_weight < 0.0 ||
    !std::isfinite(options.safety_cost_distance) || options.safety_cost_distance < 0.0 ||
    !std::isfinite(options.special_region_cost_weight) ||
    options.special_region_cost_weight < 0.0)
  {
    if (failure_reason != nullptr) {*failure_reason = "invalid_input";}
    return false;
  }

  const Eigen::Vector2i start_grid = map.worldToGrid(start);
  const Eigen::Vector2i goal_grid = map.worldToGrid(goal);
  if (!map.isInside(start_grid)) {
    if (failure_reason != nullptr) {*failure_reason = "start_outside_map";}
    return false;
  }
  if (!map.isInside(goal_grid)) {
    if (failure_reason != nullptr) {*failure_reason = "goal_outside_map";}
    return false;
  }
  if (map.isOccupied(start_grid)) {
    if (failure_reason != nullptr) {*failure_reason = "start_occupied";}
    return false;
  }
  if (map.isOccupied(goal_grid)) {
    if (failure_reason != nullptr) {*failure_reason = "goal_occupied";}
    return false;
  }
  if (start_grid == goal_grid) {
    if ((start - goal).norm() <= 1.0e-9) {
      if (failure_reason != nullptr) {*failure_reason = "identical_start_goal";}
      return false;
    }
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

  const size_t state_count = map.cellCount() * kDirectionStates;
  if (nodes_.size() != state_count) {
    prepare(map.cellCount());
  }
  if (++generation_ == 0U) {
    for (SearchNode & node : nodes_) {node.generation = 0U;}
    generation_ = 1U;
  }
  open_heap_.clear();

  const auto lessState = [&](int lhs, int rhs) {
      const SearchNode & a = nodes_[static_cast<size_t>(lhs)];
      const SearchNode & b = nodes_[static_cast<size_t>(rhs)];
      if (a.f != b.f) {return a.f < b.f;}
      if (a.g != b.g) {return a.g < b.g;}
      return lhs < rhs;
    };
  const auto swapHeap = [&](int lhs, int rhs) {
      std::swap(open_heap_[static_cast<size_t>(lhs)], open_heap_[static_cast<size_t>(rhs)]);
      nodes_[static_cast<size_t>(open_heap_[static_cast<size_t>(lhs)])].heap_index = lhs;
      nodes_[static_cast<size_t>(open_heap_[static_cast<size_t>(rhs)])].heap_index = rhs;
    };
  const auto siftUp = [&](int initial) {
      int index = initial;
      while (index > 0) {
        const int parent_index = (index - 1) / 2;
        if (!lessState(open_heap_[static_cast<size_t>(index)],
            open_heap_[static_cast<size_t>(parent_index)])) {break;}
        swapHeap(index, parent_index);
        index = parent_index;
      }
    };
  const auto pushOrDecrease = [&](int state) {
      SearchNode & node = nodes_[static_cast<size_t>(state)];
      if (node.heap_index < 0) {
        node.heap_index = static_cast<int>(open_heap_.size());
        open_heap_.push_back(state);
      }
      siftUp(node.heap_index);
    };
  const auto popMinimum = [&]() {
      const int result = open_heap_.front();
      swapHeap(0, static_cast<int>(open_heap_.size()) - 1);
      open_heap_.pop_back();
      nodes_[static_cast<size_t>(result)].heap_index = -1;
      int index = 0;
      while (true) {
        const int left = 2 * index + 1;
        if (left >= static_cast<int>(open_heap_.size())) {break;}
        const int right = left + 1;
        int smallest = left;
        if (right < static_cast<int>(open_heap_.size()) &&
          lessState(open_heap_[static_cast<size_t>(right)],
          open_heap_[static_cast<size_t>(left)]))
        {
          smallest = right;
        }
        if (!lessState(open_heap_[static_cast<size_t>(smallest)],
            open_heap_[static_cast<size_t>(index)])) {break;}
        swapHeap(index, smallest);
        index = smallest;
      }
      return result;
    };

  const auto initializeNode = [&](int state) -> SearchNode & {
      SearchNode & node = nodes_[static_cast<size_t>(state)];
      if (node.generation != generation_) {
        node.generation = generation_;
        node.g = std::numeric_limits<double>::infinity();
        node.f = std::numeric_limits<double>::infinity();
        node.parent = -1;
        node.heap_index = -1;
        node.closed = false;
      }
      return node;
    };

  const int start_state = stateIndex(map.index(start_grid), kStartDirection);
  SearchNode & start_node = initializeNode(start_state);
  start_node.g = 0.0;
  start_node.f = octile(start_grid);
  pushOrDecrease(start_state);
  int goal_state = -1;

  while (!open_heap_.empty()) {
    const int current_state = popMinimum();
    SearchNode & current = nodes_[static_cast<size_t>(current_state)];
    current.closed = true;
    const int current_cell = current_state / kDirectionStates;
    const int incoming = current_state % kDirectionStates;
    if (current_cell == map.index(goal_grid)) {
      goal_state = current_state;
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
      SearchNode & next = initializeNode(next_state);
      if (!next.closed && next_g + 1.0e-12 < next.g) {
        next.g = next_g;
        next.f = next_g + octile(next_grid);
        next.parent = current_state;
        pushOrDecrease(next_state);
      }
    }
  }
  if (goal_state < 0) {
    if (failure_reason != nullptr) {*failure_reason = "search_exhausted";}
    return false;
  }

  std::vector<int> reverse_cells;
  for (int state = goal_state; state >= 0;
    state = nodes_[static_cast<size_t>(state)].parent)
  {
    reverse_cells.push_back(state / kDirectionStates);
    if (state == start_state) {break;}
    if (reverse_cells.size() > state_count) {
      if (failure_reason != nullptr) {*failure_reason = "parent_cycle";}
      return false;
    }
  }
  if (reverse_cells.empty() || reverse_cells.back() != map.index(start_grid)) {
    if (failure_reason != nullptr) {*failure_reason = "reconstruction_failed";}
    return false;
  }
  std::reverse(reverse_cells.begin(), reverse_cells.end());
  path.reserve(reverse_cells.size());
  for (const int cell : reverse_cells) {
    path.push_back(map.gridToWorld({cell % map.width, cell / map.width}));
  }
  path.front() = start;
  path.back() = goal;
  return path.size() >= 2;
}
