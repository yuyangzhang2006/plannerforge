#include "gcopter/path_search.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>










// ============================================================
// TODO(阶段② 前端搜索)
// 在这里实现并改进路径搜索算法、代价函数和邻接扩展方式。
// 当前基线使用 Dijkstra，is_omni 控制四邻接或八邻接。
// ============================================================










bool PathSearch::search(
  const GridMap2D & map,
  const Eigen::Vector2d & start,
  const Eigen::Vector2d & goal,
  bool is_omni,
  std::vector<Eigen::Vector2d> & path)
{
  path.clear();

  if (!map.valid() || !start.allFinite() || !goal.allFinite()) {
    return false;
  }
  if ((start - goal).squaredNorm() <= 0.0) {
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
    path = {start, goal};
    return true;
  }

  struct QueueNode
  {
    double cost;
    int index;
  };

  // 优先队列按累计路径代价从小到大取出节点。
  const auto queue_compare = [](const QueueNode & lhs, const QueueNode & rhs) {
      if (lhs.cost != rhs.cost) {
        return lhs.cost > rhs.cost;
      }
      return lhs.index > rhs.index;
    };

  const size_t cell_count =
    static_cast<size_t>(map.width) * static_cast<size_t>(map.height);
  std::vector<double> distance(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(cell_count, -1);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<QueueNode, std::vector<QueueNode>, decltype(queue_compare)> open(
    queue_compare);

  const int start_index = map.index(start_grid);
  const int goal_index = map.index(goal_grid);
  distance[static_cast<size_t>(start_index)] = 0.0;
  open.push({0.0, start_index});

  const std::array<Eigen::Vector2i, 8> directions = {
    Eigen::Vector2i(1, 0), Eigen::Vector2i(-1, 0),
    Eigen::Vector2i(0, 1), Eigen::Vector2i(0, -1),
    Eigen::Vector2i(1, 1), Eigen::Vector2i(1, -1),
    Eigen::Vector2i(-1, 1), Eigen::Vector2i(-1, -1)};
  const int direction_count = is_omni ? 8 : 4;

  size_t closed_count = 0;
  while (!open.empty() && closed_count < cell_count) {
    const QueueNode current = open.top();
    open.pop();
    const size_t current_index = static_cast<size_t>(current.index);
    if (closed[current_index] || current.cost > distance[current_index]) {
      continue;
    }

    closed[current_index] = true;
    ++closed_count;
    if (current.index == goal_index) {
      break;
    }

    const Eigen::Vector2i current_grid(
      current.index % map.width, current.index / map.width);
    for (int direction_index = 0; direction_index < direction_count; ++direction_index) {
      const Eigen::Vector2i step = directions[static_cast<size_t>(direction_index)];
      const Eigen::Vector2i next_grid = current_grid + step;
      if (!map.isInside(next_grid) || map.isOccupied(next_grid)) {
        continue;
      }

      const bool diagonal = step.x() != 0 && step.y() != 0;
      if (diagonal) {
        // 对角移动需要同时满足两个相邻正交栅格可通行。
        const Eigen::Vector2i horizontal(current_grid.x() + step.x(), current_grid.y());
        const Eigen::Vector2i vertical(current_grid.x(), current_grid.y() + step.y());
        if (map.isOccupied(horizontal) || map.isOccupied(vertical)) {
          continue;
        }
      }

      const int next_index = map.index(next_grid);
      if (closed[static_cast<size_t>(next_index)]) {
        continue;
      }
      const double step_cost = diagonal ?
        std::sqrt(2.0) * map.resolution : map.resolution;
      const double new_cost = current.cost + step_cost;
      if (new_cost < distance[static_cast<size_t>(next_index)]) {
        distance[static_cast<size_t>(next_index)] = new_cost;
        parent[static_cast<size_t>(next_index)] = current.index;
        open.push({new_cost, next_index});
      }
    }
  }

  if (!closed[static_cast<size_t>(goal_index)]) {
    return false;
  }

  // parent 记录搜索树，回溯后得到从起点到终点的栅格序列。
  std::vector<int> reverse_indices;
  reverse_indices.reserve(cell_count);
  int current_index = goal_index;
  while (current_index != start_index && reverse_indices.size() < cell_count) {
    reverse_indices.push_back(current_index);
    current_index = parent[static_cast<size_t>(current_index)];
    if (current_index < 0) {
      path.clear();
      return false;
    }
  }
  if (current_index != start_index) {
    return false;
  }
  reverse_indices.push_back(start_index);
  std::reverse(reverse_indices.begin(), reverse_indices.end());

  path.reserve(reverse_indices.size());
  for (const int index : reverse_indices) {
    const Eigen::Vector2i grid(index % map.width, index / map.width);
    path.push_back(map.gridToWorld(grid));
  }
  if (path.size() < 2) {
    path.clear();
    return false;
  }
  path.front() = start;
  path.back() = goal;
  if (!std::all_of(path.begin(), path.end(), [](const Eigen::Vector2d & point) {
      return point.allFinite();
    }))
  {
    path.clear();
    return false;
  }
  return true;
}
