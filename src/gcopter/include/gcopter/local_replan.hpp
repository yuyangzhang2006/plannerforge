#pragma once

#include "gcopter/grid_map_2d.hpp"
#include "gcopter/path_search.hpp"

#include <Eigen/Eigen>

#include <cstddef>
#include <string>
#include <vector>

struct LocalReplanOptions
{
  double minimum_lookahead_distance = 1.5;
  double maximum_lookahead_distance = 6.0;
  double maximum_local_path_distance = 9.0;
  double maximum_extra_distance = 3.0;
};

// 从当前位置局部搜索到原路径前方的接入点，再拼接未改变拓扑的原路径后缀。
bool BuildLocalRejoinRoute(
  const GridMap2D & map,
  const std::vector<Eigen::Vector2d> & active_route,
  const Eigen::Vector2d & current_position,
  bool is_omni,
  const PathSearch::Options & search_options,
  const LocalReplanOptions & options,
  std::vector<Eigen::Vector2d> & output,
  size_t & rejoin_index,
  std::string & reason);
