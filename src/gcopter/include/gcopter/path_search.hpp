#pragma once

#include "gcopter/grid_map_2d.hpp"

#include <Eigen/Eigen>

#include <string>
#include <vector>

// 前端路径搜索器，根据地图和起终点生成离散路径。
class PathSearch
{
public:
  struct Options
  {
    double turn_cost_weight = 0.0;
    double safety_cost_weight = 0.0;
    double safety_cost_distance = 0.0;
    double special_region_cost_weight = 0.0;
  };

  // 输入和输出均使用 map 坐标，成功路径包含准确的起点和终点。
  bool search(
    const GridMap2D & map,
    const Eigen::Vector2d & start,
    const Eigen::Vector2d & goal,
    bool is_omni,
    const Options & options,
    std::vector<Eigen::Vector2d> & path,
    std::string * failure_reason = nullptr);
};
