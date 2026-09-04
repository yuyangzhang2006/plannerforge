#pragma once

#include "gcopter/grid_map_2d.hpp"

#include <Eigen/Eigen>

#include <vector>

// 前端路径搜索器，根据地图和起终点生成离散路径。
class PathSearch
{
public:
  // 输入和输出均使用 map 坐标，成功路径包含准确的起点和终点。
  bool search(
    const GridMap2D & map,
    const Eigen::Vector2d & start,
    const Eigen::Vector2d & goal,
    bool is_omni,
    std::vector<Eigen::Vector2d> & path);
};
