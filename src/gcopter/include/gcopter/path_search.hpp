#pragma once

#include "gcopter/grid_map_2d.hpp"

#include <Eigen/Eigen>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// 前端路径搜索器，根据地图和起终点生成离散路径。
class PathSearch
{
private:
  struct SearchNode
  {
    double g = 0.0;
    double f = 0.0;
    int parent = -1;
    int heap_index = -1;
    uint32_t generation = 0U;
    bool closed = false;
  };

  // 搜索内存跨调用复用；generation 避免每次为整张地图清空数百万个方向状态。
  std::vector<SearchNode> nodes_;
  std::vector<int> open_heap_;
  uint32_t generation_ = 0U;

public:
  struct Options
  {
    double turn_cost_weight = 0.0;
    double safety_cost_weight = 0.0;
    double safety_cost_distance = 0.0;
    double special_region_cost_weight = 0.0;
  };

  // 地图就绪时预分配搜索内存，避免用户第一次点选目标时承担大块内存初始化。
  void prepare(std::size_t cell_count);

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
