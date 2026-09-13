#pragma once

#include "gcopter/corridor_generator.hpp"

#include <Eigen/Eigen>

#include <vector>

// 轨迹生成、后端优化和连续轨迹解算之间使用的数据结构。
struct MincoTrajectoryData
{
  // 一次狗洞穿越对应的原始相交时间和扩展生效时间。
  struct CrossHoleInterval
  {
    double enter_time = 0.0;
    double exit_time = 0.0;
    double active_start_time = 0.0;
    double active_end_time = 0.0;
    int black_pixel_count = 0;
  };

  Eigen::Matrix<double, 2, 3> start_pva;  // 起点 position、velocity、acceleration
  Eigen::Matrix<double, 2, 3> end_pva;    // 终点 position、velocity、acceleration
  int segment_count = 0;                  // 轨迹分段数
  Eigen::Matrix2Xd intermediate_positions;// 内部连接点，列数为 segment_count - 1
  Eigen::VectorXd segment_durations;      // 每段持续时间，单位 s
  Eigen::VectorXi waypoint_is_spacious;   // 内部连接点的空间标志占位
  std::vector<CrossHoleInterval> cross_hole_intervals;
  std::vector<ConvexCorridor2D> corridors;

  // 创建空轨迹数据。
  MincoTrajectoryData()
  {
    clear();
  }

  // 清空所有轨迹字段。
  void clear()
  {
    start_pva.setZero();
    end_pva.setZero();
    segment_count = 0;
    intermediate_positions.resize(2, 0);
    segment_durations.resize(0);
    waypoint_is_spacious.resize(0);
    cross_hole_intervals.clear();
    corridors.clear();
  }

  // 设置首尾 PVA 和轨迹分段数。
  void setBoundaryConditions(
    const Eigen::Matrix<double, 2, 3> & start,
    const Eigen::Matrix<double, 2, 3> & end,
    int segments)
  {
    start_pva = start;
    end_pva = end;
    segment_count = segments;
  }

  // 设置内部连接点与每段持续时间。
  void setIntermediatePositionsAndDurations(
    const Eigen::Matrix2Xd & positions,
    const Eigen::VectorXd & durations)
  {
    intermediate_positions = positions;
    segment_durations = durations;
    waypoint_is_spacious.resize(0);
    cross_hole_intervals.clear();
  }
};
