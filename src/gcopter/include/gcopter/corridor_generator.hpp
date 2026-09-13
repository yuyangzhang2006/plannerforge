#pragma once

#include "gcopter/grid_map_2d.hpp"

#include <Eigen/Eigen>

#include <string>
#include <vector>

struct ConvexCorridor2D
{
  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  bool valid() const;
  bool contains(const Eigen::Vector2d & point, double tolerance = 1.0e-8) const;
};

struct CorridorOptions
{
  double obstacle_search_radius = 1.5;
  int firi_iterations = 4;
  double minimum_overlap = 0.08;
  double merge_tolerance = 1.0e-4;
};

bool GenerateCorridor(
  const GridMap2D & map,
  const std::vector<Eigen::Vector2d> & sparse_path,
  const CorridorOptions & options,
  std::vector<ConvexCorridor2D> & corridors,
  std::string & reason);
