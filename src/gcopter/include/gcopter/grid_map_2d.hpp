#pragma once

#include <Eigen/Eigen>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

enum class MapSemantic : uint8_t
{
  NORMAL = 0,
  CROSS_HOLE = 1,
  FORBIDDEN = 2,
};

// Planner-side derived map. The ROS interface remains OccupancyGrid; hard layers,
// clearance and semantics are derived when a map is received.
struct GridMap2D
{
  std::string frame_id = "map";
  double resolution = 0.0;
  Eigen::Vector2d origin = Eigen::Vector2d::Zero();
  int width = 0;
  int height = 0;
  std::vector<uint8_t> raw_occupancy;
  std::vector<uint8_t> occupancy;
  std::vector<uint8_t> compact_occupancy;
  std::vector<float> distance_field;
  std::vector<uint8_t> semantic;

  void clear()
  {
    frame_id = "map";
    resolution = 0.0;
    origin.setZero();
    width = 0;
    height = 0;
    raw_occupancy.clear();
    occupancy.clear();
    compact_occupancy.clear();
    distance_field.clear();
    semantic.clear();
  }

  size_t cellCount() const
  {
    return static_cast<size_t>(width) * static_cast<size_t>(height);
  }

  bool valid() const
  {
    return !frame_id.empty() && std::isfinite(resolution) && resolution > 0.0 &&
      origin.allFinite() && width > 0 && height > 0 &&
      raw_occupancy.size() == cellCount() && occupancy.size() == cellCount() &&
      compact_occupancy.size() == cellCount() && distance_field.size() == cellCount() &&
      semantic.size() == cellCount();
  }

  bool isInside(const Eigen::Vector2i & grid) const
  {
    return grid.x() >= 0 && grid.x() < width && grid.y() >= 0 && grid.y() < height;
  }

  int index(const Eigen::Vector2i & grid) const {return grid.x() + grid.y() * width;}

  bool isOccupied(const Eigen::Vector2i & grid) const
  {
    return !isInside(grid) || occupancy[static_cast<size_t>(index(grid))] != 0U;
  }

  bool isCompactOccupied(const Eigen::Vector2i & grid) const
  {
    return !isInside(grid) || compact_occupancy[static_cast<size_t>(index(grid))] != 0U;
  }

  MapSemantic semanticAt(const Eigen::Vector2i & grid) const
  {
    if (!isInside(grid)) {return MapSemantic::FORBIDDEN;}
    return static_cast<MapSemantic>(semantic[static_cast<size_t>(index(grid))]);
  }

  float clearanceAt(const Eigen::Vector2i & grid) const
  {
    if (!isInside(grid)) {return 0.0F;}
    return distance_field[static_cast<size_t>(index(grid))];
  }

  Eigen::Vector2i worldToGrid(const Eigen::Vector2d & world) const
  {
    const Eigen::Vector2d continuous = (world - origin) / resolution;
    const double gx = std::floor(continuous.x());
    const double gy = std::floor(continuous.y());
    if (!std::isfinite(gx) || !std::isfinite(gy) ||
      gx < static_cast<double>(std::numeric_limits<int>::min()) ||
      gx > static_cast<double>(std::numeric_limits<int>::max()) ||
      gy < static_cast<double>(std::numeric_limits<int>::min()) ||
      gy > static_cast<double>(std::numeric_limits<int>::max()))
    {
      return Eigen::Vector2i::Constant(std::numeric_limits<int>::min());
    }
    return Eigen::Vector2i(static_cast<int>(gx), static_cast<int>(gy));
  }

  Eigen::Vector2d gridToWorld(const Eigen::Vector2i & grid) const
  {
    return origin + (grid.cast<double>() + Eigen::Vector2d::Constant(0.5)) * resolution;
  }

  bool buildDerivedLayers(double normal_radius, double compact_radius)
  {
    if (width <= 0 || height <= 0 || !std::isfinite(resolution) || resolution <= 0.0 ||
      raw_occupancy.size() != cellCount() || semantic.size() != cellCount() ||
      !std::isfinite(normal_radius) || !std::isfinite(compact_radius) ||
      normal_radius < 0.0 || compact_radius < 0.0 || compact_radius > normal_radius)
    {
      return false;
    }
    constexpr double kFar = 1.0e15;
    const auto transform1d = [](const std::vector<double> & input) {
        const int count = static_cast<int>(input.size());
        std::vector<double> output(static_cast<size_t>(count));
        std::vector<int> sites(static_cast<size_t>(count));
        std::vector<double> boundaries(static_cast<size_t>(count + 1));
        int k = 0;
        sites[0] = 0;
        boundaries[0] = -std::numeric_limits<double>::infinity();
        boundaries[1] = std::numeric_limits<double>::infinity();
        for (int q = 1; q < count; ++q) {
          double separation = 0.0;
          do {
            const int site = sites[static_cast<size_t>(k)];
            separation = ((input[static_cast<size_t>(q)] + q * q) -
              (input[static_cast<size_t>(site)] + site * site)) /
              (2.0 * static_cast<double>(q - site));
            if (separation <= boundaries[static_cast<size_t>(k)]) {--k;}
          } while (k >= 0 && separation <= boundaries[static_cast<size_t>(k)]);
          ++k;
          sites[static_cast<size_t>(k)] = q;
          boundaries[static_cast<size_t>(k)] = separation;
          boundaries[static_cast<size_t>(k + 1)] = std::numeric_limits<double>::infinity();
        }
        k = 0;
        for (int q = 0; q < count; ++q) {
          while (boundaries[static_cast<size_t>(k + 1)] < q) {++k;}
          const double delta = q - sites[static_cast<size_t>(k)];
          output[static_cast<size_t>(q)] = delta * delta +
            input[static_cast<size_t>(sites[static_cast<size_t>(k)])];
        }
        return output;
      };
    std::vector<double> first_pass(cellCount(), kFar);
    std::vector<double> input_line(static_cast<size_t>(std::max(width, height)));
    for (int y = 0; y < height; ++y) {
      input_line.resize(static_cast<size_t>(width));
      for (int x = 0; x < width; ++x) {
        const size_t i = static_cast<size_t>(x + y * width);
        input_line[static_cast<size_t>(x)] =
          (raw_occupancy[i] != 0U ||
          static_cast<MapSemantic>(semantic[i]) == MapSemantic::FORBIDDEN) ? 0.0 : kFar;
      }
      const std::vector<double> transformed = transform1d(input_line);
      for (int x = 0; x < width; ++x) {
        first_pass[static_cast<size_t>(x + y * width)] = transformed[static_cast<size_t>(x)];
      }
    }
    distance_field.resize(cellCount());
    const double cell_boundary_offset = 0.5 * std::sqrt(2.0) * resolution;
    for (int x = 0; x < width; ++x) {
      input_line.resize(static_cast<size_t>(height));
      for (int y = 0; y < height; ++y) {
        input_line[static_cast<size_t>(y)] = first_pass[static_cast<size_t>(x + y * width)];
      }
      const std::vector<double> transformed = transform1d(input_line);
      for (int y = 0; y < height; ++y) {
        distance_field[static_cast<size_t>(x + y * width)] = static_cast<float>(
          std::max(0.0, std::sqrt(transformed[static_cast<size_t>(y)]) * resolution -
          cell_boundary_offset));
      }
    }
    occupancy.assign(cellCount(), 0U);
    compact_occupancy.assign(cellCount(), 0U);
    for (size_t i = 0; i < cellCount(); ++i) {
      const auto label = static_cast<MapSemantic>(semantic[i]);
      compact_occupancy[i] =
        (distance_field[i] <= compact_radius || label == MapSemantic::FORBIDDEN) ? 100U : 0U;
      const double radius = label == MapSemantic::CROSS_HOLE ? compact_radius : normal_radius;
      occupancy[i] =
        (distance_field[i] <= radius || label == MapSemantic::FORBIDDEN) ? 100U : 0U;
    }
    return true;
  }
};
