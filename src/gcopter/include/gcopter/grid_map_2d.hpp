#pragma once

#include <Eigen/Eigen>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

// 规划器内部使用的二维栅格地图，保存尺寸、坐标原点和占据数据。
struct GridMap2D
{
  std::string frame_id = "map";                    // 当前基线只接受 map
  double resolution = 0.0;                         // 单位 m/cell
  Eigen::Vector2d origin = Eigen::Vector2d::Zero(); // 左下角栅格边界的 map 坐标
  int width = 0;
  int height = 0;
  std::vector<uint8_t> data;                       // 0 可通行，非 0 障碍

  // 清空地图内容和元数据。
  void clear()
  {
    frame_id = "map";
    resolution = 0.0;
    origin.setZero();
    width = 0;
    height = 0;
    data.clear();
  }

  // 检查地图尺寸、分辨率和数据长度。
  bool valid() const
  {
    if (frame_id.empty() || !std::isfinite(resolution) || resolution <= 0.0 ||
      !origin.allFinite() || width <= 0 || height <= 0)
    {
      return false;
    }
    return data.size() == static_cast<size_t>(width) * static_cast<size_t>(height);
  }

  // 判断栅格索引是否位于地图范围内。
  bool isInside(const Eigen::Vector2i & grid) const
  {
    return grid.x() >= 0 && grid.x() < width && grid.y() >= 0 && grid.y() < height;
  }

  // 将二维栅格索引转换为 data 中的一维索引。
  int index(const Eigen::Vector2i & grid) const
  {
    return grid.x() + grid.y() * width;
  }

  // 查询栅格占据状态，地图外位置按占据处理。
  bool isOccupied(const Eigen::Vector2i & grid) const
  {
    return !isInside(grid) || data[static_cast<size_t>(index(grid))] != 0U;
  }

  // 将 map 坐标转换为所属栅格索引。
  Eigen::Vector2i worldToGrid(const Eigen::Vector2d & world) const
  {
    const Eigen::Vector2d continuous = (world - origin) / resolution;
    const double grid_x = std::floor(continuous.x());
    const double grid_y = std::floor(continuous.y());
    if (!std::isfinite(grid_x) || !std::isfinite(grid_y) ||
      grid_x < static_cast<double>(std::numeric_limits<int>::min()) ||
      grid_x > static_cast<double>(std::numeric_limits<int>::max()) ||
      grid_y < static_cast<double>(std::numeric_limits<int>::min()) ||
      grid_y > static_cast<double>(std::numeric_limits<int>::max()))
    {
      return Eigen::Vector2i::Constant(std::numeric_limits<int>::min());
    }
    return Eigen::Vector2i(static_cast<int>(grid_x), static_cast<int>(grid_y));
  }

  // 返回栅格中心在 map 坐标系中的位置。
  Eigen::Vector2d gridToWorld(const Eigen::Vector2i & grid) const
  {
    return origin + (grid.cast<double>() + Eigen::Vector2d::Constant(0.5)) * resolution;
  }
};
