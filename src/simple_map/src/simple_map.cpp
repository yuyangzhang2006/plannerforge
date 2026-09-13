#include "simple_map/simple_map.hpp"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <utility>

namespace simple_map
{

namespace
{

// 检查数组参数是否都为有限值。
bool finiteVector(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

enum class MotionPattern
{
  HORIZONTAL,
  VERTICAL,
  DIAGONAL,
  ELLIPSE,
};

// 描述一个轴对齐矩形及其确定性运动轨迹。比例值均相对于当前地图尺寸，
// 因此同一测试场景可以适配不同分辨率的地图。
struct MovingRectangle
{
  double width_m;
  double height_m;
  double speed_mps;
  double center_x_ratio;
  double center_y_ratio;
  double phase_ratio;
  double span_x_ratio;
  double span_y_ratio;
  MotionPattern motion_pattern;
};

// 压力测试场景：横向、纵向、对角和椭圆运动混合，速度、尺寸和相位互不相同。
// 数量保持为确定的 12 个，便于复现实验与比较规划耗时。
constexpr std::array<MovingRectangle, 12> kMovingRectangles{{
  {0.75, 0.55, 0.85, 0.50, 0.18, 0.00, 0.0, 0.0, MotionPattern::HORIZONTAL},
  {0.55, 0.80, 0.62, 0.50, 0.38, 0.23, 0.0, 0.0, MotionPattern::HORIZONTAL},
  {0.90, 0.60, 1.05, 0.50, 0.61, 0.49, 0.0, 0.0, MotionPattern::HORIZONTAL},
  {0.65, 0.65, 0.74, 0.50, 0.82, 0.76, 0.0, 0.0, MotionPattern::HORIZONTAL},
  {0.60, 0.90, 0.70, 0.17, 0.50, 0.12, 0.0, 0.0, MotionPattern::VERTICAL},
  {0.80, 0.55, 0.92, 0.47, 0.50, 0.43, 0.0, 0.0, MotionPattern::VERTICAL},
  {0.55, 0.75, 0.58, 0.79, 0.50, 0.71, 0.0, 0.0, MotionPattern::VERTICAL},
  {0.55, 0.55, 0.82, 0.50, 0.50, 0.08, 0.0, 0.0, MotionPattern::DIAGONAL},
  {0.70, 0.50, 0.66, 0.50, 0.50, 0.39, 0.0, 0.0, MotionPattern::DIAGONAL},
  {0.50, 0.70, 0.97, 0.50, 0.50, 0.69, 0.0, 0.0, MotionPattern::DIAGONAL},
  {0.65, 0.65, 0.78, 0.34, 0.48, 0.17, 0.15, 0.25, MotionPattern::ELLIPSE},
  {0.85, 0.50, 0.60, 0.68, 0.54, 0.63, 0.13, 0.21, MotionPattern::ELLIPSE},
}};

struct MotionSample
{
  double x = 0.0;
  double y = 0.0;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
};

// 计算物体在给定区间内匀速往返运动的位置。
double reflectedPosition(
  double minimum,
  double maximum,
  double traveled_distance,
  double phase_ratio)
{
  const double travel = maximum - minimum;
  if (!std::isfinite(travel) || travel <= 0.0) {
    return minimum;
  }
  const double cycle = 2.0 * travel;
  const double phase_distance = std::fmod(
    std::max(0.0, traveled_distance) + std::clamp(phase_ratio, 0.0, 1.0) * cycle,
    cycle);
  return phase_distance <= travel ?
    minimum + phase_distance : maximum - (phase_distance - travel);
}

double reflectedVelocity(
  double minimum, double maximum, double traveled_distance, double phase_ratio,
  double speed)
{
  const double travel = maximum - minimum;
  if (!std::isfinite(travel) || travel <= 0.0) {return 0.0;}
  const double cycle = 2.0 * travel;
  const double phase_distance = std::fmod(
    std::max(0.0, traveled_distance) + std::clamp(phase_ratio, 0.0, 1.0) * cycle,
    cycle);
  return phase_distance <= travel ? speed : -speed;
}

MotionSample sampleMotion(
  const MovingRectangle & rectangle,
  double origin_x,
  double origin_y,
  double maximum_x,
  double maximum_y,
  double elapsed)
{
  MotionSample sample;
  const double half_width = 0.5 * rectangle.width_m;
  const double half_height = 0.5 * rectangle.height_m;
  const double minimum_x = origin_x + half_width;
  const double allowed_maximum_x = maximum_x - half_width;
  const double minimum_y = origin_y + half_height;
  const double allowed_maximum_y = maximum_y - half_height;
  const double width = maximum_x - origin_x;
  const double height = maximum_y - origin_y;

  const auto fixed_x = [&]() {
      return std::clamp(
        origin_x + rectangle.center_x_ratio * width, minimum_x, allowed_maximum_x);
    };
  const auto fixed_y = [&]() {
      return std::clamp(
        origin_y + rectangle.center_y_ratio * height, minimum_y, allowed_maximum_y);
    };

  if (rectangle.motion_pattern == MotionPattern::HORIZONTAL) {
    const double distance = rectangle.speed_mps * elapsed;
    sample.x = reflectedPosition(
      minimum_x, allowed_maximum_x, distance, rectangle.phase_ratio);
    sample.y = fixed_y();
    sample.velocity_x = reflectedVelocity(
      minimum_x, allowed_maximum_x, distance, rectangle.phase_ratio, rectangle.speed_mps);
  } else if (rectangle.motion_pattern == MotionPattern::VERTICAL) {
    const double distance = rectangle.speed_mps * elapsed;
    sample.x = fixed_x();
    sample.y = reflectedPosition(
      minimum_y, allowed_maximum_y, distance, rectangle.phase_ratio);
    sample.velocity_y = reflectedVelocity(
      minimum_y, allowed_maximum_y, distance, rectangle.phase_ratio, rectangle.speed_mps);
  } else if (rectangle.motion_pattern == MotionPattern::DIAGONAL) {
    const double x_distance = rectangle.speed_mps * elapsed;
    const double y_speed = 0.67 * rectangle.speed_mps;
    const double y_distance = y_speed * elapsed;
    const double y_phase = std::fmod(rectangle.phase_ratio + 0.31, 1.0);
    sample.x = reflectedPosition(
      minimum_x, allowed_maximum_x, x_distance, rectangle.phase_ratio);
    sample.y = reflectedPosition(minimum_y, allowed_maximum_y, y_distance, y_phase);
    sample.velocity_x = reflectedVelocity(
      minimum_x, allowed_maximum_x, x_distance, rectangle.phase_ratio, rectangle.speed_mps);
    sample.velocity_y = reflectedVelocity(
      minimum_y, allowed_maximum_y, y_distance, y_phase, y_speed);
  } else {
    const double center_x = fixed_x();
    const double center_y = fixed_y();
    const double radius_x = std::max(0.0, std::min({
      rectangle.span_x_ratio * width, center_x - minimum_x, allowed_maximum_x - center_x}));
    const double radius_y = std::max(0.0, std::min({
      rectangle.span_y_ratio * height, center_y - minimum_y, allowed_maximum_y - center_y}));
    const double reference_radius = std::max(0.1, std::max(radius_x, radius_y));
    const double angular_speed = rectangle.speed_mps / reference_radius;
    const double angle = 2.0 * std::acos(-1.0) * rectangle.phase_ratio +
      angular_speed * elapsed;
    sample.x = center_x + radius_x * std::cos(angle);
    sample.y = center_y + radius_y * std::sin(angle);
    sample.velocity_x = -radius_x * angular_speed * std::sin(angle);
    sample.velocity_y = radius_y * angular_speed * std::cos(angle);
  }
  return sample;
}

// 将轴对齐矩形覆盖到 OccupancyGrid 中。
void rasterizeRectangle(
  double center_x,
  double center_y,
  double width_m,
  double height_m,
  nav_msgs::msg::OccupancyGrid & map)
{
  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const double half_width = 0.5 * width_m;
  const double half_height = 0.5 * height_m;

  const int min_grid_x = std::max(
    0, static_cast<int>(std::floor((center_x - half_width - origin_x) / resolution)));
  const int max_grid_x = std::min(
    static_cast<int>(map.info.width) - 1,
    static_cast<int>(std::floor((center_x + half_width - origin_x) / resolution)));
  const int min_grid_y = std::max(
    0, static_cast<int>(std::floor((center_y - half_height - origin_y) / resolution)));
  const int max_grid_y = std::min(
    static_cast<int>(map.info.height) - 1,
    static_cast<int>(std::floor((center_y + half_height - origin_y) / resolution)));

  for (int grid_y = min_grid_y; grid_y <= max_grid_y; ++grid_y) {
    for (int grid_x = min_grid_x; grid_x <= max_grid_x; ++grid_x) {
      const double cell_x = origin_x + (static_cast<double>(grid_x) + 0.5) * resolution;
      const double cell_y = origin_y + (static_cast<double>(grid_y) + 0.5) * resolution;
      if (std::abs(cell_x - center_x) <= half_width &&
        std::abs(cell_y - center_y) <= half_height)
      {
        const size_t index = static_cast<size_t>(grid_x) +
          static_cast<size_t>(grid_y) * map.info.width;
        map.data[index] = 100;
      }
    }
  }
}

}  // namespace










// ============================================================
// Convert the source image into the raw occupancy map. Footprint inflation and
// distance fields are derived by the planner so dynamic updates use the same policy.
// 在这里加入机器人半径、安全余量、障碍膨胀、距离场或安全走廊等处理。
// 当前基线只做灰度阈值转换，输出中 0 表示可通行，100 表示障碍。
// ============================================================















bool BuildStaticMap(
  const Config & config,
  const cv::Mat & static_image,
  nav_msgs::msg::OccupancyGrid & output,
  std::string & reason)
{
  output = nav_msgs::msg::OccupancyGrid();
  reason.clear();

  if (static_image.empty() || static_image.type() != CV_8UC1) {
    reason = "static_image";
    return false;
  }
  if (config.map_bounds_m.size() < 6 || !finiteVector(config.map_bounds_m)) {
    reason = "map_bounds_m";
    return false;
  }
  if (config.image_origin_m.size() < 2 || !finiteVector(config.image_origin_m)) {
    reason = "image_origin_m";
    return false;
  }
  if (!std::isfinite(config.image_resolution_m_per_pixel) ||
    config.image_resolution_m_per_pixel <= 0.0)
  {
    reason = "image_resolution_m_per_pixel";
    return false;
  }
  if (!std::isfinite(config.map_resolution_m_per_cell) ||
    config.map_resolution_m_per_cell <= 0.0)
  {
    reason = "map_resolution_m_per_cell";
    return false;
  }
  if (config.obstacle_gray_threshold < 0 || config.obstacle_gray_threshold > 255) {
    reason = "obstacle_gray_threshold";
    return false;
  }

  const double map_origin_x = config.map_bounds_m[0];
  const double map_origin_y = config.map_bounds_m[2];
  const double map_resolution = config.map_resolution_m_per_cell;
  const int base_width = static_cast<int>(
    (config.map_bounds_m[1] - config.map_bounds_m[0]) / map_resolution);
  const int base_height = static_cast<int>(
    (config.map_bounds_m[3] - config.map_bounds_m[2]) / map_resolution);

  const double image_map_ratio = config.image_resolution_m_per_pixel / map_resolution;
  const double image_to_map_offset_x =
    (config.image_origin_m[0] - map_origin_x) / map_resolution;
  const double image_to_map_offset_y =
    (config.image_origin_m[1] - map_origin_y) / map_resolution;
  const int last_image_grid_x = static_cast<int>(
    image_map_ratio * static_cast<double>(static_image.cols - 1) + image_to_map_offset_x);
  const int last_image_grid_y = static_cast<int>(
    image_map_ratio * static_cast<double>(static_image.rows - 1) + image_to_map_offset_y);

  const int width = std::max(base_width, last_image_grid_x + 1);
  const int height = std::max(base_height, last_image_grid_y + 1);
  if (width <= 0 || height <= 0) {
    reason = "map_size";
    return false;
  }

  const size_t width_size = static_cast<size_t>(width);
  const size_t height_size = static_cast<size_t>(height);
  if (width_size > std::numeric_limits<size_t>::max() / height_size) {
    reason = "map_size_overflow";
    return false;
  }
  const size_t cell_count = width_size * height_size;
  if (width_size > std::numeric_limits<uint32_t>::max() ||
    height_size > std::numeric_limits<uint32_t>::max())
  {
    reason = "map_size_message_limit";
    return false;
  }

  output.header.frame_id = "map";
  output.info.resolution = static_cast<float>(map_resolution);
  output.info.width = static_cast<uint32_t>(width);
  output.info.height = static_cast<uint32_t>(height);
  output.info.origin.position.x = map_origin_x;
  output.info.origin.position.y = map_origin_y;
  output.info.origin.position.z = 0.0;
  output.info.origin.orientation.w = 1.0;
  output.data.assign(cell_count, 0);

  // 栅格坐标从左下角开始，图片行号从左上角开始，image_row 完成 y 方向翻转。
  const double map_image_ratio = map_resolution / config.image_resolution_m_per_pixel;
  const double map_to_image_offset_x =
    (map_origin_x - config.image_origin_m[0]) / config.image_resolution_m_per_pixel;
  const double map_to_image_offset_y =
    (map_origin_y - config.image_origin_m[1]) / config.image_resolution_m_per_pixel;

  for (int grid_y = 0; grid_y < height; ++grid_y) {
    for (int grid_x = 0; grid_x < width; ++grid_x) {
      const int image_x = static_cast<int>(
        map_image_ratio * static_cast<double>(grid_x) + map_to_image_offset_x);
      const int image_y = static_cast<int>(
        map_image_ratio * static_cast<double>(grid_y) + map_to_image_offset_y);
      if (image_x < 0 || image_x >= static_image.cols ||
        image_y < 0 || image_y >= static_image.rows)
      {
        continue;
      }

      const int image_row = static_image.rows - image_y - 1;
      const bool occupied =
        static_image.at<uint8_t>(image_row, image_x) < config.obstacle_gray_threshold;
      if (occupied) {
        const size_t index = static_cast<size_t>(grid_x) +
          static_cast<size_t>(grid_y) * width_size;
        output.data[index] = 100;
      }
    }
  }

  return true;
}

SimpleMapNode::SimpleMapNode(const rclcpp::NodeOptions & options)
: Node("simple_map", options), config_(readConfig()), dynamic_start_time_(now())
{
  std::string reason;
  if (!validateConfig(reason)) {
    RCLCPP_ERROR(get_logger(), "simple_map configuration error: %s", reason.c_str());
    return;
  }

  static_image_ = cv::imread(config_.image_path, cv::IMREAD_GRAYSCALE);
  if (static_image_.empty()) {
    RCLCPP_ERROR(get_logger(), "Failed to read static image: %s", config_.image_path.c_str());
    return;
  }
  if (!BuildStaticMap(config_, static_image_, static_map_, reason)) {
    RCLCPP_ERROR(get_logger(), "Failed to build static map: %s", reason.c_str());
    return;
  }

  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.reliable();
  map_qos.transient_local();
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_map", map_qos);
  dynamic_pub_ = create_publisher<plan_interfaces::msg::DynamicObstacleArray>(
    "/dynamic_obstacles", map_qos);

  initialized_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Static map ready: %ux%u, resolution %.3f m/cell, dynamic=%s, rectangles=%zu",
    static_map_.info.width, static_map_.info.height, static_map_.info.resolution,
    config_.generate_dynamic_obstacles ? "true" : "false",
    config_.generate_dynamic_obstacles ? kMovingRectangles.size() : 0U);

  if (config_.generate_dynamic_obstacles) {
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SimpleMapNode::timerCallback, this));
  } else {
    publishStaticMap();
  }
}

Config SimpleMapNode::readConfig()
{
  Config config;
  declare_parameter("cloud_bridge.image_path", std::string(""));
  declare_parameter("cloud_bridge.obstacle_gray_threshold", 128);
  declare_parameter(
    "cloud_bridge.map_bounds_m", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  declare_parameter("cloud_bridge.image_origin_m", std::vector<double>{0.0, 0.0});
  declare_parameter("cloud_bridge.image_resolution_m_per_pixel", 0.05);
  declare_parameter("cloud_bridge.map_resolution_m_per_cell", 0.05);
  declare_parameter("cloud_bridge.generate_dynamic_obstacles", false);

  config.image_path = get_parameter("cloud_bridge.image_path").as_string();
  config.obstacle_gray_threshold =
    static_cast<int>(get_parameter("cloud_bridge.obstacle_gray_threshold").as_int());
  config.map_bounds_m = get_parameter("cloud_bridge.map_bounds_m").as_double_array();
  config.image_origin_m = get_parameter("cloud_bridge.image_origin_m").as_double_array();
  config.image_resolution_m_per_pixel =
    get_parameter("cloud_bridge.image_resolution_m_per_pixel").as_double();
  config.map_resolution_m_per_cell =
    get_parameter("cloud_bridge.map_resolution_m_per_cell").as_double();
  config.generate_dynamic_obstacles =
    get_parameter("cloud_bridge.generate_dynamic_obstacles").as_bool();
  return config;
}

bool SimpleMapNode::validateConfig(std::string & reason) const
{
  reason.clear();
  if (config_.image_path.empty()) {
    reason = "cloud_bridge.image_path is empty";
    return false;
  }
  if (config_.map_bounds_m.size() < 6 || !finiteVector(config_.map_bounds_m)) {
    reason = "cloud_bridge.map_bounds_m must contain six finite values";
    return false;
  }
  if (config_.image_origin_m.size() < 2 || !finiteVector(config_.image_origin_m)) {
    reason = "cloud_bridge.image_origin_m must contain two finite values";
    return false;
  }
  if (!std::isfinite(config_.image_resolution_m_per_pixel) ||
    config_.image_resolution_m_per_pixel <= 0.0)
  {
    reason = "cloud_bridge.image_resolution_m_per_pixel must be positive";
    return false;
  }
  if (!std::isfinite(config_.map_resolution_m_per_cell) ||
    config_.map_resolution_m_per_cell <= 0.0)
  {
    reason = "cloud_bridge.map_resolution_m_per_cell must be positive";
    return false;
  }
  if (config_.obstacle_gray_threshold < 0 || config_.obstacle_gray_threshold > 255) {
    reason = "cloud_bridge.obstacle_gray_threshold must be in [0, 255]";
    return false;
  }
  return true;
}

void SimpleMapNode::publishStaticMap()
{
  combined_map_ = static_map_;
  combined_map_.header.stamp = now();
  combined_map_.info.map_load_time = combined_map_.header.stamp;
  map_pub_->publish(combined_map_);
  plan_interfaces::msg::DynamicObstacleArray dynamic_message;
  dynamic_message.header = combined_map_.header;
  dynamic_pub_->publish(dynamic_message);
}

















// ============================================================
// Deterministic moving-obstacle source used by the demo and prediction tests.
// 在这里接入或生成动态障碍，并写入传给规划器的地图数据。
// 当前演示生成 12 个不同尺寸和速度的矩形，混合横向、纵向、对角与椭圆运动。
// ============================================================





















void SimpleMapNode::updateDynamicObstacles(
  const rclcpp::Time & now_time,
  nav_msgs::msg::OccupancyGrid & map) const
{
  if (map.info.width == 0 || map.info.height == 0 || map.info.resolution <= 0.0F ||
    map.data.size() != static_cast<size_t>(map.info.width) * map.info.height)
  {
    return;
  }

  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const double map_width_m = static_cast<double>(map.info.width) * resolution;
  const double map_height_m = static_cast<double>(map.info.height) * resolution;
  const double map_max_x = origin_x + map_width_m;
  const double map_max_y = origin_y + map_height_m;
  const double elapsed = std::max(0.0, (now_time - dynamic_start_time_).seconds());

  for (const MovingRectangle & rectangle : kMovingRectangles) {
    if (map_width_m <= rectangle.width_m || map_height_m <= rectangle.height_m) {
      continue;
    }
    const MotionSample motion = sampleMotion(
      rectangle, origin_x, origin_y, map_max_x, map_max_y, elapsed);
    rasterizeRectangle(
      motion.x, motion.y, rectangle.width_m, rectangle.height_m, map);
  }
}

void SimpleMapNode::timerCallback()
{
  if (!initialized_) {
    return;
  }
  // 每帧从静态底图重新合成动态层。
  combined_map_ = static_map_;
  const rclcpp::Time stamp = now();
  updateDynamicObstacles(stamp, combined_map_);
  combined_map_.header.stamp = stamp;
  combined_map_.info.map_load_time = stamp;
  map_pub_->publish(combined_map_);

  plan_interfaces::msg::DynamicObstacleArray dynamic_message;
  dynamic_message.header = combined_map_.header;
  const double resolution = combined_map_.info.resolution;
  const double origin_x = combined_map_.info.origin.position.x;
  const double origin_y = combined_map_.info.origin.position.y;
  const double maximum_x = origin_x + combined_map_.info.width * resolution;
  const double maximum_y = origin_y + combined_map_.info.height * resolution;
  const double elapsed = std::max(0.0, (stamp - dynamic_start_time_).seconds());
  for (const MovingRectangle & rectangle : kMovingRectangles) {
    if (maximum_x - origin_x <= rectangle.width_m ||
      maximum_y - origin_y <= rectangle.height_m)
    {continue;}
    plan_interfaces::msg::DynamicObstacle obstacle;
    obstacle.header = dynamic_message.header;
    obstacle.shape = plan_interfaces::msg::DynamicObstacle::SHAPE_BOX;
    obstacle.size.x = rectangle.width_m;
    obstacle.size.y = rectangle.height_m;
    obstacle.valid_for = 0.25;
    const MotionSample motion = sampleMotion(
      rectangle, origin_x, origin_y, maximum_x, maximum_y, elapsed);
    obstacle.position.x = motion.x;
    obstacle.position.y = motion.y;
    obstacle.velocity.x = motion.velocity_x;
    obstacle.velocity.y = motion.velocity_y;
    dynamic_message.obstacles.push_back(obstacle);
  }
  dynamic_pub_->publish(dynamic_message);
}

}  // namespace simple_map
