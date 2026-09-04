#include "simple_map/simple_map.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_map::SimpleMapNode>());
  rclcpp::shutdown();
  return 0;
}
