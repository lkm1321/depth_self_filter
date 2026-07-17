// Standalone executable wrapper around the composable node.
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "depth_self_filter/depth_self_filter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<depth_self_filter::DepthSelfFilterNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
