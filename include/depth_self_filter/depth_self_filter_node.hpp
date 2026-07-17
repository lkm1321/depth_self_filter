#pragma once

#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "depth_self_filter/renderer.hpp"
#include "depth_self_filter/urdf_meshes.hpp"

namespace depth_self_filter
{

/// Composable node: subtracts the robot's own body from a depth image by
/// rendering its URDF meshes into the camera view and masking pixels at/behind
/// the rendered robot surface.
class DepthSelfFilterNode : public rclcpp::Node
{
public:
  explicit DepthSelfFilterNode(const rclcpp::NodeOptions & options);

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using CamInfoMsg = sensor_msgs::msg::CameraInfo;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<ImageMsg, CamInfoMsg>;

  void on_robot_description(const std_msgs::msg::String::SharedPtr msg);
  void load_meshes(const std::string & urdf_xml);
  void on_frame(
    const ImageMsg::ConstSharedPtr & depth,
    const CamInfoMsg::ConstSharedPtr & info);

  // Build render items by looking up each link pose at `stamp` in the optical
  // frame. Returns false if too many TF lookups fail.
  bool build_items(
    const std::string & optical_frame, const rclcpp::Time & stamp,
    std::vector<RenderItem> & items);

  // Parameters
  bool use_collision_{true};
  double depth_margin_m_{0.02};
  int robot_dilation_px_{2};   // grow rendered robot depth (absorbs silhouette error)
  int mask_dilation_px_{4};
  double tf_timeout_s_{0.05};
  double time_offset_s_{0.0};
  float near_{0.02f}, far_{10.0f};
  std::vector<std::string> include_links_, exclude_links_;

  // State
  std::vector<LinkGeometry> links_;
  std::unique_ptr<IRenderer> renderer_;
  std::vector<float> robot_depth_;  // reused scratch buffer

  // ROS interfaces
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  message_filters::Subscriber<ImageMsg> depth_sub_;
  message_filters::Subscriber<CamInfoMsg> info_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr robot_depth_pub_;  // debug: rendered robot depth
  bool publish_debug_{false};
};

}  // namespace depth_self_filter
