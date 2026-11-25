#include "rgbd_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <functional>
#include <stdexcept>

namespace orbslam3_ros2_wrapper {

RgbdNode::RgbdNode() : Node("orbslam3_rgbd_node") {
  const auto vocab_path = declare_parameter<std::string>("vocab_path", "");
  const auto settings_path = declare_parameter<std::string>("settings_path", "");
  const auto optical_frame =
      declare_parameter<std::string>("camera_optical_frame", "camera_color_optical_frame");
  const auto constraint_topic =
      declare_parameter<std::string>("constraint_topic", "/visual_constraints");
  const auto rgb_topic =
      declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
  const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/camera/aligned_depth_to_color/image_raw");
  const bool enable_viewer = declare_parameter<bool>("enable_viewer", false);
  const int sync_queue_size = declare_parameter<int>("sync_queue_size", 10);

  if (vocab_path.empty() || settings_path.empty()) {
    RCLCPP_FATAL(get_logger(), "Missing required parameters: vocab_path or settings_path");
    throw std::invalid_argument("vocab_path and settings_path must not be empty");
  }
  if (sync_queue_size <= 0) {
    RCLCPP_FATAL(get_logger(), "sync_queue_size must be positive, got %d", sync_queue_size);
    throw std::invalid_argument("sync_queue_size must be positive");
  }

  slam_ = std::make_unique<ORB_SLAM3::System>(
      vocab_path, settings_path, ORB_SLAM3::System::RGBD, enable_viewer);
  bridge_ = std::make_shared<ConstraintBridge>(this, constraint_topic, optical_frame);
  slam_->SetVisualConstraintObserver(bridge_.get());

  // Best-effort để khớp với phần lớn camera driver (RealSense, USB cam...).
  sub_rgb_.subscribe(this, rgb_topic, rmw_qos_profile_sensor_data);
  sub_depth_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);

  sync_ = std::make_shared<Sync>(ApproxSyncPolicy(sync_queue_size), sub_rgb_, sub_depth_);
  sync_->registerCallback(std::bind(&RgbdNode::RgbdCallback, this,
                                    std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(),
              "ORB-SLAM3 RGB-D node ready (rgb: %s, depth: %s, out: %s)",
              rgb_topic.c_str(), depth_topic.c_str(), constraint_topic.c_str());
}

RgbdNode::~RgbdNode() {
  // 1. Ngắt nhận callback ảnh trước
  sync_.reset();
  sub_rgb_.unsubscribe();
  sub_depth_.unsubscribe();

  // 2. Gỡ observer và tắt toàn bộ luồng nội bộ của ORB-SLAM3
  if (slam_) {
    slam_->SetVisualConstraintObserver(nullptr);
    slam_->Shutdown();
  }

  // 3. Dừng luồng publisher sau khi chắc chắn không còn callback từ ORB-SLAM3
  if (bridge_) {
    bridge_->Stop();
  }
}

void RgbdNode::RgbdCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
  cv_bridge::CvImageConstPtr cv_rgb;
  cv_bridge::CvImageConstPtr cv_depth;

  try {
    // Khớp với Camera.RGB: 0 trong YAML ORB-SLAM3 (đổi sang "rgb8" nếu YAML đặt Camera.RGB: 1)
    cv_rgb = cv_bridge::toCvShare(rgb_msg, "bgr8");
    // Truyền "" để giữ nguyên encoding gốc (16UC1 theo mm hoặc 32FC1 theo m)
    cv_depth = cv_bridge::toCvShare(depth_msg, "");
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "cv_bridge conversion failed: %s", e.what());
    return;
  }

  if (cv_rgb->image.empty() || cv_depth->image.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Received empty RGB or Depth image, skipping frame.");
    return;
  }

  const double stamp = rclcpp::Time(rgb_msg->header.stamp).seconds();
  slam_->TrackRGBD(cv_rgb->image, cv_depth->image, stamp);
}

}  // namespace orbslam3_ros2_wrapper

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<orbslam3_ros2_wrapper::RgbdNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("orbslam3_rgbd_node"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}