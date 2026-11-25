#include "stereo_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <functional>
#include <stdexcept>

namespace orbslam3_ros2_wrapper {

StereoNode::StereoNode() : Node("orbslam3_stereo_node") {
  const auto vocab_path = declare_parameter<std::string>("vocab_path", "");
  const auto settings_path = declare_parameter<std::string>("settings_path", "");
  const auto optical_frame =
      declare_parameter<std::string>("camera_optical_frame", "cam0_optical_frame");
  const auto constraint_topic =
      declare_parameter<std::string>("constraint_topic", "/visual_constraints");
  const auto left_topic =
      declare_parameter<std::string>("left_topic", "/camera/left/image_raw");
  const auto right_topic =
      declare_parameter<std::string>("right_topic", "/camera/right/image_raw");
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
      vocab_path, settings_path, ORB_SLAM3::System::STEREO, enable_viewer);
  bridge_ = std::make_shared<ConstraintBridge>(this, constraint_topic, optical_frame);
  slam_->SetVisualConstraintObserver(bridge_.get());

  // Best-effort để khớp với phần lớn camera driver (RealSense, USB cam...).
  sub_left_.subscribe(this, left_topic, rmw_qos_profile_sensor_data);
  sub_right_.subscribe(this, right_topic, rmw_qos_profile_sensor_data);

  sync_ = std::make_shared<Sync>(ApproxSyncPolicy(sync_queue_size), sub_left_, sub_right_);
  sync_->registerCallback(
      std::bind(&StereoNode::StereoCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(),
              "ORB-SLAM3 Stereo node ready (left: %s, right: %s, out: %s)",
              left_topic.c_str(), right_topic.c_str(), constraint_topic.c_str());
}

StereoNode::~StereoNode() {
  // 1. Ngắt nhận callback ảnh trước
  sync_.reset();
  sub_left_.unsubscribe();
  sub_right_.unsubscribe();

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

void StereoNode::StereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& right_msg) {
  cv_bridge::CvImageConstPtr cv_left;
  cv_bridge::CvImageConstPtr cv_right;

  try {
    cv_left = cv_bridge::toCvShare(left_msg, "mono8");
    cv_right = cv_bridge::toCvShare(right_msg, "mono8");
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "cv_bridge conversion failed: %s", e.what());
    return;
  }

  if (cv_left->image.empty() || cv_right->image.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Received empty left or right image, skipping frame.");
    return;
  }

  const double stamp = rclcpp::Time(left_msg->header.stamp).seconds();
  slam_->TrackStereo(cv_left->image, cv_right->image, stamp);
}

}  // namespace orbslam3_ros2_wrapper

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<orbslam3_ros2_wrapper::StereoNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("orbslam3_stereo_node"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
