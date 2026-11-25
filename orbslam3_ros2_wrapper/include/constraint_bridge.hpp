#pragma once

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "visual_constraint_msgs/msg/visual_constraint.hpp"

#include "VisualConstraintObserver.h"

namespace orbslam3_ros2_wrapper {

class ConstraintBridge : public ORB_SLAM3::VisualConstraintObserver {
 public:
  ConstraintBridge(rclcpp::Node* node, const std::string& topic_name,
                   const std::string& optical_frame_id);
  ~ConstraintBridge() override;

  void Stop();

  void OnVisualConstraint(const ORB_SLAM3::RawVisualConstraint& raw) override;

 private:
  void PublishLoop();

  static constexpr std::size_t kMaxQueueSize = 200;

  std::string optical_frame_id_;
  rclcpp::Publisher<visual_constraint_msgs::msg::VisualConstraint>::SharedPtr pub_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::queue<ORB_SLAM3::RawVisualConstraint> queue_;
  bool running_;
  std::thread worker_thread_;
};

}  // namespace orbslam3_ros2_wrapper
