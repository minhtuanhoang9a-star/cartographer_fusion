#pragma once

#include <memory>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "constraint_bridge.hpp"
#include "system.h"  // mock ORB-SLAM3

namespace orbslam3_ros2_wrapper {

class StereoNode : public rclcpp::Node {
 public:
  StereoNode();
  ~StereoNode() override;

 private:
  using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Sync = message_filters::Synchronizer<ApproxSyncPolicy>;

  void StereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);

  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::shared_ptr<ConstraintBridge> bridge_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_left_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_right_;
  std::shared_ptr<Sync> sync_;
};

}  // namespace orbslam3_ros2_wrapper
