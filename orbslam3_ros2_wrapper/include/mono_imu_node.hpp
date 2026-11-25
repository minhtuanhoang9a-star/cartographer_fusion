#pragma once

#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "constraint_bridge.hpp"
#include "system.h"  // mock ORB-SLAM3

namespace orbslam3_ros2_wrapper {

class MonoImuNode : public rclcpp::Node {
 public:
  MonoImuNode();
  ~MonoImuNode() override;

 private:
  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void SyncWithImuLoop();

  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::shared_ptr<ConstraintBridge> bridge_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  std::mutex data_mutex_;
  std::condition_variable data_cv_;
  std::queue<sensor_msgs::msg::Image::ConstSharedPtr> img_buf_;
  std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buf_;

  bool running_;
  double last_imu_stamp_;
  std::size_t max_img_queue_size_{0};
  std::thread sync_thread_;
};

}  // namespace orbslam3_ros2_wrapper
