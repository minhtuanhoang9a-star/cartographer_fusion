#include "constraint_bridge.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <cmath>
#include <Eigen/Geometry>

namespace orbslam3_ros2_wrapper {

namespace {
builtin_interfaces::msg::Time DoubleToRosTime(double t_sec) {
  const int64_t total_ns = static_cast<int64_t>(std::llround(t_sec * 1e9));
  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<int32_t>(total_ns / 1000000000LL);
  msg.nanosec = static_cast<uint32_t>(total_ns % 1000000000LL);
  return msg;
}
}  // namespace

ConstraintBridge::ConstraintBridge(rclcpp::Node* node,
                                   const std::string& topic_name,
                                   const std::string& optical_frame_id)
    : optical_frame_id_(optical_frame_id), running_(true) {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable();
  pub_ = node->create_publisher<visual_constraint_msgs::msg::VisualConstraint>(
      topic_name, qos);
  worker_thread_ = std::thread(&ConstraintBridge::PublishLoop, this);
}

ConstraintBridge::~ConstraintBridge() { Stop(); }

void ConstraintBridge::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) return;
    running_ = false;
  }
  cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void ConstraintBridge::OnVisualConstraint(
    const ORB_SLAM3::RawVisualConstraint& raw) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) return;
    if (queue_.size() >= kMaxQueueSize) {
      queue_.pop();  // Tránh phình bộ nhớ nếu subscriber dừng
    }
    queue_.push(raw);
  }
  cv_.notify_one();
}

void ConstraintBridge::PublishLoop() {
  while (true) {
    ORB_SLAM3::RawVisualConstraint item;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return !queue_.empty() || !running_; });
      if (!running_ && queue_.empty()) break;
      item = std::move(queue_.front());
      queue_.pop();
    }

    visual_constraint_msgs::msg::VisualConstraint msg;
    msg.header.frame_id = optical_frame_id_;
    msg.stamp_i = DoubleToRosTime(item.stamp_i);
    msg.stamp_j = DoubleToRosTime(item.stamp_j);
    msg.header.stamp = msg.stamp_j;

    msg.keyframe_id_i = item.kf_id_i;
    msg.keyframe_id_j = item.kf_id_j;
    msg.map_id = item.map_id;
    msg.type = item.is_loop
                   ? visual_constraint_msgs::msg::VisualConstraint::LOOP_CLOSURE
                   : visual_constraint_msgs::msg::VisualConstraint::SEQUENTIAL;
    msg.covisible_points = item.covisible_points;
    msg.sim3_inliers = item.sim3_inliers;

    // Quy đổi T_{Ki, Kj} sang geometry_msgs/Pose (luôn chuẩn hóa quaternion)
    const Eigen::Vector3d t = item.T_ki_kj.translation().cast<double>();
    Eigen::Quaterniond q(item.T_ki_kj.rotationMatrix().cast<double>());
    q.normalize();

    msg.relative_pose.position.x = t.x();
    msg.relative_pose.position.y = t.y();
    msg.relative_pose.position.z = t.z();
    msg.relative_pose.orientation.x = q.x();
    msg.relative_pose.orientation.y = q.y();
    msg.relative_pose.orientation.z = q.z();
    msg.relative_pose.orientation.w = q.w();

    pub_->publish(msg);
  }
}

}  // namespace orbslam3_ros2_wrapper