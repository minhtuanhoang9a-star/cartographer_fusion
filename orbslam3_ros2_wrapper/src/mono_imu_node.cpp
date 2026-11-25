#include "mono_imu_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <stdexcept>
#include <vector>

namespace orbslam3_ros2_wrapper {

MonoImuNode::MonoImuNode()
    : Node("orbslam3_mono_imu_node"), running_(false), last_imu_stamp_(-1.0) {
  const auto vocab_path = declare_parameter<std::string>("vocab_path", "");
  const auto settings_path = declare_parameter<std::string>("settings_path", "");
  const auto optical_frame =
      declare_parameter<std::string>("camera_optical_frame", "cam0_optical_frame");
  const auto constraint_topic =
      declare_parameter<std::string>("constraint_topic", "/visual_constraints");
  const auto image_topic =
      declare_parameter<std::string>("image_topic", "/cam0/image_raw");
  const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu0");
  const bool enable_viewer = declare_parameter<bool>("enable_viewer", false);
  // Đặt 0 khi phát lại ROS bag (không drop frame); đặt 2-5 khi chạy real-time trên robot
  max_img_queue_size_ = static_cast<size_t>(
      declare_parameter<int>("max_img_queue_size", 0));

  if (vocab_path.empty() || settings_path.empty()) {
    RCLCPP_FATAL(get_logger(), "Missing required parameters: vocab_path or settings_path");
    throw std::invalid_argument("vocab_path and settings_path must not be empty");
  }

  slam_ = std::make_unique<ORB_SLAM3::System>(
      vocab_path, settings_path, ORB_SLAM3::System::IMU_MONOCULAR, enable_viewer);
  bridge_ = std::make_shared<ConstraintBridge>(this, constraint_topic, optical_frame);
  slam_->SetVisualConstraintObserver(bridge_.get());

  // Best-effort để khớp với phần lớn camera/IMU driver; subscriber reliable
  // sẽ không nhận được dữ liệu từ publisher best-effort.
  // Hàng đợi lớn cho IMU (200Hz - 500Hz) để không rớt mẫu giữa hai khung hình.
  const auto imu_qos = rclcpp::SensorDataQoS().keep_last(2000);
  const auto img_qos = rclcpp::SensorDataQoS().keep_last(20);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, imu_qos,
      std::bind(&MonoImuNode::ImuCallback, this, std::placeholders::_1));
  img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, img_qos,
      std::bind(&MonoImuNode::ImageCallback, this, std::placeholders::_1));

  running_ = true;
  sync_thread_ = std::thread(&MonoImuNode::SyncWithImuLoop, this);

  RCLCPP_INFO(get_logger(),
              "ORB-SLAM3 Mono-IMU node ready (img: %s, imu: %s, out: %s)",
              image_topic.c_str(), imu_topic.c_str(), constraint_topic.c_str());
}

MonoImuNode::~MonoImuNode() {
  // 1. Dừng luồng đồng bộ trước để không còn lời gọi nào vào slam_->TrackMonocular()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    running_ = false;
  }
  data_cv_.notify_all();
  if (sync_thread_.joinable()) {
    sync_thread_.join();
  }

  // 2. Gỡ observer và tắt ORB-SLAM3
  if (slam_) {
    slam_->SetVisualConstraintObserver(nullptr);
    slam_->Shutdown();
  }

  // 3. Dừng luồng publish ROS 2
  if (bridge_) {
    bridge_->Stop();
  }
}

void MonoImuNode::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!running_) return;

    if (max_img_queue_size_ > 0 && img_buf_.size() >= max_img_queue_size_) {
      img_buf_.pop();
    }
    img_buf_.push(msg);
  }
  data_cv_.notify_one();
}

void MonoImuNode::ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  const double t_imu = rclcpp::Time(msg->header.stamp).seconds();

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!running_) return;

    // Lọc gói IMU trùng hoặc lùi thời gian để tránh lỗi dt <= 0 trong IMU Preintegration
    if (t_imu <= last_imu_stamp_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Out-of-order IMU sample dropped (t=%.6f <= last=%.6f)",
                           t_imu, last_imu_stamp_);
      return;
    }
    last_imu_stamp_ = t_imu;
    imu_buf_.push(msg);
  }
  data_cv_.notify_one();
}

void MonoImuNode::SyncWithImuLoop() {
  while (true) {
    sensor_msgs::msg::Image::ConstSharedPtr img_msg;
    double t_img = 0.0;
    std::vector<ORB_SLAM3::IMU::Point> v_imu_meas;

    {
      std::unique_lock<std::mutex> lock(data_mutex_);
      data_cv_.wait(lock, [this]() {
        if (!running_) return true;
        if (img_buf_.empty() || imu_buf_.empty()) return false;

        const double t_first_img =
            rclcpp::Time(img_buf_.front()->header.stamp).seconds();
        const double t_last_imu =
            rclcpp::Time(imu_buf_.back()->header.stamp).seconds();
        // Chỉ xử lý khi đã có ít nhất một mẫu IMU bọc phía sau thời điểm chụp ảnh
        return t_first_img <= t_last_imu;
      });

      if (!running_) break;

      img_msg = img_buf_.front();
      img_buf_.pop();
      t_img = rclcpp::Time(img_msg->header.stamp).seconds();

      while (!imu_buf_.empty()) {
        const auto& imu_msg = imu_buf_.front();
        const double t_imu = rclcpp::Time(imu_msg->header.stamp).seconds();
        if (t_imu > t_img) break;

        v_imu_meas.emplace_back(
            static_cast<float>(imu_msg->linear_acceleration.x),
            static_cast<float>(imu_msg->linear_acceleration.y),
            static_cast<float>(imu_msg->linear_acceleration.z),
            static_cast<float>(imu_msg->angular_velocity.x),
            static_cast<float>(imu_msg->angular_velocity.y),
            static_cast<float>(imu_msg->angular_velocity.z),
            t_imu);
        imu_buf_.pop();
      }
    }  // Nhả mutex trước khi giải mã ảnh và chạy TrackMonocular

    // Bỏ qua các khung ảnh đầu tiên đến trước khi stream IMU bắt đầu
    if (v_imu_meas.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No IMU measurements before image stamp %.6f, skipping frame.",
                           t_img);
      continue;
    }

    cv_bridge::CvImageConstPtr cv_img;
    try {
      cv_img = cv_bridge::toCvShare(img_msg, "mono8");
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "cv_bridge conversion failed: %s", e.what());
      continue;
    }

    if (cv_img->image.empty()) continue;

    // ORB-SLAM3 nhận ma trận ảnh mono8 và danh sách mẫu IMU tích lũy đến thời điểm t_img
    slam_->TrackMonocular(cv_img->image, t_img, v_imu_meas);
  }
}

}  // namespace orbslam3_ros2_wrapper

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<orbslam3_ros2_wrapper::MonoImuNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("orbslam3_mono_imu_node"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}