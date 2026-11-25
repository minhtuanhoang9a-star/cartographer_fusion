// mock_orbslam3/include/System.h
#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include "VisualConstraintObserver.h"

namespace ORB_SLAM3 {

namespace IMU {
struct Point {
  float a_x, a_y, a_z, w_x, w_y, w_z;
  double t;
  Point(float ax, float ay, float az, float wx, float wy, float wz, double ts)
      : a_x(ax), a_y(ay), a_z(az), w_x(wx), w_y(wy), w_z(wz), t(ts) {}
};
}  // namespace IMU

class System {
 public:
  enum eSensor { MONOCULAR = 0, STEREO = 1, RGBD = 2, IMU_MONOCULAR = 3 };

  System(const std::string&, const std::string&, eSensor, bool) {}

  void SetVisualConstraintObserver(VisualConstraintObserver* obs) { observer_ = obs; }

  void TrackMonocular(const cv::Mat& im, double stamp,
                      const std::vector<IMU::Point>& vImu) {
    EmitFakeConstraint(stamp, vImu.size());
  }

  void TrackStereo(const cv::Mat& left, const cv::Mat& right, double stamp) {
    EmitFakeConstraint(stamp, 0);
  }

  void TrackRGBD(const cv::Mat& rgb, const cv::Mat& depth, double stamp) {
    EmitFakeConstraint(stamp, 0);
  }

  void Shutdown() { std::cout << "[Mock ORB-SLAM3] Shutdown cleanly.\n"; }

 private:
  void EmitFakeConstraint(double stamp, size_t imu_count) {
    if (++frame_count_ % 10 != 0) return;  // Giả lập mỗi 10 frame tạo 1 KeyFrame
    if (last_kf_stamp_ > 0.0 && observer_) {
      RawVisualConstraint c{};
      c.stamp_i = last_kf_stamp_;
      c.stamp_j = stamp;
      c.kf_id_i = kf_id_;
      c.kf_id_j = ++kf_id_;
      c.map_id = 0;
      c.is_loop = (kf_id_ % 5 == 0);  // Cứ 5 KF giả lập 1 Loop Closure
      c.covisible_points = 80;
      c.sim3_inliers = c.is_loop ? 45 : 0;
      c.T_ki_kj = Sophus::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.5f));
      observer_->OnVisualConstraint(c);
    }
    last_kf_stamp_ = stamp;
  }

  VisualConstraintObserver* observer_ = nullptr;
  uint64_t frame_count_ = 0;
  uint64_t kf_id_ = 0;
  double last_kf_stamp_ = -1.0;
};

}  // namespace ORB_SLAM3