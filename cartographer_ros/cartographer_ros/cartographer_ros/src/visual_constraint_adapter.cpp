// Start Modify
#include "cartographer_ros/visual_constraint_adapter.h"

#include <algorithm>
#include <cmath>

#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"
#include "rclcpp/time.hpp"

namespace cartographer_ros {

namespace {

bool IsFinitePose(const geometry_msgs::msg::Pose& p) {
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.position.z) && std::isfinite(p.orientation.x) &&
         std::isfinite(p.orientation.y) && std::isfinite(p.orientation.z) &&
         std::isfinite(p.orientation.w);
}

}  // namespace

VisualConstraintAdapter::VisualConstraintAdapter(
    const TfBridge* tf_bridge, const VisualConstraintGateOptions& options)
    : tf_bridge_(tf_bridge), options_(options) {}

std::optional<cartographer::sensor::VisualConstraintData>
VisualConstraintAdapter::Convert(
    const visual_constraint_msgs::msg::VisualConstraint& msg) const {
  if (tf_bridge_ == nullptr) {
    LOG_EVERY_N(ERROR, 50) << "VisualConstraintAdapter: tf_bridge_ is null.";
    return std::nullopt;
  }

  // 1. Kiểm tra dữ liệu hợp lệ cơ bản
  if (!IsFinitePose(msg.relative_pose) || msg.header.frame_id.empty()) {
    LOG_EVERY_N(WARNING, 20) << "Dropped VisualConstraint: NaN/Inf pose or empty frame_id.";
    return std::nullopt;
  }

  const cartographer::common::Time stamp_i =
      FromRos(rclcpp::Time(msg.stamp_i));
  const cartographer::common::Time stamp_j =
      FromRos(rclcpp::Time(msg.stamp_j));
  if (stamp_j <= stamp_i) {
    LOG_EVERY_N(WARNING, 20) << "Dropped VisualConstraint: stamp_j <= stamp_i.";
    return std::nullopt;
  }

  const bool is_loop =
      (msg.type == visual_constraint_msgs::msg::VisualConstraint::LOOP_CLOSURE);
  if (!is_loop && msg.covisible_points < options_.min_covisible_points) {
    return std::nullopt;
  }
  if (is_loop && msg.sim3_inliers < options_.min_sim3_inliers) {
    return std::nullopt;
  }

  // 2. Tra TF tĩnh T_{BC}: tracking_frame <- camera_optical_frame
  const auto sensor_to_tracking =
      tf_bridge_->LookupToTracking(stamp_j, msg.header.frame_id);
  if (sensor_to_tracking == nullptr) {
    LOG_EVERY_N(WARNING, 20)
        << "Dropped VisualConstraint: TF lookup failed for frame '"
        << msg.header.frame_id << "'.";
    return std::nullopt;
  }

  // 3. Đổi cơ sở từ Camera sang Tracking Frame (Base):
  //    T_{Bi, Bj} = T_{BC} * T_{Ki, Kj} * inv(T_{BC})
  const cartographer::transform::Rigid3d T_ki_kj = ToRigid3d(msg.relative_pose);
  const cartographer::transform::Rigid3d T_bi_bj =
      (*sensor_to_tracking) * T_ki_kj * sensor_to_tracking->inverse();

  // 4. Sanity Gate cho bài toán 2D phẳng (lọc delta_z, roll, pitch, bước nhảy)
  const double dz = std::abs(T_bi_bj.translation().z());
  const Eigen::Matrix3d R = T_bi_bj.rotation().toRotationMatrix();
  const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  const double roll = std::atan2(R(2, 1), R(2, 2));

  if (dz > options_.max_abs_delta_z ||
      std::abs(roll) > options_.max_abs_roll_rad ||
      std::abs(pitch) > options_.max_abs_pitch_rad) {
    LOG_EVERY_N(WARNING, 20)
        << "Dropped VisualConstraint (KF " << msg.keyframe_id_i << "->"
        << msg.keyframe_id_j << "): planar gate violated (dz=" << dz
        << ", roll=" << roll << ", pitch=" << pitch << ").";
    return std::nullopt;
  }

  const cartographer::transform::Rigid2d T_bi_bj_2d =
      cartographer::transform::Project2D(T_bi_bj);

  if (!is_loop && T_bi_bj_2d.translation().norm() > options_.max_seq_translation_m) {
    LOG_EVERY_N(WARNING, 20)
        << "Dropped Sequential VisualConstraint: jump too large ("
        << T_bi_bj_2d.translation().norm() << " m).";
    return std::nullopt;
  }

  // 5. Đóng gói dữ liệu chuẩn Cartographer Core
  cartographer::sensor::VisualConstraintData data;
  data.stamp_i = stamp_i;
  data.stamp_j = stamp_j;
  data.keyframe_id_i = msg.keyframe_id_i;
  data.keyframe_id_j = msg.keyframe_id_j;
  data.map_id = msg.map_id;
  data.relative_pose_2d = T_bi_bj_2d;
  data.type = is_loop ? cartographer::sensor::VisualConstraintType::kLoopClosure
                      : cartographer::sensor::VisualConstraintType::kSequential;
  data.covisible_points = msg.covisible_points;
  data.sim3_inliers = msg.sim3_inliers;

  if (options_.dry_run_log) {
    LOG(INFO) << "[VisualDryRun][" << (is_loop ? "LOOP" : "SEQ") << "] KF "
              << data.keyframe_id_i << "->" << data.keyframe_id_j
              << " dx=" << data.relative_pose_2d.translation().x()
              << " dy=" << data.relative_pose_2d.translation().y()
              << " dyaw=" << data.relative_pose_2d.rotation().angle();
  }

  return data;
}

}  // namespace cartographer_ros
// End Modify