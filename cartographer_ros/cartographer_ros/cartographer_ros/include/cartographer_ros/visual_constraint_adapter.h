// Start Modify
#ifndef CARTOGRAPHER_ROS_VISUAL_CONSTRAINT_ADAPTER_H_
#define CARTOGRAPHER_ROS_VISUAL_CONSTRAINT_ADAPTER_H_

#include <cstdint>
#include <optional>
#include <string>

#include "cartographer/sensor/visual_constraint_data.h"
#include "cartographer_ros/tf_bridge.h"
#include "visual_constraint_msgs/msg/visual_constraint.hpp"

namespace cartographer_ros {

struct VisualConstraintGateOptions {
  double max_abs_delta_z = 0.25;          // mét (chặn lỗi mất phẳng 2D)
  double max_abs_roll_rad = 0.20;         // ~11.5 độ
  double max_abs_pitch_rad = 0.20;        // ~11.5 độ
  double max_seq_translation_m = 3.0;     // chặn nhảy pose bất thường ở cạnh Sequential
  uint32_t min_covisible_points = 20;     // ngưỡng tối thiểu cho Sequential
  uint32_t min_sim3_inliers = 25;         // ngưỡng tối thiểu cho Loop Closure
  bool dry_run_log = false;               // in log đối chiếu trong Phase 3
};

class VisualConstraintAdapter {
 public:
  VisualConstraintAdapter(const TfBridge* tf_bridge,
                          const VisualConstraintGateOptions& options = {});

  std::optional<cartographer::sensor::VisualConstraintData> Convert(
      const visual_constraint_msgs::msg::VisualConstraint& msg) const;

 private:
  const TfBridge* const tf_bridge_;  // không sở hữu (do MapBuilderBridge quản lý)
  const VisualConstraintGateOptions options_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_VISUAL_CONSTRAINT_ADAPTER_H_
// End Modify