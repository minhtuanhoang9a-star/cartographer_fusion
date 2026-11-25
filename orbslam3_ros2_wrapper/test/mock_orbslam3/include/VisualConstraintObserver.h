#pragma once

#include <cstdint>
#include <sophus/se3.hpp>

namespace ORB_SLAM3 {

struct RawVisualConstraint {
  double stamp_i = 0.0;
  double stamp_j = 0.0;

  uint64_t kf_id_i = 0;
  uint64_t kf_id_j = 0;
  uint64_t map_id = 0;

  // Relative pose of KeyFrame j expressed in KeyFrame i optical frame:
  // T_{Ki, Kj} = T_{Ki, W} * inv(T_{Kj, W})
  Sophus::SE3f T_ki_kj;

  bool is_loop = false;
  uint32_t covisible_points = 0;
  uint32_t sim3_inliers = 0;
};

class VisualConstraintObserver {
 public:
  virtual ~VisualConstraintObserver() = default;
  virtual void OnVisualConstraint(const RawVisualConstraint& constraint) = 0;
};

}  // namespace ORB_SLAM3