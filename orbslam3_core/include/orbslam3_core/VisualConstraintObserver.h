#pragma once

#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace orbslam3 {

enum class VisualConstraintType : std::uint8_t {
  kSequential = 0,
  kLoopClosure = 1,
};

struct RawVisualConstraint {
  double stamp_i;
  double stamp_j;
  Eigen::Isometry3d relative_pose;
  VisualConstraintType type;
  std::uint32_t covisible_points;
  std::uint32_t sim3_inliers;
  std::uint64_t map_id;
};

class VisualConstraintObserver {
public:
  virtual ~VisualConstraintObserver() = default;
  virtual void OnSequentialConstraint(const RawVisualConstraint& constraint) = 0;
  virtual void OnLoopConstraint(const RawVisualConstraint& constraint) = 0;
};

}  // namespace orbslam3
