#pragma once

#include "Eigen/Geometry"

namespace cartographer_ros {

struct VisualConstraintSanityLimits {
  double max_delta_z;
  double max_delta_roll;
  double max_delta_pitch;
};

class VisualConstraintAdapter {
public:
  Eigen::Isometry2d ProjectTo2d(const Eigen::Isometry3d& camera_pose,
                                const Eigen::Isometry3d& base_to_camera);
  bool PassesSanityGate(const Eigen::Isometry3d& constraint,
                        const VisualConstraintSanityLimits& limits) const;
};

}  // namespace cartographer_ros
