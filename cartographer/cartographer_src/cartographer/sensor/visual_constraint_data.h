// Start Modify
#ifndef CARTOGRAPHER_SENSOR_VISUAL_CONSTRAINT_DATA_H_
#define CARTOGRAPHER_SENSOR_VISUAL_CONSTRAINT_DATA_H_

#include <cstdint>

#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace sensor {

enum class VisualConstraintType : uint8_t {
  kSequential = 0,
  kLoopClosure = 1,
};

struct VisualConstraintData {
  common::Time stamp_i;
  common::Time stamp_j;
  uint64_t keyframe_id_i = 0;
  uint64_t keyframe_id_j = 0;
  uint64_t map_id = 0;
  // T_{Bi, Bj} da doi ve tracking_frame va chieu xuong SE(2).
  transform::Rigid2d relative_pose_2d;
  VisualConstraintType type = VisualConstraintType::kSequential;
  uint32_t covisible_points = 0;
  uint32_t sim3_inliers = 0;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_VISUAL_CONSTRAINT_DATA_H_
// End Modify
