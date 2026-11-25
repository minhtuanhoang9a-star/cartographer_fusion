#pragma once

#include <mutex>
#include <queue>

#include "orbslam3_core/VisualConstraintObserver.h"
#include "visual_constraint_msgs/msg/visual_constraint.hpp"

namespace fusion_slam {

class ConstraintBridge : public orbslam3::VisualConstraintObserver {
public:
  void OnSequentialConstraint(const orbslam3::RawVisualConstraint& constraint) override;
  void OnLoopConstraint(const orbslam3::RawVisualConstraint& constraint) override;

  bool Pop(visual_constraint_msgs::msg::VisualConstraint* message);

private:
  std::mutex mutex_;
  std::queue<visual_constraint_msgs::msg::VisualConstraint> queue_;
};

}  // namespace fusion_slam
