#pragma once

#include "rclcpp/rclcpp.hpp"

namespace fusion_slam {
class MonoImuNode : public rclcpp::Node {
public:
  MonoImuNode();
};
}  // namespace fusion_slam
