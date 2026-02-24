//
// Author: Qiayuan Liao
// Organization: Hybrid Robotics
// Source: https://github.com/HybridRobotics/motion_tracking_controller
//

#include "humanoid_controller/tasks/motion_tracking/Observation.h"

namespace legged {

// MotionAnchorPosition
vector_t MotionAnchorPosition::evaluate() {
  return commandTerm_->getAnchorPositionLocal();
}

// MotionAnchorOrientation
vector_t MotionAnchorOrientation::evaluate() {
  return commandTerm_->getAnchorOrientationLocal();
}

// RobotBodyPosition
vector_t RobotBodyPosition::evaluate() {
  return commandTerm_->getRobotBodyPositionLocal();
}

// RobotBodyOrientation
vector_t RobotBodyOrientation::evaluate() {
  return commandTerm_->getRobotBodyOrientationLocal();
}

}  // namespace legged
