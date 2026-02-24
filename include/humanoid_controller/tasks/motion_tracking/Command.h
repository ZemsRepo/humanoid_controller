//
// Author: Qiayuan Liao
// Organization: Hybrid Robotics
// Source: https://github.com/HybridRobotics/motion_tracking_controller
//

#pragma once

#include <legged_rl_controllers/CommandManager.h>

#include <utility>

#include "humanoid_controller/tasks/motion_tracking/Policy.h"

namespace legged {

struct MotionCommandCfg {
  std::string anchorBody;
  std::vector<std::string> bodyNames;
  bool loop_motion = false;
};

class MotionCommandTerm : public CommandTerm {
 public:
  using SharedPtr = std::shared_ptr<MotionCommandTerm>;

  MotionCommandTerm(MotionCommandCfg cfg, MotionOnnxPolicy::SharedPtr motionPolicy)
      : cfg_(std::move(cfg)), motionPolicy_(std::move(motionPolicy)), anchorRobotIndex_(0), anchorMotionIndex_(0) {}

  vector_t getValue() override;
  void reset() override;

  MotionCommandCfg getCfg() const {
    return cfg_;
  }
  vector3_t getAnchorPositionLocal() const;
  vector_t getAnchorOrientationLocal() const;
  vector_t getRobotBodyPositionLocal() const;
  vector_t getRobotBodyOrientationLocal() const;

 protected:
  size_t getSize() const override {
    return 2 * model_->getNumJoints();
  }

  MotionCommandCfg cfg_;
  MotionOnnxPolicy::SharedPtr motionPolicy_;

  size_t anchorRobotIndex_, anchorMotionIndex_;
  std::vector<size_t> bodyIndices_{};
  pinocchio::SE3 worldToInit_;
};

}  // namespace legged
