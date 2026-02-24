//
// Author: Qiayuan Liao
// Organization: Hybrid Robotics
// Source: https://github.com/HybridRobotics/motion_tracking_controller
//

#pragma once

#include <legged_model/common.h>
#include <legged_rl_controllers/ObservationManager.h>

#include "humanoid_controller/tasks/motion_tracking/Command.h"

namespace legged {

class MotionObservation : public ObservationTerm {
 public:
  explicit MotionObservation(const MotionCommandTerm::SharedPtr& commandTerm) : commandTerm_(commandTerm) {}

 protected:
  MotionCommandTerm::SharedPtr commandTerm_;
};

class MotionAnchorPosition final : public MotionObservation {
 public:
  using MotionObservation::MotionObservation;
  size_t getSize() const override {
    return 3;
  }

 protected:
  vector_t evaluate() override;
};

class MotionAnchorOrientation final : public MotionObservation {
 public:
  using MotionObservation::MotionObservation;
  size_t getSize() const override {
    return 6;
  }

 protected:
  vector_t evaluate() override;
};

class RobotBodyPosition final : public MotionObservation {
 public:
  using MotionObservation::MotionObservation;
  size_t getSize() const override {
    return 3 * commandTerm_->getCfg().bodyNames.size();
  }

 protected:
  vector_t evaluate() override;
};

class RobotBodyOrientation final : public MotionObservation {
 public:
  using MotionObservation::MotionObservation;
  size_t getSize() const override {
    return 6 * commandTerm_->getCfg().bodyNames.size();
  }

 protected:
  vector_t evaluate() override;
};

}  // namespace legged
