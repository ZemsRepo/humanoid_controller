//
// Author: Amazon FAR
// Organization: Amazon FAR
// Source: https://github.com/amazon-far/holosoma
//

#include "humanoid_controller/tasks/holosoma_locomotion/Command.h"

namespace legged {

void LocomotionGaitCommandTerm::update(const vector_t& command_vel, const scalar_t period_seconds) {
  stand_mask_ = command_vel.norm() < 0.01 && std::fabs(command_vel(2)) < 0.01;

  if (stand_mask_) {
    phase_vec_ = stand_phase_value_ * vector_t::Ones(2);
    phase_ = 0.0;
  } else {
    scalar_t phase_dt = 2.0 * M_PI * period_seconds / gait_period_;
    phase_ += phase_dt;

    vector_t phase_tp1 = vector_t::Zero(2);
    phase_tp1[0] = phase_ + phase_offset_[0];
    phase_tp1[1] = phase_ + phase_offset_[1];
    phase_vec_[0] = std::fmod(phase_tp1[0] + M_PI, 2.0 * M_PI) - M_PI;
    phase_vec_[1] = std::fmod(phase_tp1[1] + M_PI, 2.0 * M_PI) - M_PI;
  }
}

vector_t LocomotionGaitCommandTerm::getValue() {
  return phase_vec_;
}

void LocomotionGaitCommandTerm::reset() {
  stand_mask_ = true;
}

}  // namespace legged