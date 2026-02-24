//
// Author: Amazon FAR
// Organization: Amazon FAR
// Source: https://github.com/amazon-far/holosoma
//

#pragma once

#include <legged_model/common.h>
#include <legged_rl_controllers/CommandManager.h>

namespace legged {

struct WalkingCommandCfg {
  scalar_t gait_period;
  scalar_t scale_lin_vel;
  scalar_t scale_ang_vel;
  scalar_t scale_dof_pos;
  scalar_t scale_dof_vel;
  vector3_t scale_command;
};

class LocomotionGaitCommandTerm : public CommandTerm {
 public:
  explicit LocomotionGaitCommandTerm(scalar_t gait_period) : gait_period_(gait_period) {}

  void update(const vector_t& command_vel, const scalar_t period_seconds);
  vector_t getValue() override;
  void reset() override;

  size_t getSize() const override {
    return 2;
  }

 protected:
  scalar_t gait_period_{1.0};
  scalar_t phase_{0.0};
  vector2_t phase_offset_{M_PI, 0.0};
  scalar_t stand_phase_value_{M_PI};
  bool stand_mask_{true};
  vector_t phase_vec_{stand_phase_value_ * vector_t::Ones(2)};
};

}  // namespace legged
