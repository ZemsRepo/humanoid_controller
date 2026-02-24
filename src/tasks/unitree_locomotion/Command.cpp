//
// Author: Unitree Robotics
// Organization: Unitree Robotics
// Source: https://github.com/unitreerobotics/unitree_rl_gym
//

#include "humanoid_controller/tasks/unitree_locomotion/Command.h"

#include <cmath>

namespace legged {

void UnitreeGaitCommandTerm::update() {
  counter_++;
  scalar_t time = static_cast<scalar_t>(counter_) * envDt_;
  phase_ = std::fmod(time, gaitPeriod_) / gaitPeriod_;
}

}  // namespace legged
