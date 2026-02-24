//
// Author: Unitree Robotics
// Organization: Unitree Robotics
// Source: https://github.com/unitreerobotics/unitree_rl_gym
//

#pragma once

#include <legged_rl_controllers/RlController.h>

#include "humanoid_controller/tasks/unitree_locomotion/Command.h"
#include "humanoid_controller/tasks/unitree_locomotion/Policy.h"

namespace legged {
class UnitreeLocomotionController : public RlController {
 public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  bool parserCommand(const std::string& name) override;
  bool parserObservation(const std::string& name) override;

  // Configuration
  WalkingCommandCfg cfg_;

  // Command terms
  UnitreeVelocityCommandTerm::SharedPtr velocityCommandTerm_;
  UnitreeGaitCommandTerm::SharedPtr gaitCommandTerm_;

  // Policy (casted for accessing unitree-specific methods)
  std::shared_ptr<UnitreeLocomotionPolicy> unitreePolicy_;

  // Upper body control (fixed position with PD)
  std::vector<std::string> upperBodyJointNames_;
  vector_t upperBodyDefaultPosition_;
  vector_t upperBodyKp_;
  vector_t upperBodyKd_;

  // Mapping from upper body joint names to indices in jointNameInControl_
  std::vector<int> upperBodyJointIndices_;
};

}  // namespace legged
