//
// Author: Amazon FAR
// Organization: Amazon FAR
// Source: https://github.com/amazon-far/holosoma
//

#pragma once

#include <legged_rl_controllers/RlController.h>

#include "humanoid_controller/tasks/holosoma_locomotion/Command.h"

namespace legged {
class HolosomaLocomotionController : public RlController {
 public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  bool parserCommand(const std::string& name) override;
  bool parserObservation(const std::string& name) override;

  WalkingCommandCfg cfg_;
  std::shared_ptr<VelocityTopicCommandTerm> twistCommandTerm_;
  std::shared_ptr<LocomotionGaitCommandTerm> locomotionCommandTerm_;
};

}  // namespace legged
