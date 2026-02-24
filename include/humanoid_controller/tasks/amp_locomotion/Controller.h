//
// Author: Zitong Bai
// Organization: BUAA
// Source: https://github.com/zitongbai/legged_lab
//

#pragma once

#include <legged_rl_controllers/RlController.h>

namespace legged {

/**
 * @brief AMP Locomotion Controller
 *
 * Controller for AMP (Adversarial Motion Priors) trained locomotion policies.
 * Features:
 * - Simple MLP policy with observation history
 * - Uses ObservationManager's built-in history buffer
 * - Supports key body position observations
 */
class AmpLocomotionController : public RlController {
 public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  bool parserCommand(const std::string& name) override;
  bool parserObservation(const std::string& name) override;

  // Velocity command term (subscribed via ROS topic)
  std::shared_ptr<VelocityTopicCommandTerm> velocityCommandTerm_;

  // Key body names for key_body_pos_b observation
  std::vector<std::string> keyBodyNames_;
};

}  // namespace legged
