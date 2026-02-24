//
// Author: Qiayuan Liao
// Organization: Hybrid Robotics
// Source: https://github.com/HybridRobotics/motion_tracking_controller
//

#pragma once

#include <legged_rl_controllers/RlController.h>

#include <controller_manager_msgs/srv/switch_controller.hpp>

#include "humanoid_controller/tasks/motion_tracking/Command.h"

namespace legged {
class MotionTrackingController : public RlController {
 public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  bool parserCommand(const std::string& name) override;
  bool parserObservation(const std::string& name) override;

  void requestControllerSwitch();

  MotionCommandCfg cfg_;
  MotionCommandTerm::SharedPtr commandTerm_;

  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switchControllerClient_;
  std::string nextControllerName_;
  bool switchRequested_ = false;
};

}  // namespace legged
