//
// Author: Zitong Bai
// Organization: BUAA
// Source: https://github.com/zitongbai/legged_lab
//

#include "humanoid_controller/tasks/amp_locomotion/Controller.h"

#include <legged_rl_controllers/ObservationManager.h>

#include "humanoid_controller/tasks/amp_locomotion/Observation.h"
#include "humanoid_controller/tasks/amp_locomotion/Policy.h"

namespace legged {

controller_interface::CallbackReturn AmpLocomotionController::on_init() {
  if (RlController::on_init() != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    // Declare AMP-specific parameters
    auto_declare<std::vector<std::string>>("key_body_names", std::vector<std::string>{});
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AmpLocomotionController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  // Get policy path
  const auto policyPath = get_node()->get_parameter("policy.path").as_string();

  // Create AMP policy
  auto ampPolicy = std::make_shared<AmpLocomotionPolicy>(policyPath);
  ampPolicy->init();
  policy_ = ampPolicy;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("AmpLocomotionController"),
                     "Loaded AmpLocomotionPolicy from " << policyPath << " successfully!");
  RCLCPP_INFO_STREAM(rclcpp::get_logger("AmpLocomotionController"),
                     "Observation size: " << policy_->getObservationSize());
  RCLCPP_INFO_STREAM(rclcpp::get_logger("AmpLocomotionController"), "Action size: " << policy_->getActionSize());

  // Get key body names for key_body_pos_b observation
  keyBodyNames_ = get_node()->get_parameter("key_body_names").as_string_array();
  if (!keyBodyNames_.empty()) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("AmpLocomotionController"),
                       "Key body names: " << keyBodyNames_.size() << " bodies");
  }

  // Pre-create velocity command term - needed before parserObservation is called by base class
  // (velocity_commands observation depends on this term)
  velocityCommandTerm_ = std::make_shared<VelocityTopicCommandTerm>(get_node(), "/cmd_vel");

  return RlController::on_configure(previous_state);
}

// controller_interface::CallbackReturn AmpLocomotionController::on_activate(
//     const rclcpp_lifecycle::State& previous_state) {
//   // Set observation history length from policy
//   auto ampPolicy = std::dynamic_pointer_cast<AmpLocomotionPolicy>(policy_);
//   if (ampPolicy && observationManager_) {
//     size_t historyLength = ampPolicy->getHistoryLength();
//     // Create a vector with the same history length for all observation terms
//     size_t numTerms = observationManager_->getTerms().size();
//     if (numTerms > 0) {
//       std::vector<size_t> historyLengths(numTerms, historyLength);
//       observationManager_->setHistoryLengths(historyLengths);
//       RCLCPP_INFO_STREAM(rclcpp::get_logger("AmpLocomotionController"),
//                          "Set observation history length to: " << historyLength << " for " << numTerms << " terms");
//     } else {
//       RCLCPP_WARN(rclcpp::get_logger("AmpLocomotionController"),
//                   "No observation terms found! Check if observations were parsed correctly.");
//     }
//   }

//   return RlController::on_activate(previous_state);
// }

controller_interface::return_type AmpLocomotionController::update(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period) {
  return RlController::update(time, period);
}

bool AmpLocomotionController::parserCommand(const std::string& name) {
  if (name == "twist" || name == "base_velocity") {
    // velocityCommandTerm_ is pre-created in on_configure to avoid dependency issues
    // Just add it to command manager here
    if (velocityCommandTerm_) {
      commandManager_->addTerm(velocityCommandTerm_);
    }
  } else {
    return false;
  }
  return true;
}

bool AmpLocomotionController::parserObservation(const std::string& name) {
  if (name == "root_local_rot_tan_norm") {
    observationManager_->addTerm(std::make_shared<RootLocalRotTanNormObservationTerm>());
  } else if (name == "base_ang_vel") {
    observationManager_->addTerm(std::make_shared<AmpBaseAngVelObservationTerm>());
  } else if (name == "velocity_commands") {
    observationManager_->addTerm(std::make_shared<AmpCommandVelocityObservationTerm>(velocityCommandTerm_));
  } else if (name == "joint_pos") {
    observationManager_->addTerm(std::make_shared<AmpJointPositionsObservationTerm>(policy_->getJointNames()));
  } else if (name == "joint_vel") {
    observationManager_->addTerm(std::make_shared<AmpJointVelocitiesObservationTerm>(policy_->getJointNames()));
  } else if (name == "actions") {
    observationManager_->addTerm(std::make_shared<AmpLastActionObservationTerm>(policy_));
  } else if (name == "key_body_pos_b") {
    if (keyBodyNames_.empty()) {
      RCLCPP_WARN(get_node()->get_logger(), "key_body_pos_b observation requested but no key_body_names configured");
      return false;
    }
    observationManager_->addTerm(std::make_shared<KeyBodyPosBObservationTerm>(keyBodyNames_));
  } else if (name == "projected_gravity") {
    observationManager_->addTerm(std::make_shared<ProjectedGravityObservationTerm>());
  } else {
    return false;
  }

  return true;
}

}  // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(legged::AmpLocomotionController, controller_interface::ControllerInterface)
