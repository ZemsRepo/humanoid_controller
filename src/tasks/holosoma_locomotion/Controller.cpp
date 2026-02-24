//
// Author: Amazon FAR
// Organization: Amazon FAR
// Source: https://github.com/amazon-far/holosoma
//

#include "humanoid_controller/tasks/holosoma_locomotion/Controller.h"

#include <legged_rl_controllers/ObservationManager.h>

#include "humanoid_controller/tasks/holosoma_locomotion/Command.h"
#include "humanoid_controller/tasks/holosoma_locomotion/Observation.h"
#include "humanoid_controller/tasks/holosoma_locomotion/Policy.h"

namespace legged {
controller_interface::CallbackReturn HolosomaLocomotionController::on_init() {
  if (RlController::on_init() != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    auto_declare<int>("motion.start_step", 0);
    auto_declare<std::string>("motion.next_controller", "");
    auto_declare<bool>("motion.loop_motion", false);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HolosomaLocomotionController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  const auto policyPath = get_node()->get_parameter("policy.path").as_string();
  cfg_.gait_period = get_node()->get_parameter("gait_period").as_double();

  policy_ = std::make_shared<HolosomaLocomotionPolicy>(policyPath);
  policy_->init();

  RCLCPP_INFO_STREAM(rclcpp::get_logger("HolosomaLocomotionController"),
                     "Loaded HolosomaLocomotionPolicy from " << policyPath << " successfully!");
  RCLCPP_INFO_STREAM(rclcpp::get_logger("HolosomaLocomotionController"), "Gait Period: " << cfg_.gait_period);

  return RlController::on_configure(previous_state);
}

controller_interface::return_type HolosomaLocomotionController::update(const rclcpp::Time& time,
                                                                       const rclcpp::Duration& period) {
  vector_t command_vel = twistCommandTerm_->getValue();
  locomotionCommandTerm_->update(command_vel, period.seconds());
  return RlController::update(time, period);
}

bool HolosomaLocomotionController::parserCommand(const std::string& name) {
  if (name == "twist") {
    locomotionCommandTerm_ = std::make_shared<LocomotionGaitCommandTerm>(cfg_.gait_period);
    commandManager_->addTerm(locomotionCommandTerm_);
    twistCommandTerm_ = std::make_shared<VelocityTopicCommandTerm>(get_node(), "/cmd_vel");
    commandManager_->addTerm(twistCommandTerm_);
  } else {
    return false;
  }
  return true;
}

bool HolosomaLocomotionController::parserObservation(const std::string& name) {
  if (name == "base_ang_vel") {
    observationManager_->addTerm(std::make_shared<ScaledBaseAngVelObservationTerm>());
  } else if (name == "projected_gravity") {
    observationManager_->addTerm(std::make_shared<ProjectedGravityObservationTerm>());
  } else if (name == "command_lin_vel") {
    observationManager_->addTerm(std::make_shared<CommandLinearVelocityObservationTerm>(twistCommandTerm_));
  } else if (name == "command_ang_vel") {
    observationManager_->addTerm(std::make_shared<CommandAngularVelocityObservationTerm>(twistCommandTerm_));
  } else if (name == "dof_pos") {
    observationManager_->addTerm(std::make_shared<ScaledJointPositionsObservationTerm>(
        policy_->getJointNames(), policy_->getDefaultJointPositions()));
  } else if (name == "dof_vel") {
    observationManager_->addTerm(std::make_shared<ScaledJointVelocitiesObservationTerm>(policy_->getJointNames()));
  } else if (name == "actions") {
    observationManager_->addTerm(std::make_shared<LastActionObservationTerm>(policy_));
  } else if (name == "sin_phase") {
    observationManager_->addTerm(std::make_shared<SinePhaseObservationTerm>(locomotionCommandTerm_));
  } else if (name == "cos_phase") {
    observationManager_->addTerm(std::make_shared<CosinePhaseObservationTerm>(locomotionCommandTerm_));
  } else {
    return false;
  }

  return true;
}

}  // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(legged::HolosomaLocomotionController, controller_interface::ControllerInterface)
