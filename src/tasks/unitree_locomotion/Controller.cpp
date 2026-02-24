//
// Author: Unitree Robotics
// Organization: Unitree Robotics
// Source: https://github.com/unitreerobotics/unitree_rl_gym
//

#include "humanoid_controller/tasks/unitree_locomotion/Controller.h"

#include <legged_rl_controllers/ObservationManager.h>

#include <algorithm>
#include <set>

#include "humanoid_controller/tasks/unitree_locomotion/Observation.h"

// Direct access to command_interfaces_ for upper body joints
// Each joint has 5 interfaces: position(0), velocity(1), effort(2), stiffness(3), damping(4)
constexpr size_t INTERFACES_PER_JOINT = 5;
constexpr size_t POS_OFFSET = 0;
constexpr size_t VEL_OFFSET = 1;
constexpr size_t EFF_OFFSET = 2;
constexpr size_t KP_OFFSET = 3;
constexpr size_t KD_OFFSET = 4;

namespace legged {

controller_interface::CallbackReturn UnitreeLocomotionController::on_init() {
  if (RlController::on_init() != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    // Declare unitree-specific parameters
    auto_declare<double>("gait_period", 0.8);

    // Declare upper body joint parameters
    auto_declare<std::vector<std::string>>("upper_body.joint_names", std::vector<std::string>{});
    auto_declare<std::vector<double>>("upper_body.default_position", std::vector<double>{});
    auto_declare<std::vector<double>>("upper_body.kp", std::vector<double>{});
    auto_declare<std::vector<double>>("upper_body.kd", std::vector<double>{});
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreeLocomotionController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  // Load policy path from parameters
  const auto policyPath = get_node()->get_parameter("policy.path").as_string();
  cfg_.gait_period = get_node()->get_parameter("gait_period").as_double();

  // Create and initialize policy
  unitreePolicy_ = std::make_shared<UnitreeLocomotionPolicy>(policyPath);
  unitreePolicy_->init();
  policy_ = unitreePolicy_;  // Assign to base class pointer

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Loaded UnitreeLocomotionPolicy from " << policyPath);
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  Observation size: " << policy_->getObservationSize());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  Action size: " << policy_->getActionSize());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  Recurrent: " << (unitreePolicy_->isRecurrent() ? "yes" : "no"));
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  Gait period: " << cfg_.gait_period);

  // Load upper body joint configuration
  upperBodyJointNames_ = get_node()->get_parameter("upper_body.joint_names").as_string_array();

  if (!upperBodyJointNames_.empty()) {
    auto defaultPosVec = get_node()->get_parameter("upper_body.default_position").as_double_array();
    auto kpVec = get_node()->get_parameter("upper_body.kp").as_double_array();
    auto kdVec = get_node()->get_parameter("upper_body.kd").as_double_array();

    size_t numUpperBody = upperBodyJointNames_.size();

    // Convert to Eigen vectors
    upperBodyDefaultPosition_.resize(numUpperBody);
    upperBodyKp_.resize(numUpperBody);
    upperBodyKd_.resize(numUpperBody);

    for (size_t i = 0; i < numUpperBody; ++i) {
      upperBodyDefaultPosition_[i] = (i < defaultPosVec.size()) ? defaultPosVec[i] : 0.0;
      upperBodyKp_[i] = (i < kpVec.size()) ? kpVec[i] : 100.0;
      upperBodyKd_[i] = (i < kdVec.size()) ? kdVec[i] : 5.0;
    }

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "  Upper body joints: " << numUpperBody);
  }

  // Call base class configure first
  auto result = RlController::on_configure(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  // Add upper body joints to jointNameInControl_ so they are included in command_interface_configuration
  // This must be done AFTER RlController::on_configure which sets up the lower body joints
  for (const auto& jointName : upperBodyJointNames_) {
    // Check if not already in the list
    if (std::find(jointNameInControl_.begin(), jointNameInControl_.end(), jointName) == jointNameInControl_.end()) {
      jointNameInControl_.push_back(jointName);
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "Added upper body joint to control: " << jointName);
    }
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Total joints in control: " << jointNameInControl_.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreeLocomotionController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  auto result = RlController::on_activate(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  // Build mapping from upper body joint names to indices in jointNameInControl_
  upperBodyJointIndices_.clear();
  for (const auto& name : upperBodyJointNames_) {
    auto it = std::find(jointNameInControl_.begin(), jointNameInControl_.end(), name);
    if (it != jointNameInControl_.end()) {
      int idx = static_cast<int>(std::distance(jointNameInControl_.begin(), it));
      upperBodyJointIndices_.push_back(idx);
      RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Upper body joint " << name << " -> index " << idx);
    } else {
      RCLCPP_WARN_STREAM(get_node()->get_logger(), "Upper body joint " << name << " not found in control joints");
      upperBodyJointIndices_.push_back(-1);  // Mark as not found
    }
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Upper body joint indices mapped: " << upperBodyJointIndices_.size());

  // Set stiffness and damping for all joints
  // Combine lower body (from policy) and upper body (from config)
  if (!upperBodyJointIndices_.empty()) {
    size_t totalJoints = jointNameInControl_.size();
    vector_t fullKp = vector_t::Zero(totalJoints);
    vector_t fullKd = vector_t::Zero(totalJoints);

    // First, set lower body gains from policy
    const auto& policyJointNames = policy_->getJointNames();
    const auto& policyKp = policy_->getJointStiffness();
    const auto& policyKd = policy_->getJointDamping();

    for (size_t i = 0; i < policyJointNames.size(); ++i) {
      auto it = std::find(jointNameInControl_.begin(), jointNameInControl_.end(), policyJointNames[i]);
      if (it != jointNameInControl_.end()) {
        int idx = static_cast<int>(std::distance(jointNameInControl_.begin(), it));
        fullKp[idx] = policyKp[i];
        fullKd[idx] = policyKd[i];
      }
    }

    // Then, set upper body gains from config
    for (size_t i = 0; i < upperBodyJointIndices_.size(); ++i) {
      int idx = upperBodyJointIndices_[i];
      if (idx >= 0 && idx < static_cast<int>(totalJoints)) {
        fullKp[idx] = upperBodyKp_[i];
        fullKd[idx] = upperBodyKd_[i];
      }
    }

    setStiffnesses(fullKp);
    setDampings(fullKd);

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Set combined Kp/Kd for " << totalJoints << " joints");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type UnitreeLocomotionController::update(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period) {
  if (gaitCommandTerm_) {
    gaitCommandTerm_->update();
  }

  // Call base class update
  auto result = RlController::update(time, period);

  for (size_t i = 0; i < upperBodyJointIndices_.size(); ++i) {
    int idx = upperBodyJointIndices_[i];
    if (idx >= 0) {
      size_t baseIdx = static_cast<size_t>(idx) * INTERFACES_PER_JOINT;
      if (baseIdx + KD_OFFSET < command_interfaces_.size()) {
        command_interfaces_[baseIdx + POS_OFFSET].set_value(upperBodyDefaultPosition_[i]);
        command_interfaces_[baseIdx + VEL_OFFSET].set_value(0.0);  // Zero velocity
        command_interfaces_[baseIdx + EFF_OFFSET].set_value(0.0);  // Zero feedforward torque
        command_interfaces_[baseIdx + KP_OFFSET].set_value(upperBodyKp_[i]);
        command_interfaces_[baseIdx + KD_OFFSET].set_value(upperBodyKd_[i]);
      }
    }
  }
  return result;
}

bool UnitreeLocomotionController::parserCommand(const std::string& name) {
  if (name == "twist") {
    // Create velocity command term (subscribes to /cmd_vel)
    velocityCommandTerm_ = std::make_shared<UnitreeVelocityCommandTerm>(get_node(), "/cmd_vel");
    commandManager_->addTerm(velocityCommandTerm_);

    // Create gait phase command term
    gaitCommandTerm_ = std::make_shared<UnitreeGaitCommandTerm>(cfg_.gait_period, 0.005);
    commandManager_->addTerm(gaitCommandTerm_);

    return true;
  }

  return false;
}

bool UnitreeLocomotionController::parserObservation(const std::string& name) {
  if (name == "ang_vel") {
    // Base angular velocity, scaled
    observationManager_->addTerm(std::make_shared<UnitreeBaseAngVelObservationTerm>(unitreePolicy_->getAngVelScale()));

  } else if (name == "gravity") {
    // Projected gravity
    observationManager_->addTerm(std::make_shared<UnitreeProjectedGravityObservationTerm>());

  } else if (name == "cmd") {
    // Command velocity [vx, vy, wz], scaled
    observationManager_->addTerm(
        std::make_shared<UnitreeCommandVelocityObservationTerm>(velocityCommandTerm_, unitreePolicy_->getCmdScale()));

  } else if (name == "dof_pos") {
    // Joint positions (relative to default), scaled
    observationManager_->addTerm(std::make_shared<UnitreeJointPositionsObservationTerm>(
        policy_->getJointNames(), policy_->getDefaultJointPositions(), unitreePolicy_->getDofPosScale()));

  } else if (name == "dof_vel") {
    // Joint velocities, scaled
    observationManager_->addTerm(std::make_shared<UnitreeJointVelocitiesObservationTerm>(
        policy_->getJointNames(), unitreePolicy_->getDofVelScale()));

  } else if (name == "actions") {
    // Last action
    observationManager_->addTerm(std::make_shared<UnitreeLastActionObservationTerm>(policy_));

  } else if (name == "sin_phase") {
    // Sine phase
    observationManager_->addTerm(std::make_shared<UnitreeSinePhaseObservationTerm>(gaitCommandTerm_));

  } else if (name == "cos_phase") {
    // Cosine phase
    observationManager_->addTerm(std::make_shared<UnitreeCosinePhaseObservationTerm>(gaitCommandTerm_));

  } else if (name == "phase") {
    // Combined phase observation (sin + cos) - for backwards compatibility
    observationManager_->addTerm(std::make_shared<UnitreeSinePhaseObservationTerm>(gaitCommandTerm_));
    observationManager_->addTerm(std::make_shared<UnitreeCosinePhaseObservationTerm>(gaitCommandTerm_));

  } else {
    return false;
  }

  return true;
}

}  // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(legged::UnitreeLocomotionController, controller_interface::ControllerInterface)
