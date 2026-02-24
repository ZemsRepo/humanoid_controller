//
// Author: Qiayuan Liao
// Organization: Hybrid Robotics
// Source: https://github.com/HybridRobotics/motion_tracking_controller
//

#include "humanoid_controller/tasks/motion_tracking/Controller.h"

#include "humanoid_controller/tasks/motion_tracking/Command.h"
#include "humanoid_controller/tasks/motion_tracking/Observation.h"

namespace legged {
controller_interface::CallbackReturn MotionTrackingController::on_init() {
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

controller_interface::CallbackReturn MotionTrackingController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  const auto policyPath = get_node()->get_parameter("policy.path").as_string();
  const auto startStep = static_cast<size_t>(get_node()->get_parameter("motion.start_step").as_int());
  nextControllerName_ = get_node()->get_parameter("motion.next_controller").as_string();
  cfg_.loop_motion = get_node()->get_parameter("motion.loop_motion").as_bool();

  policy_ = std::make_shared<MotionOnnxPolicy>(policyPath, startStep);
  policy_->init();

  auto policy = std::dynamic_pointer_cast<MotionOnnxPolicy>(policy_);
  cfg_.anchorBody = policy->getAnchorBodyName();
  cfg_.bodyNames = policy->getBodyNames();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("MotionTrackingController"),
                     "Load Onnx model from " << policyPath << " successfully !");
  RCLCPP_INFO_STREAM(rclcpp::get_logger("MotionTrackingController"), "Start step: " << startStep);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("MotionTrackingController"), "Loop motion: " << cfg_.loop_motion);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("MotionTrackingController"), "Next controller: " << nextControllerName_);

  // Create service client for controller switching
  if (!nextControllerName_.empty()) {
    switchControllerClient_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");
  }

  return RlController::on_configure(previous_state);
}

controller_interface::CallbackReturn MotionTrackingController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  if (RlController::on_activate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  switchRequested_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionTrackingController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  if (RlController::on_deactivate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MotionTrackingController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period) {
  auto ret = RlController::update(time, period);

  // When Motion reaches the end of the trajectory, handle the looping motion or switching to next controller.
  auto motionPolicy = std::dynamic_pointer_cast<MotionOnnxPolicy>(policy_);
  if (motionPolicy->getTimeStep() >= motionPolicy->getMotionLength()) {
    if (cfg_.loop_motion) {
      RCLCPP_INFO(get_node()->get_logger(), "Motion finished, looping motion");
      motionPolicy->resetTimeStep();
    } else if (!switchRequested_ && !nextControllerName_.empty()) {
      RCLCPP_INFO(get_node()->get_logger(), "Motion finished, switching to %s", nextControllerName_.c_str());
      requestControllerSwitch();
    }
  }

  return ret;
}

void MotionTrackingController::requestControllerSwitch() {
  if (!switchControllerClient_ || switchRequested_) {
    return;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = {nextControllerName_};
  request->deactivate_controllers = {get_node()->get_name()};
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

  switchControllerClient_->async_send_request(
      request, [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        auto result = future.get();
        if (result->ok) {
          RCLCPP_INFO(get_node()->get_logger(), "Successfully switched to %s", nextControllerName_.c_str());
        } else {
          RCLCPP_ERROR(get_node()->get_logger(), "Failed to switch to %s", nextControllerName_.c_str());
        }
      });

  switchRequested_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Motion finished, requesting switch to %s", nextControllerName_.c_str());
}

bool MotionTrackingController::parserCommand(const std::string& name) {
  if (RlController::parserCommand(name)) {
    return true;
  }
  if (name == "motion") {
    commandTerm_ = std::make_shared<MotionCommandTerm>(cfg_, std::dynamic_pointer_cast<MotionOnnxPolicy>(policy_));
    commandManager_->addTerm(commandTerm_);
    return true;
  }
  return false;
}

bool MotionTrackingController::parserObservation(const std::string& name) {
  if (RlController::parserObservation(name)) {
    return true;
  }
  if (name == "motion_ref_pos_b" || name == "motion_anchor_pos_b") {
    observationManager_->addTerm(std::make_shared<MotionAnchorPosition>(commandTerm_));
  } else if (name == "motion_ref_ori_b" || name == "motion_anchor_ori_b") {
    observationManager_->addTerm(std::make_shared<MotionAnchorOrientation>(commandTerm_));
  } else if (name == "robot_body_pos") {
    observationManager_->addTerm(std::make_shared<RobotBodyPosition>(commandTerm_));
  } else if (name == "robot_body_ori") {
    observationManager_->addTerm(std::make_shared<RobotBodyOrientation>(commandTerm_));
  } else {
    return false;
  }
  return true;
}

}  // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(legged::MotionTrackingController, controller_interface::ControllerInterface)
