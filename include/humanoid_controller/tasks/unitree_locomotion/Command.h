//
// Author: Unitree Robotics
// Organization: Unitree Robotics
// Source: https://github.com/unitreerobotics/unitree_rl_gym
//

#pragma once

#include <legged_model/common.h>
#include <legged_rl_controllers/CommandManager.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace legged {

struct WalkingCommandCfg {
  scalar_t gait_period;
  scalar_t scale_lin_vel;
  scalar_t scale_ang_vel;
  scalar_t scale_dof_pos;
  scalar_t scale_dof_vel;
  vector3_t scale_command;
};

/**
 * @brief Velocity command from ROS topic (vx, vy, wz)
 */
class UnitreeVelocityCommandTerm : public CommandTerm {
 public:
  using Twist = geometry_msgs::msg::TwistStamped;
  UnitreeVelocityCommandTerm(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string& topicName)
      : node_(std::move(node)) {
    subscriber_ =
        node_->create_subscription<Twist>(topicName, 10, [this](const Twist::SharedPtr msg) { receivedMsg_.set(msg); });
  }

  vector_t getValue() override {
    std::shared_ptr<Twist> lastCommandMsg;
    receivedMsg_.get(lastCommandMsg);
    const auto timeNow = node_->get_clock()->now();
    if (lastCommandMsg->header.stamp.sec == 0 && lastCommandMsg->header.stamp.nanosec == 0) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "Received TwistStamped with zero timestamp, setting it to current "
                                            "time, this message will only be shown once");
      lastCommandMsg->header.stamp = timeNow;
    }
    vector_t command = vector_t::Zero(3);
    if (timeNow - lastCommandMsg->header.stamp < std::chrono::milliseconds{static_cast<int>(0.5 * 1000.0)}) {
      command << lastCommandMsg->twist.linear.x, lastCommandMsg->twist.linear.y, lastCommandMsg->twist.angular.z;
    }
    return command;
  }

  void reset() override {
    const auto twist = std::make_shared<Twist>();
    receivedMsg_.set(twist);
  }

  size_t getSize() const override {
    return 3;
  }

 protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Subscription<Twist>::SharedPtr subscriber_;
  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> receivedMsg_;
};

/**
 * @brief Gait phase command for locomotion
 *
 * Computes a single phase value [0, 1) based on gait period and time counter.
 * Used to generate sin_phase and cos_phase observations.
 */
class UnitreeGaitCommandTerm : public CommandTerm {
 public:
  using SharedPtr = std::shared_ptr<UnitreeGaitCommandTerm>;

  /**
   * @brief Construct gait command term
   * @param gait_period Gait period in seconds (e.g., 0.8)
   * @param env_dt Environment dt (1 / policy_frequency)
   */
  UnitreeGaitCommandTerm(scalar_t gait_period, scalar_t env_dt) : gaitPeriod_(gait_period), envDt_(env_dt) {}

  /**
   * @brief Update phase based on iterations
   */
  void update();

  /**
   * @brief Get current phase value [0, 1)
   */
  scalar_t getPhase() const {
    return phase_;
  }

  /**
   * @brief Get phase as vector (for compatibility)
   */
  vector_t getValue() override {
    vector_t v(1);
    v[0] = phase_;
    return v;
  }

  size_t getSize() const override {
    return 1;
  }

  void reset() override {
    phase_ = 0.0;
    counter_ = 0;
  }

 protected:
  scalar_t gaitPeriod_{0.8};
  scalar_t envDt_{0.005};
  scalar_t phase_{0.0};
  size_t counter_{0};
};

}  // namespace legged
