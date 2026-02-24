//
// Author: Unitree Robotics
// Organization: Unitree Robotics
// Source: https://github.com/unitreerobotics/unitree_rl_gym
//

#pragma once

#include <legged_model/common.h>
#include <legged_rl_controllers/ObservationManager.h>

#include "humanoid_controller/tasks/unitree_locomotion/Command.h"

namespace legged {

/**
 * @brief Base angular velocity observation, scaled by ang_vel_scale
 * Size: 3 (wx, wy, wz in base frame)
 */
class UnitreeBaseAngVelObservationTerm final : public ObservationTerm {
 public:
  explicit UnitreeBaseAngVelObservationTerm(scalar_t scale = 0.25) : scale_(scale) {}
  size_t getSize() const override {
    return 3;
  }

 protected:
  vector_t evaluate() override;
  vector_t modify(const vector_t& observation) override;

  scalar_t scale_;
};

/**
 * @brief Projected gravity observation (gravity vector in base frame)
 * Size: 3 (gx, gy, gz normalized)
 */
class UnitreeProjectedGravityObservationTerm final : public ObservationTerm {
 public:
  using ObservationTerm::ObservationTerm;
  size_t getSize() const override {
    return 3;
  }

 protected:
  vector_t evaluate() override;
};

/**
 * @brief Command velocity observation (vx, vy, wz), scaled by cmd_scale
 * Size: 3
 */
class UnitreeCommandVelocityObservationTerm final : public ObservationTerm {
 public:
  UnitreeCommandVelocityObservationTerm(const UnitreeVelocityCommandTerm::SharedPtr& commandTerm,
                                        const vector_t& cmdScale)
      : commandTerm_(commandTerm), cmdScale_(cmdScale) {}
  size_t getSize() const override {
    return 3;
  }

 protected:
  vector_t evaluate() override;
  vector_t modify(const vector_t& observation) override;

  UnitreeVelocityCommandTerm::SharedPtr commandTerm_;
  vector_t cmdScale_;
};

/**
 * @brief Joint positions observation (dof_pos - default_pos), scaled
 * Size: num_actions
 */
class UnitreeJointPositionsObservationTerm final : public JointObservationTerm {
 public:
  UnitreeJointPositionsObservationTerm(const std::vector<std::string>& jointNames, const vector_t& defaultPositions,
                                       scalar_t scale = 1.0)
      : JointObservationTerm(jointNames), defaultPositions_(defaultPositions), scale_(scale) {}

 protected:
  vector_t evaluate() override;
  vector_t modify(const vector_t& observation) override;

  vector_t defaultPositions_;
  scalar_t scale_;
};

/**
 * @brief Joint velocities observation, scaled by dof_vel_scale
 * Size: num_actions
 */
class UnitreeJointVelocitiesObservationTerm final : public JointObservationTerm {
 public:
  UnitreeJointVelocitiesObservationTerm(const std::vector<std::string>& jointNames, scalar_t scale = 0.05)
      : JointObservationTerm(jointNames), scale_(scale) {}

 protected:
  vector_t evaluate() override;
  vector_t modify(const vector_t& observation) override;

  scalar_t scale_;
};

/**
 * @brief Last action observation (actions from previous step)
 * Size: num_actions
 */
class UnitreeLastActionObservationTerm final : public ObservationTerm {
 public:
  explicit UnitreeLastActionObservationTerm(const Policy::SharedPtr& policy) : policy_(policy) {}
  size_t getSize() const override {
    return policy_->getActionSize();
  }

 protected:
  vector_t evaluate() override;

  Policy::SharedPtr policy_;
};

/**
 * @brief Sine phase observation for gait timing
 * Size: 1
 */
class UnitreeSinePhaseObservationTerm final : public ObservationTerm {
 public:
  explicit UnitreeSinePhaseObservationTerm(const UnitreeGaitCommandTerm::SharedPtr& commandTerm)
      : commandTerm_(commandTerm) {}
  size_t getSize() const override {
    return 1;
  }

 protected:
  vector_t evaluate() override;

  UnitreeGaitCommandTerm::SharedPtr commandTerm_;
};

/**
 * @brief Cosine phase observation for gait timing
 * Size: 1
 */
class UnitreeCosinePhaseObservationTerm final : public ObservationTerm {
 public:
  explicit UnitreeCosinePhaseObservationTerm(const UnitreeGaitCommandTerm::SharedPtr& commandTerm)
      : commandTerm_(commandTerm) {}
  size_t getSize() const override {
    return 1;
  }

 protected:
  vector_t evaluate() override;

  UnitreeGaitCommandTerm::SharedPtr commandTerm_;
};

}  // namespace legged
