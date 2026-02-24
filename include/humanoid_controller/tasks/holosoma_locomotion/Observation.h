//
// Author: Amazon FAR
// Organization: Amazon FAR
// Source: https://github.com/amazon-far/holosoma
//

#pragma once

#include <legged_model/common.h>
#include <legged_rl_controllers/ObservationManager.h>

#include "humanoid_controller/tasks/holosoma_locomotion/Command.h"

namespace legged {

class ScaledBaseAngVelObservationTerm final : public ObservationTerm {
 public:
  using ObservationTerm::ObservationTerm;
  explicit ScaledBaseAngVelObservationTerm(scalar_t scale = 0.25) : scale_(scale) {}
  size_t getSize() const override {
    return 3;
  }

 protected:
  vector_t evaluate() override;
  vector_t modify(const vector_t& observation) override;

  scalar_t scale_;
};

class CommandLinearVelocityObservationTerm final : public ObservationTerm {
 public:
  using ObservationTerm::ObservationTerm;
  CommandLinearVelocityObservationTerm(const VelocityTopicCommandTerm::SharedPtr& commandTerm, scalar_t scale = 1.0)
      : commandTerm_(commandTerm), scale_(scale) {}
  size_t getSize() const override {
    return 2;
  }

 protected:
  vector_t evaluate() override;
  vector_t modify(const vector_t& observation) override;

  VelocityTopicCommandTerm::SharedPtr commandTerm_;
  scalar_t scale_;
};

class CommandAngularVelocityObservationTerm final : public ObservationTerm {
 public:
  using ObservationTerm::ObservationTerm;
  CommandAngularVelocityObservationTerm(const VelocityTopicCommandTerm::SharedPtr& commandTerm, scalar_t scale = 1.0)
      : commandTerm_(commandTerm), scale_(scale) {}
  size_t getSize() const override {
    return 1;
  }

 protected:
  vector_t evaluate() override;
  vector_t modify(const vector_t& observation) override;

  VelocityTopicCommandTerm::SharedPtr commandTerm_;
  scalar_t scale_;
};

class ScaledJointPositionsObservationTerm final : public JointObservationTerm {
 public:
  using JointObservationTerm::JointObservationTerm;
  ScaledJointPositionsObservationTerm(const std::vector<std::string>& jointNameInPolicy, vector_t defaultPosition,
                                      scalar_t scale = 1.0)
      : JointObservationTerm(jointNameInPolicy), defaultPosition_(std::move(defaultPosition)), scale_(scale) {}

 protected:
  vector_t evaluate() override;
  vector_t modify(const vector_t& observation) override;

  vector_t defaultPosition_;
  scalar_t scale_;
};

class ScaledJointVelocitiesObservationTerm final : public JointObservationTerm {
 public:
  ScaledJointVelocitiesObservationTerm(const std::vector<std::string>& jointNameInPolicy, scalar_t scale = 0.05)
      : JointObservationTerm(jointNameInPolicy), scale_(scale) {}

 protected:
  vector_t evaluate() override;
  vector_t modify(const vector_t& observation) override;

  scalar_t scale_;
};

class CosinePhaseObservationTerm final : public ObservationTerm {
 public:
  using ObservationTerm::ObservationTerm;
  CosinePhaseObservationTerm(const LocomotionGaitCommandTerm::SharedPtr& commandTerm) : commandTerm_(commandTerm) {}
  size_t getSize() const override {
    return 2;
  }

 protected:
  vector_t evaluate() override;
  vector_t modify(const vector_t& observation) override;

  LocomotionGaitCommandTerm::SharedPtr commandTerm_;
};

class SinePhaseObservationTerm final : public ObservationTerm {
 public:
  using ObservationTerm::ObservationTerm;
  SinePhaseObservationTerm(const LocomotionGaitCommandTerm::SharedPtr& commandTerm) : commandTerm_(commandTerm) {}
  size_t getSize() const override {
    return 2;
  }

 protected:
  vector_t evaluate() override;
  vector_t modify(const vector_t& observation) override;

  LocomotionGaitCommandTerm::SharedPtr commandTerm_;
};

}  // namespace legged