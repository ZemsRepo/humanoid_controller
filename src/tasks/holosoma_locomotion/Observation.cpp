//
// Author: Amazon FAR
// Organization: Amazon FAR
// Source: https://github.com/amazon-far/holosoma
//

#include "humanoid_controller/tasks/holosoma_locomotion/Observation.h"

namespace legged {

// ScaledBaseAngVelObservationTerm
vector_t ScaledBaseAngVelObservationTerm::evaluate() {
  vector_t omega = model_->getGeneralizedVelocity().segment<3>(3);
  return omega;
}

vector_t ScaledBaseAngVelObservationTerm::modify(const vector_t& observation) {
  return observation * scale_;
}

// CommandLinearVelocityObservationTerm
vector_t CommandLinearVelocityObservationTerm::evaluate() {
  vector_t cmd_lin_vel = commandTerm_->getValue().head(2);
  return cmd_lin_vel;
}

vector_t CommandLinearVelocityObservationTerm::modify(const vector_t& observation) {
  return observation * scale_;
}

// CommandAngularVelocityObservationTerm
vector_t CommandAngularVelocityObservationTerm::evaluate() {
  vector_t cmd_ang_vel = vector_t::Zero(1);
  cmd_ang_vel[0] = commandTerm_->getValue()[2];
  return cmd_ang_vel;
}

vector_t CommandAngularVelocityObservationTerm::modify(const vector_t& observation) {
  return observation * scale_;
}

// ScaledJointPositionsObservationTerm
vector_t ScaledJointPositionsObservationTerm::evaluate() {
  return model_->getGeneralizedPosition().tail(model_->getJointNames().size());
}

vector_t ScaledJointPositionsObservationTerm::modify(const vector_t& observation) {
  return (JointObservationTerm::modify(observation) - defaultPosition_) * scale_;
}

// ScaledJointVelocitiesObservationTerm
vector_t ScaledJointVelocitiesObservationTerm::evaluate() {
  return model_->getGeneralizedVelocity().tail(model_->getJointNames().size());
}

vector_t ScaledJointVelocitiesObservationTerm::modify(const vector_t& observation) {
  return JointObservationTerm::modify(observation) * scale_;
}

// CosinePhaseObservationTerm
vector_t CosinePhaseObservationTerm::evaluate() {
  vector_t phase = commandTerm_->getValue();
  vector_t obs = vector_t::Zero(2);
  obs[0] = std::cos(phase[0]);
  obs[1] = std::cos(phase[1]);
  return obs;
}

vector_t CosinePhaseObservationTerm::modify(const vector_t& observation) {
  return observation;
}

// SinePhaseObservationTerm
vector_t SinePhaseObservationTerm::evaluate() {
  vector_t phase = commandTerm_->getValue();
  vector_t obs = vector_t::Zero(2);
  obs[0] = std::sin(phase[0]);
  obs[1] = std::sin(phase[1]);
  return obs;
}

vector_t SinePhaseObservationTerm::modify(const vector_t& observation) {
  return observation;
}
}  // namespace legged