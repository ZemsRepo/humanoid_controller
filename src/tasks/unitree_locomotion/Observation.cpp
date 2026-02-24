//
// Author: Unitree Robotics
// Organization: Unitree Robotics
// Source: https://github.com/unitreerobotics/unitree_rl_gym
//

#include "humanoid_controller/tasks/unitree_locomotion/Observation.h"

#include <cmath>

namespace legged {

// ============================================================================
// UnitreeBaseAngVelObservationTerm
// ============================================================================

vector_t UnitreeBaseAngVelObservationTerm::evaluate() {
  // Angular velocity in base frame (wx, wy, wz)
  return model_->getGeneralizedVelocity().segment<3>(3);
}

vector_t UnitreeBaseAngVelObservationTerm::modify(const vector_t& observation) {
  return observation * scale_;
}

// ============================================================================
// UnitreeProjectedGravityObservationTerm
// ============================================================================

vector_t UnitreeProjectedGravityObservationTerm::evaluate() {
  // Get base orientation quaternion
  // Use the same formula as unitree_rl_gym deploy_mujoco.py
  const quaternion_t& quat = model_->getBaseRotation();

  // Quaternion components: Eigen stores as [x, y, z, w] internally
  // but quaternion.w(), .x(), .y(), .z() give correct values
  scalar_t qw = quat.w();
  scalar_t qx = quat.x();
  scalar_t qy = quat.y();
  scalar_t qz = quat.z();

  // Formula from unitree_rl_gym/deploy/deploy_mujoco/deploy_mujoco.py
  // This computes projected gravity in base frame
  vector_t gravity_orientation(3);
  gravity_orientation[0] = 2.0 * (-qz * qx + qw * qy);
  gravity_orientation[1] = -2.0 * (qz * qy + qw * qx);
  gravity_orientation[2] = 1.0 - 2.0 * (qw * qw + qz * qz);

  return gravity_orientation;
}

// ============================================================================
// UnitreeCommandVelocityObservationTerm
// ============================================================================

vector_t UnitreeCommandVelocityObservationTerm::evaluate() {
  // Get command: [vx, vy, wz]
  return commandTerm_->getValue();
}

vector_t UnitreeCommandVelocityObservationTerm::modify(const vector_t& observation) {
  // Apply per-component scaling
  vector_t scaled = observation;
  for (Eigen::Index i = 0; i < std::min(observation.size(), cmdScale_.size()); ++i) {
    scaled[i] *= cmdScale_[i];
  }
  return scaled;
}

// ============================================================================
// UnitreeJointPositionsObservationTerm
// ============================================================================

vector_t UnitreeJointPositionsObservationTerm::evaluate() {
  // Get all joint positions
  return model_->getGeneralizedPosition().tail(model_->getJointNames().size());
}

vector_t UnitreeJointPositionsObservationTerm::modify(const vector_t& observation) {
  // Reorder to policy joint order and subtract default position
  return (JointObservationTerm::modify(observation) - defaultPositions_) * scale_;
}

// ============================================================================
// UnitreeJointVelocitiesObservationTerm
// ============================================================================

vector_t UnitreeJointVelocitiesObservationTerm::evaluate() {
  // Get all joint velocities
  return model_->getGeneralizedVelocity().tail(model_->getJointNames().size());
}

vector_t UnitreeJointVelocitiesObservationTerm::modify(const vector_t& observation) {
  // Reorder to policy joint order and scale
  return JointObservationTerm::modify(observation) * scale_;
}

// ============================================================================
// UnitreeLastActionObservationTerm
// ============================================================================

vector_t UnitreeLastActionObservationTerm::evaluate() {
  return policy_->getLastAction();
}

// ============================================================================
// UnitreeSinePhaseObservationTerm
// ============================================================================

vector_t UnitreeSinePhaseObservationTerm::evaluate() {
  // Get phase from command term and compute sin
  scalar_t phase = commandTerm_->getPhase();
  vector_t obs(1);
  obs[0] = std::sin(2.0 * M_PI * phase);
  return obs;
}

// ============================================================================
// UnitreeCosinePhaseObservationTerm
// ============================================================================

vector_t UnitreeCosinePhaseObservationTerm::evaluate() {
  // Get phase from command term and compute cos
  scalar_t phase = commandTerm_->getPhase();
  vector_t obs(1);
  obs[0] = std::cos(2.0 * M_PI * phase);
  return obs;
}

}  // namespace legged
