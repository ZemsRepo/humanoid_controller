//
// Author: Zitong Bai
// Organization: BUAA
// Source: https://github.com/zitongbai/legged_lab
//

#include "humanoid_controller/tasks/amp_locomotion/Observation.h"

#include <cmath>
#include <iostream>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace legged {

// ============================================================================
// Helper functions for quaternion operations
// ============================================================================

namespace {

/**
 * @brief Extract yaw quaternion from a full quaternion
 * @param quat Input quaternion (w, x, y, z)
 * @return Yaw-only quaternion
 */
quaternion_t yawQuat(const quaternion_t& quat) {
  // Extract yaw angle from quaternion
  scalar_t yaw = std::atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()),
                            1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
  // Create yaw-only quaternion
  return quaternion_t(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

/**
 * @brief Apply inverse quaternion rotation to a vector
 * @param quat Quaternion
 * @param vec Vector to rotate
 * @return Rotated vector
 */
vector3_t quatApplyInverse(const quaternion_t& quat, const vector3_t& vec) {
  return quat.conjugate() * vec;
}

}  // namespace

// ============================================================================
// RootLocalRotTanNormObservationTerm
// ============================================================================

vector_t RootLocalRotTanNormObservationTerm::evaluate() {
  // Get root quaternion
  const quaternion_t& rootQuat = model_->getBaseRotation();

  // Extract yaw quaternion
  quaternion_t yawQ = yawQuat(rootQuat);

  // Compute local quaternion (yaw-removed)
  quaternion_t localQuat = yawQ.conjugate() * rootQuat;

  // Convert to rotation matrix
  matrix3_t rotMat = localQuat.toRotationMatrix();

  // Extract tangent (first column) and normal (last column)
  vector3_t tanVec = rotMat.col(0);
  vector3_t normVec = rotMat.col(2);

  // Concatenate: [tan_x, tan_y, tan_z, norm_x, norm_y, norm_z]
  vector_t obs(6);
  obs.head<3>() = tanVec;
  obs.tail<3>() = normVec;

  return obs;
}

// ============================================================================
// AmpBaseAngVelObservationTerm
// ============================================================================

vector_t AmpBaseAngVelObservationTerm::evaluate() {
  // Angular velocity in base frame (wx, wy, wz)
  return model_->getGeneralizedVelocity().segment<3>(3);
}

// ============================================================================
// AmpCommandVelocityObservationTerm
// ============================================================================

vector_t AmpCommandVelocityObservationTerm::evaluate() {
  // Get command: [vx, vy, wz]
  return commandTerm_->getValue();
}

// ============================================================================
// AmpJointPositionsObservationTerm
// ============================================================================

vector_t AmpJointPositionsObservationTerm::evaluate() {
  // Get all joint positions (absolute, not relative to default)
  return model_->getGeneralizedPosition().tail(model_->getJointNames().size());
}

// ============================================================================
// AmpJointVelocitiesObservationTerm
// ============================================================================

vector_t AmpJointVelocitiesObservationTerm::evaluate() {
  // Get all joint velocities
  return model_->getGeneralizedVelocity().tail(model_->getJointNames().size());
}

// ============================================================================
// KeyBodyPosBObservationTerm
// ============================================================================

void KeyBodyPosBObservationTerm::setModel(const LeggedModel::SharedPtr& model) {
  ObservationTerm::setModel(model);

  // Get frame indices for key bodies
  keyBodyIds_.clear();
  const auto& pinModel = model_->getPinModel();

  for (const auto& bodyName : keyBodyNames_) {
    bool found = false;
    for (size_t i = 0; i < pinModel.frames.size(); ++i) {
      if (pinModel.frames[i].name == bodyName) {
        keyBodyIds_.push_back(i);
        found = true;
        break;
      }
    }
    if (!found) {
      std::cerr << "Warning: Key body '" << bodyName << "' not found in model" << std::endl;
    }
  }

  std::cout << "KeyBodyPosBObservationTerm initialized with " << keyBodyIds_.size() << " key bodies" << std::endl;
}

vector_t KeyBodyPosBObservationTerm::evaluate() {
  const size_t numKeyBodies = keyBodyIds_.size();
  vector_t obs(numKeyBodies * 3);

  if (numKeyBodies == 0) {
    return obs;
  }

  // Get root position and orientation
  const vector_t& q = model_->getGeneralizedPosition();
  vector3_t rootPosW = q.head<3>();
  quaternion_t rootQuat = model_->getBaseRotation();

  // Update kinematics to get frame positions
  auto& pinData = model_->getPinData();
  const auto& pinModel = model_->getPinModel();
  pinocchio::framesForwardKinematics(pinModel, pinData, q);

  // Compute each key body position in body frame
  for (size_t i = 0; i < numKeyBodies; ++i) {
    size_t frameId = keyBodyIds_[i];

    // Get world position of key body
    vector3_t keyBodyPosW = pinData.oMf[frameId].translation();

    // Transform to body frame: key_body_pos_b = quat_apply_inverse(root_quat, key_body_pos_w - root_pos_w)
    vector3_t keyBodyPosB = quatApplyInverse(rootQuat, keyBodyPosW - rootPosW);

    obs.segment<3>(i * 3) = keyBodyPosB;
  }

  return obs;
}

// ============================================================================
// AmpLastActionObservationTerm
// ============================================================================

vector_t AmpLastActionObservationTerm::evaluate() {
  return policy_->getLastAction();
}

}  // namespace legged
