//
// Author: Zitong Bai
// Organization: BUAA
// Source: https://github.com/zitongbai/legged_lab
//

#pragma once

#include <legged_model/common.h>
#include <legged_rl_controllers/ObservationManager.h>

namespace legged {

/**
 * @brief Root local rotation tangent-normal observation
 *
 * Computes the tangent and normal vectors from the local (yaw-removed) rotation.
 * This is computed by:
 * 1. Extract yaw from root quaternion
 * 2. Compute local quaternion = conj(yaw_quat) * root_quat
 * 3. Convert to rotation matrix
 * 4. Extract first column (tangent) and last column (normal)
 *
 * Size: 6 (tan_x, tan_y, tan_z, norm_x, norm_y, norm_z)
 */
class RootLocalRotTanNormObservationTerm final : public ObservationTerm {
 public:
  using ObservationTerm::ObservationTerm;
  size_t getSize() const override {
    return 6;
  }

 protected:
  vector_t evaluate() override;
};

/**
 * @brief Base angular velocity observation
 * Size: 3 (wx, wy, wz in base frame)
 */
class AmpBaseAngVelObservationTerm final : public ObservationTerm {
 public:
  using ObservationTerm::ObservationTerm;
  size_t getSize() const override {
    return 3;
  }

 protected:
  vector_t evaluate() override;
};

/**
 * @brief Command velocity observation (vx, vy, wz)
 * Size: 3
 */
class AmpCommandVelocityObservationTerm final : public ObservationTerm {
 public:
  explicit AmpCommandVelocityObservationTerm(const VelocityTopicCommandTerm::SharedPtr& commandTerm)
      : commandTerm_(commandTerm) {}
  size_t getSize() const override {
    return 3;
  }

 protected:
  vector_t evaluate() override;

  VelocityTopicCommandTerm::SharedPtr commandTerm_;
};

/**
 * @brief Joint positions observation (absolute, not relative to default)
 * Size: num_joints
 */
class AmpJointPositionsObservationTerm final : public JointObservationTerm {
 public:
  explicit AmpJointPositionsObservationTerm(const std::vector<std::string>& jointNames)
      : JointObservationTerm(jointNames) {}

 protected:
  vector_t evaluate() override;
};

/**
 * @brief Joint velocities observation
 * Size: num_joints
 */
class AmpJointVelocitiesObservationTerm final : public JointObservationTerm {
 public:
  explicit AmpJointVelocitiesObservationTerm(const std::vector<std::string>& jointNames)
      : JointObservationTerm(jointNames) {}

 protected:
  vector_t evaluate() override;
};

/**
 * @brief Key body positions in body frame observation
 *
 * Computes positions of key bodies (e.g., hands, feet, head) relative to
 * the root body, transformed to body frame.
 *
 * key_body_pos_b = quat_apply_inverse(root_quat, key_body_pos_w - root_pos_w)
 *
 * Size: num_key_bodies * 3
 */
class KeyBodyPosBObservationTerm final : public ObservationTerm {
 public:
  explicit KeyBodyPosBObservationTerm(const std::vector<std::string>& keyBodyNames) : keyBodyNames_(keyBodyNames) {}

  size_t getSize() const override {
    return keyBodyNames_.size() * 3;
  }

  void setModel(const LeggedModel::SharedPtr& model) override;

 protected:
  vector_t evaluate() override;

  std::vector<std::string> keyBodyNames_;
  std::vector<size_t> keyBodyIds_;
};

/**
 * @brief Last action observation (actions from previous step)
 * Size: num_actions
 */
class AmpLastActionObservationTerm final : public ObservationTerm {
 public:
  explicit AmpLastActionObservationTerm(const Policy::SharedPtr& policy) : policy_(policy) {}
  size_t getSize() const override {
    return policy_->getActionSize();
  }

 protected:
  vector_t evaluate() override;

  Policy::SharedPtr policy_;
};

}  // namespace legged
