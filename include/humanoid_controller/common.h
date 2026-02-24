//
// Author: Qiayuan Liao
// Organization: Hybrid Robotics
// Source: https://github.com/HybridRobotics/motion_tracking_controller
//

#pragma once

#include <legged_model/common.h>

namespace legged {

inline vector_t rotationToVectorWxyz(const quaternion_t& ori) {
  vector_t vec(4);
  vec(0) = ori.w();
  vec.segment(1, 3) = ori.coeffs().head(3);
  return vec;
}

inline vector_t rotationToVectorWxyz(const matrix3_t& ori) {
  return rotationToVectorWxyz(quaternion_t(ori));
}

inline quaternion_t yawQuaternion(const quaternion_t& q) {
  scalar_t yaw = std::atan2(scalar_t(2) * (q.w() * q.z() + q.x() * q.y()),
                            scalar_t(1) - scalar_t(2) * (q.y() * q.y() + q.z() * q.z()));
  scalar_t half_yaw = yaw * scalar_t(0.5);
  quaternion_t ret(std::cos(half_yaw), scalar_t(0), scalar_t(0), std::sin(half_yaw));
  return ret.normalized();
}

}  // namespace legged
