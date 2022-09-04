// Copyright 2019 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_MOTION_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_MOTION_H_

#include "Eigen/Core"
#include "Room/Components/Pose.h"

namespace room
{
class Motion
{
 public:
  Motion();
  ~Motion() = default;

  // Motion(const Pose& init_pose, const uint64_t timestamp);
  Pose& GetCurrentPose();

  Pose PredictPose(const uint64_t& current_timestamp);
  void UpdatePose(const Pose& new_pose, const uint64_t& new_timestamp);

 private:
  Pose m_prev_pose;
  uint64_t m_prev_timestamp;
  bool m_is_initialized;

  Eigen::Vector3f m_v_linear;
  float m_v_angular;
  Eigen::Vector3f m_angular_axis;
  float m_damping_ratio;

};  //  class motion
}  // namespace room

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_MOTION_H_
