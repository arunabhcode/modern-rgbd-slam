// Copyright 2019 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_POSE_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_POSE_H_

#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"

namespace slam
{
struct Pose
{
  uint64_t m_timestamp;
  Eigen::Quaternionf m_orientation;
  Eigen::Vector3f m_position;
  Pose()
      : m_timestamp(0)
      , m_orientation(Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f))
      , m_position((Eigen::Vector3f(0.0f, 0.0f, 0.0f)))
  {
  }
  Pose(const Eigen::Quaternionf& orientation, const Eigen::Vector3f& position)
      : m_timestamp(0), m_orientation(orientation), m_position(position)
  {
  }
  Pose(const uint64_t timestamp,
       const Eigen::Quaternionf& orientation,
       const Eigen::Vector3f& position)
      : m_timestamp(timestamp), m_orientation(orientation), m_position(position)
  {
  }
  ~Pose() = default;

  Pose Inverse()
  {
    return Pose();
  }
};  // struct pose

inline std::ostream& operator<<(std::ostream& os, const Pose& pose)
{
  os << pose.m_timestamp << " " << pose.m_orientation.w() << " "
     << pose.m_orientation.x() << " " << pose.m_orientation.y() << " "
     << pose.m_orientation.z() << " " << pose.m_position[0] << " "
     << pose.m_position[1] << " " << pose.m_position[2];
  return os;
}

}  // namespace slam

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_POSE_H_
