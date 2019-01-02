// Copyright 2019 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_POSE_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_POSE_H_

#include <iostream>

#include "Eigen/Core"

namespace room
{
struct Pose
{
  uint64_t timestamp;
  Eigen::Quaternionf orientation;
  Eigen::Vector3f position;
  Pose()
      : timestamp(0)
      , orientation(Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f))
      , position((Eigen::Vector3f(0.0f, 0.0f, 0.0f)))
  {
  }
  ~Pose() = default;
};

std::ostream& operator<<(std::ostream& os, const Pose& pose)
{
  os << timestamp << " " << pose.orientation.w() << " " << pose.orientation.x()
     << " " << pose.orientation.y() << " " << pose.orientation.z() << " "
     << pose.position[0] << " " << pose.position[1] << " " << pose.position[2];
  return os;
}

}  // namespace room

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_POSE_H_
