// Copyright 2019 Arunabh Sharma

#ifndef MODULES_COMPONENTS_INCLUDE_COMPONENTS_POSE_H_
#define MODULES_COMPONENTS_INCLUDE_COMPONENTS_POSE_H_

#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"

namespace slam
{
struct Pose
{
  uint64_t timestamp_;
  Eigen::Quaternionf orientation_;
  Eigen::Vector3f position_;
  Pose()
      : timestamp_(0)
      , orientation_(Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f))
      , position_((Eigen::Vector3f(0.0f, 0.0f, 0.0f)))
  {
  }
  Pose(const Eigen::Quaternionf& orientation, const Eigen::Vector3f& position)
      : timestamp_(0), orientation_(orientation), position_(position)
  {
  }
  Pose(const uint64_t timestamp,
       const Eigen::Quaternionf& orientation,
       const Eigen::Vector3f& position)
      : timestamp_(timestamp), orientation_(orientation), position_(position)
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
  os << pose.timestamp_ << " " << pose.orientation_.w() << " "
     << pose.orientation_.x() << " " << pose.orientation_.y() << " "
     << pose.orientation_.z() << " " << pose.position_[0] << " "
     << pose.position_[1] << " " << pose.position_[2];
  return os;
}

}  // namespace slam

#endif  // MODULES_COMPONENTS_INCLUDE_COMPONENTS_POSE_H_
