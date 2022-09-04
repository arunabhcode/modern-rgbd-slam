// Copyright 2018 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_ODOMETRY_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_ODOMETRY_H_

#include <string>
#include <vector>

#include "Components/Frame.h"
#include "Components/Pose.h"
#include "opencv2/opencv.hpp"

namespace room
{
class Odometry
{
 public:
  Odometry() = delete;
  explicit Odometry(const std::string show_debug);
  ~Odometry() = default;

  Pose PoseEstimationICP(Frame& frame0,
                         Frame& frame1,
                         std::vector<int>& matches,
                         Pose initial_pose = Pose());

 private:
  std::string m_show_debug;
};  // class odometry
}  // namespace room

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_ODOMETRY_H_
