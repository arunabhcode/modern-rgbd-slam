// Copyright 2019 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_RGBDSLAM_TRACK_H_
#define MODULES_RGBDSLAM_INCLUDE_RGBDSLAM_TRACK_H_

#include "Components/Camera.h"
#include "Components/Frame.h"
#include "Components/Map.h"
#include "Eigen/Core"

//
#include "RGBDSlam/Feature.h"

namespace slam
{
class Track
{
 public:
  Track()  = default;
  ~Track() = default;

  void Configure(const SlamParams& params);

  void UndistortFrames(Frame& frame);

  /**
   * @brief      Initializes the map points for the first frame which are
   * then later added to the map
   *
   * @param      frame  First frame
   *
   * @return     Whether initialization succeeded or not
   */
  bool Initialize(Frame& frame, Map& map);

 private:
  Feature feature_inst_;
  SlamParams params_inst_;
  Camera camera_inst_;

};  //  class motion
}  // namespace slam

#endif  // MODULES_RGBDSLAM_INCLUDE_RGBDSLAM_TRACK_H_
