// Copyright 2022 Arunabh Sharma

#include "RGBDSlam/Track.h"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

namespace slam
{
void Track::Configure(const SlamParams& params)
{
  params_inst_ = params;
  feature_inst_.Configure(params);
  camera_inst_.Configure(params);
}

void Track::UndistortFrames(Frame& frame)
{
  cv::undistort(frame.color_img_,
                frame.undist_color_img_,
                camera_inst_.ToIntrinsicsMat(),
                camera_inst_.dist_);
  cv::undistort(frame.depth_img_,
                frame.undist_depth_img_,
                camera_inst_.ToIntrinsicsMat(),
                camera_inst_.dist_);
}

bool Track::Initialize(Frame& frame, Map& map)
{
  UndistortFrames(frame);
  if (!feature_inst_.FindFeatures(frame))
  {
    SPDLOG_ERROR("ORB Features not found in current frame");
    return false;
  }
  for (std::size_t i = 0; i < frame.keypoints_.size(); ++i)
  {
    float Z = frame.undist_depth_img_.at<float>(frame.keypoints_[i].x,
                                                frame.keypoints_[i].y);
    float X =
        (frame.keypoints_[i].x - camera_inst_.pp_.x) / camera_inst_.focal_ * Z;
    float Y =
        (frame.keypoints_[i].x - camera_inst_.pp_.x) / camera_inst_.focal_ * Z;

    map.AddMapPoint(MapPoint(i, 0, i, Eigen::Vector3f(X, Y, Z)));
  }
  return true;
}

}  // namespace slam