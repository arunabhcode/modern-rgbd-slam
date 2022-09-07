// Copyright 2022 Arunabh Sharma

#include "RGBDSlam/Track.h"

#include "opencv2/opencv.hpp"
#include "spdlog/fmt/ostr.h"
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
    SPDLOG_ERROR("Features not found in current frame");
    return false;
  }
  for (std::size_t i = 0; i < frame.features_.size(); ++i)
  {
    float Z = frame.undist_depth_img_.at<float>(frame.features_[i].pt.x,
                                                frame.features_[i].pt.y);
    if (Z > 0.0f)
    {
      float X = (frame.features_[i].pt.x - camera_inst_.pp_.x) /
                camera_inst_.focal_ * Z;
      float Y = (frame.features_[i].pt.x - camera_inst_.pp_.x) /
                camera_inst_.focal_ * Z;
      SPDLOG_INFO("Feature x = {}, y = {}",
                  frame.features_[i].pt.x,
                  frame.features_[i].pt.y);
      SPDLOG_INFO("X = {}, Y = {}, Z = {}", X, Y, Z);
      map.AddMapPoint(MapPoint(i, 0, i, Eigen::Vector3f(X, Y, Z)));
    }
  }
  map.AddKeyFrame(frame);
  map.initialized_ = true;
  SPDLOG_INFO("Map = {}", map);
  return true;
}

}  // namespace slam