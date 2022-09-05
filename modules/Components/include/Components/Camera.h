// Copyright 2022 Arunabh Sharma

#ifndef MODULES_COMPONENTS_INCLUDE_COMPONENTS_CAMERA_H_
#define MODULES_COMPONENTS_INCLUDE_COMPONENTS_CAMERA_H_

#include "opencv2/opencv.hpp"

//
#include "Components/Params.h"

namespace slam
{
/**
 * @brief      Structure that holds parameters related to a pinhole camera
 */
struct Camera
{
  float focal_;     ///< Focal length
  cv::Point2f pp_;  ///< Principle point
  cv::Mat dist_;    ///< Distortion coefficients

  void Configure(const SlamParams& params)
  {
    focal_ = params.focal_;
    pp_    = params.pp_;
    dist_  = params.dist_;
  }

  cv::Mat ToIntrinsicsMat()
  {
    cv::Mat intrinsics         = cv::Mat::eye(3, 3, CV_32FC1);
    intrinsics.at<float>(0, 0) = focal_;
    intrinsics.at<float>(1, 1) = focal_;
    intrinsics.at<float>(0, 2) = pp_.x;
    intrinsics.at<float>(1, 2) = pp_.y;
    return intrinsics;
  }
};  // struct pinhole_camera
}  // namespace slam
#endif  // MODULES_COMPONENTS_INCLUDE_COMPONENTS_CAMERA_H_