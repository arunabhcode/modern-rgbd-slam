// Copyright 2019 Arunabh Sharma

#ifndef MODULES_COMPONENTS_INCLUDE_COMPONENTS_FRAME_H_
#define MODULES_COMPONENTS_INCLUDE_COMPONENTS_FRAME_H_

#include <unordered_map>
#include <vector>

#include "Components/Pose.h"
#include "Eigen/Core"
#include "opencv2/opencv.hpp"

namespace slam
{
struct Frame
{
  Frame()  = delete;
  ~Frame() = default;

  explicit Frame(const int id)
      : frame_id_(id)
      , pose_(Pose())
      , timestamp_(0)
      , color_img_(cv::Mat())
      , depth_img_(cv::Mat())
      , focal_(0.0f)
      , pp_(Eigen::Vector2f(0.0f, 0.0f))
      , features_found_(false)
  {
  }
  Frame(const int id,
        const uint64_t timestamp,
        const cv::Mat& color,
        const cv::Mat& depth,
        const float focal,
        const Eigen::Vector2f& pp)
      : frame_id_(id)
      , timestamp_(timestamp)
      , color_img_(color)
      , depth_img_(depth)
      , focal_(focal)
      , pp_(pp)
      , features_found_(false)

  {
  }

  cv::Mat ToIntrinsicsMat()
  {
    cv::Mat intrinsics         = cv::Mat::eye(3, 3, CV_32FC1);
    intrinsics.at<float>(0, 0) = focal_;
    intrinsics.at<float>(1, 1) = focal_;
    intrinsics.at<float>(0, 2) = pp_[0];
    intrinsics.at<float>(1, 2) = pp_[1];
    return intrinsics;
  }

  int frame_id_;
  Pose pose_;
  uint64_t timestamp_;
  cv::Mat color_img_;
  cv::Mat depth_img_;
  float focal_;
  Eigen::Vector2f pp_;
  std::vector<cv::Point2f> keypoints_;
  std::vector<cv::KeyPoint> features_;
  std::unordered_map<int, int>
      key_v_map_points_;  ///< key is kp_idx and value is global
                          ///< mappoint id assigned localmapping
  cv::Mat descriptors_;
  std::vector<float> depth_;
  bool features_found_;
};  // struct frame

}  // namespace slam

#endif  // MODULES_COMPONENTS_INCLUDE_COMPONENTS_FRAME_H_
