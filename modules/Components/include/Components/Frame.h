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
  {
  }
  Frame(const int id,
        const uint64_t timestamp,
        const cv::Mat& color,
        const cv::Mat& depth)
      : frame_id_(id)
      , timestamp_(timestamp)
      , color_img_(color)
      , depth_img_(depth)
  {
  }

  bool operator==(const Frame& rhs) const
  {
    return frame_id_ == rhs.frame_id_;
  }

  int frame_id_;
  Pose pose_;
  uint64_t timestamp_;
  cv::Mat color_img_;
  cv::Mat depth_img_;
  cv::Mat undist_color_img_;
  cv::Mat undist_depth_img_;
  std::vector<cv::Point2f> keypoints_;
  std::vector<cv::KeyPoint> features_;
  std::vector<int> mappoint_idxs;
  cv::Mat descriptors_;
  bool features_found_;
};  // struct frame

}  // namespace slam

#endif  // MODULES_COMPONENTS_INCLUDE_COMPONENTS_FRAME_H_
