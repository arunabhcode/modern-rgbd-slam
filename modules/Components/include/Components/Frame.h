// Copyright 2019 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FRAME_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FRAME_H_

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
      : m_frame_id(id)
      , m_pose(Pose())
      , m_timestamp(0)
      , m_color_img(cv::Mat())
      , m_depth_img(cv::Mat())
      , m_focal(0.0f)
      , m_pp(Eigen::Vector2f(0.0f, 0.0f))
      , m_features_found(false)
  {
  }
  Frame(const int id,
        const uint64_t timestamp,
        const cv::Mat& color,
        const cv::Mat& depth,
        const float focal,
        const Eigen::Vector2f& pp)
      : m_frame_id(id)
      , m_timestamp(timestamp)
      , m_color_img(color)
      , m_depth_img(depth)
      , m_focal(focal)
      , m_pp(pp)
      , m_features_found(false)

  {
  }

  cv::Mat ToIntrinsicsMat()
  {
    cv::Mat intrinsics         = cv::Mat::eye(3, 3, CV_32FC1);
    intrinsics.at<float>(0, 0) = m_focal;
    intrinsics.at<float>(1, 1) = m_focal;
    intrinsics.at<float>(0, 2) = m_pp[0];
    intrinsics.at<float>(1, 2) = m_pp[1];
    return intrinsics;
  }

  int m_frame_id;
  Pose m_pose;
  uint64_t m_timestamp;
  cv::Mat m_color_img;
  cv::Mat m_depth_img;
  float m_focal;
  Eigen::Vector2f m_pp;
  std::vector<cv::Point2f> m_keypoints;
  std::vector<cv::KeyPoint> m_features;
  std::unordered_map<int, int>
      m_key_v_map_points;  ///< key is kp_idx and value is global
                           ///< mappoint id assigned localmapping
  cv::Mat m_descriptors;
  std::vector<float> m_depth;
  bool m_features_found;
};  // struct frame

}  // namespace slam

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FRAME_H_
