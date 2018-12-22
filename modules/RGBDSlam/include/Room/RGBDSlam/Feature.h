// Copyright 2018 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FEATURE_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FEATURE_H_

#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

namespace room
{
class Feature
{
 public:
  explicit Feature(const std::string show_debug);
  ~Feature() = default;

  void FeatureDetectionAndMatchORB(const cv::Mat& img1,
                                   const cv::Mat& img2,
                                   int nfeatures,
                                   float scale_factor,
                                   int nlevels,
                                   int edge_threshold,
                                   int first_level,
                                   int WTA_K,
                                   cv::ORB::ScoreType score_type,
                                   int patch_size,
                                   int fast_threshold,
                                   bool with_rotation,
                                   bool with_scale,
                                   float threshold_factor,
                                   std::vector<cv::Point2f>& pts1,
                                   std::vector<cv::Point2f>& pts2);

 private:
  std::string m_show_debug;
};  // class feature
}  // namespace room

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FEATURE_H_
