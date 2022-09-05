// Copyright 2018 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_RGBDSLAM_FEATURE_H_
#define MODULES_RGBDSLAM_INCLUDE_RGBDSLAM_FEATURE_H_

#include <string>
#include <vector>

#include "Components/Frame.h"
#include "Components/Params.h"
#include "opencv2/opencv.hpp"

namespace slam
{
class Feature
{
 public:
  Feature()  = default;
  ~Feature() = default;

  void Configure(const SlamParams& params);

  bool FindFeatures(Frame& frame);

  void FeatureMatch(Frame& frame0,
                    Frame& frame1,
                    const bool with_rotation,
                    const bool with_scale,
                    const float threshold_factor);

  void FeatureTrack(Frame& frame0,
                    Frame& frame1,
                    const cv::Size win_size,
                    const int max_level,
                    std::vector<int>& kps_idx_track);

  std::vector<cv::KeyPoint> ANMS(std::vector<cv::KeyPoint> keypoints,
                                 int num_points);

 private:
  std::string show_debug_;
  SlamParams params_inst_;
  cv::Ptr<cv::Feature2D> feature_md_inst_;
  int num_points_;
};  // class feature
}  // namespace slam

#endif  // MODULES_RGBDSLAM_INCLUDE_RGBDSLAM_FEATURE_H_
