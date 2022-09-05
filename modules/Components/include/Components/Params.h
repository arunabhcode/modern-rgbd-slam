// Copyright 2022 Arunabh Sharma

#ifndef MODULES_COMPONENTS_INCLUDE_COMPONENTS_PARAMS_H_
#define MODULES_COMPONENTS_INCLUDE_COMPONENTS_PARAMS_H_

#include "opencv2/opencv.hpp"

struct SlamParams
{
  /// Camera intrinsic params
  float focal_;
  cv::Point2f pp_;
  cv::Mat dist_;

  /// ORB feature detector params
  std::string show_debug_;
  int nfeatures_;
  float scale_factor_;
  int nlevels_;
  int edge_threshold_;
  int first_level_;
  int WTA_K_;
  cv::ORB::ScoreType score_type_;
  int patch_size_;
  int fast_threshold_;
  int num_points_;
};

#endif  // MODULES_COMPONENTS_INCLUDE_COMPONENTS_PARAMS_H_