// Copyright 2018 Arunabh Sharma

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

#include "Room/RGBDSlam/Feature.h"

int main()
{
  cv::Mat img1 = cv::imread("/home/user/realsense-rgbd-slam/img1_Color.png");
  cv::Mat img2 = cv::imread("/home/user/realsense-rgbd-slam/img2_Color.png");

  int nfeatures                 = 10000;
  float scale_factor            = 1.2f;
  int nlevels                   = 8;
  int edge_threshold            = 31;
  int first_level               = 0;
  int WTA_K                     = 2;
  cv::ORB::ScoreType score_type = cv::ORB::HARRIS_SCORE;
  int patch_size                = 31;
  int fast_threshold            = 0;
  bool with_rotation            = false;
  bool with_scale               = false;
  float threshold_factor        = 10.0f;

  std::string show_debug = "ALL";

  std::vector<cv::Point2f> pts1;
  std::vector<cv::Point2f> pts2;

  room::Feature feature_inst(show_debug);
  feature_inst.FeatureDetectionAndMatchORB(img1,
                                           img2,
                                           nfeatures,
                                           scale_factor,
                                           nlevels,
                                           edge_threshold,
                                           first_level,
                                           WTA_K,
                                           score_type,
                                           patch_size,
                                           fast_threshold,
                                           with_rotation,
                                           with_scale,
                                           threshold_factor,
                                           pts1,
                                           pts2);

  return 0;
}
