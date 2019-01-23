// Copyright 2018 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FEATURE_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FEATURE_H_

#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

#include "Room/RGBDSlam/Frame.h"

namespace room
{
class Feature
{
   public:
    Feature() = delete;
    explicit Feature(const std::string show_debug);
    ~Feature() = default;

    void FeatureDetectionAndMatchORB(Frame& frame0,
                                     Frame& frame1,
                                     const int nfeatures,
                                     const float scale_factor,
                                     const int nlevels,
                                     const int edge_threshold,
                                     const int first_level,
                                     const int WTA_K,
                                     const cv::ORB::ScoreType score_type,
                                     const int patch_size,
                                     const int fast_threshold,
                                     const bool with_rotation,
                                     const bool with_scale,
                                     const float threshold_factor,
                                     const int num_points);

    std::vector<cv::KeyPoint> ANMS(std::vector<cv::KeyPoint> keypoints,
                                   int num_points);

   private:
    std::string m_show_debug;
};  // class feature
}  // namespace room

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FEATURE_H_
