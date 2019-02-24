// Copyright 2018 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FEATURE_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FEATURE_H_

#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

#include "Room/Components/Frame.h"

namespace room
{
class Feature
{
   public:
    Feature() = delete;
    Feature(const std::string show_debug,
            const int nfeatures,
            const float scale_factor,
            const int nlevels,
            const int edge_threshold,
            const int first_level,
            const int WTA_K,
            const cv::ORB::ScoreType score_type,
            const int patch_size,
            const int fast_threshold,
            const int num_points);
    ~Feature() = default;

    void FindFeatures(Frame& frame);

    void FeatureMatch(Frame& frame0,
                      Frame& frame1,
                      const bool with_rotation,
                      const bool with_scale,
                      const float threshold_factor);

    void FeatureTrack(Frame& frame0, Frame& frame1, const cv::Size win_size, const int max_level);

    std::vector<cv::KeyPoint> ANMS(std::vector<cv::KeyPoint> keypoints, int num_points);

   private:
    std::string m_show_debug;
    cv::Ptr<cv::Feature2D> m_feature_md_inst;
    int m_num_points;
};  // class feature
}  // namespace room

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FEATURE_H_
