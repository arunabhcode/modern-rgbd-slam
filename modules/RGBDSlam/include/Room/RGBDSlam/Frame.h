// Copyright 2019 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FRAME_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FRAME_H_

#include <vector>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "Room/RGBDSlam/Pose.h"

namespace room
{
class Frame
{
   public:
    Frame()  = delete;
    ~Frame() = default;
    explicit Frame(const int id);
    Frame(const int id,
          const Pose frame_pose,
          const uint64_t timestamp,
          const cv::Mat& color,
          const cv::Mat& depth,
          const float focal,
          const Eigen::Vector2f& pp);

    cv::Mat ToIntrinsicsMat();

    void SetKeypoints(const std::vector<cv::KeyPoint>& in_keypoints);
    void SetDescriptors(const cv::Mat& in_descriptors);

    std::vector<cv::KeyPoint>& GetKeypoints();
    cv::Mat& GetDescriptors();

   private:
    int m_id;
    Pose m_pose;
    uint64_t m_timestamp;
    cv::Mat m_color_img;
    cv::Mat m_depth_img;
    float m_focal;
    Eigen::Vector2f m_pp;
    std::vector<cv::KeyPoint> m_keypoints;
    cv::Mat m_descriptors;
};  // class frame

}  // namespace room

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_FRAME_H_
