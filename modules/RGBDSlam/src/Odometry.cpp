// Copyright 2018 Arunabh Sharma

#include "Room/RGBDSlam/Odometry.h"

#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

namespace room
{
Odometry::Odometry(const std::string show_debug) : m_show_debug(show_debug)
{
}

void Odometry::RtEstimationEssential(const std::vector<cv::Point2f> pts1,
                                     const std::vector<cv::Point2f> pts2,
                                     const float focal,
                                     const cv::Point2d pp,
                                     cv::Mat &R,
                                     cv::Mat &t)
{
  cv::Mat E, mask, R1;
  E = cv::findEssentialMat(pts1, pts2, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
  cv::recoverPose(E, pts1, pts2, R, t, focal, pp, mask);  // R is from 1 to 2
}
}  // namespace room
