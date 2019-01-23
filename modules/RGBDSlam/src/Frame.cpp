// Copyright 2019 Arunabh Sharma

#include "Room/RGBDSlam/Frame.h"

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "Room/RGBDSlam/Pose.h"

namespace room
{
Frame::Frame(const int id)
    : m_id(id)
    , m_pose(Pose())
    , m_timestamp(0)
    , m_color_img(cv::Mat())
    , m_depth_img(cv::Mat())
    , m_focal(0.0f)
    , m_pp(Eigen::Vector2f(0.0f, 0.0f))
{
}

Frame::Frame(const int id,
             const uint64_t timestamp,
             const cv::Mat& color,
             const cv::Mat& depth,
             const float focal,
             const Eigen::Vector2f& pp)
    : m_id(id)
    , m_timestamp(timestamp)
    , m_color_img(color)
    , m_depth_img(depth)
    , m_focal(focal)
    , m_pp(pp)
{
}

cv::Mat Frame::ToIntrinsicsMat()
{
    cv::Mat intrinsics         = cv::Mat::eye(3, 3, CV_32FC1);
    intrinsics.at<float>(0, 0) = m_focal;
    intrinsics.at<float>(1, 1) = m_focal;
    intrinsics.at<float>(0, 2) = m_pp[0];
    intrinsics.at<float>(1, 2) = m_pp[1];
    return intrinsics;
}

void Frame::SetKeypoints(const std::vector<cv::Point2f>& in_keypoints)
{
    m_keypoints = in_keypoints;
}

void Frame::SetDescriptors(const cv::Mat& in_descriptors)
{
    m_descriptors = in_descriptors;
}

std::vector<cv::Point2f>& Frame::GetKeypoints()
{
    return m_keypoints;
}

cv::Mat& Frame::GetDescriptors()
{
    return m_descriptors;
}

int Frame::GetKeypointsSize()
{
    return m_keypoints.size();
}

Eigen::Vector2f Frame::GetPrincipalPoint()
{
    return m_pp;
}

float Frame::GetFocal()
{
    return m_focal;
}

cv::Mat& Frame::GetDepth()
{
    return m_depth_img;
}

cv::Mat& Frame::GetColor()
{
    return m_color_img;
}

void Frame::SetPose(const Pose& in_pose)
{
    m_pose = in_pose;
}

}  // namespace room
