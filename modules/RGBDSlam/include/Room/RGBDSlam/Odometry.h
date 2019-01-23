// Copyright 2018 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_ODOMETRY_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_ODOMETRY_H_

#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

#include "Room/RGBDSlam/Frame.h"
#include "Room/RGBDSlam/Pose.h"

namespace room
{
class Odometry
{
   public:
    Odometry() = delete;
    explicit Odometry(const std::string show_debug);
    ~Odometry() = default;

    // void RtEstimationEssential(const std::vector<cv::Point2f>& pts1,
    //                            const std::vector<cv::Point2f>& pts2,
    //                            const float focal,
    //                            const cv::Point2d pp,
    //                            cv::Mat& R,
    //                            cv::Mat& t);

    Pose PoseEstimationICP(Frame& frame0,
                           Frame& frame1,
                           Pose initial_pose = Pose());

   private:
    std::string m_show_debug;
};  // class odometry
}  // namespace room

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_ODOMETRY_H_
