// Copyright 2018 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_ODOMETRY_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_ODOMETRY_H_

#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

namespace room
{
class Odometry
{
 public:
  Odometry() = delete;
  explicit Odometry(const std::string show_debug);
  ~Odometry() = default;

  void RtEstimationEssential(const std::vector<cv::Point2f>& pts1,
                             const std::vector<cv::Point2f>& pts2,
                             const float focal,
                             const cv::Point2d pp,
                             cv::Mat& R,
                             cv::Mat& t);

  /**
   * @brief      Deconstructed variant of ICP implementing for checking results
   *             and because of KITTI results
   *
   * @param[in]  pts0    The points 0
   * @param[in]  pts1    The points 1
   * @param[in]  depth0  The depth 0
   * @param[in]  depth1  The depth 1
   * @param[in]  focal   The focal
   * @param[in]  pp      { parameter_description }
   * @param      R       { parameter_description }
   * @param      t       { parameter_description }
   */
  void RtEstimationICP(const std::vector<cv::Point2f>& pts0,
                       const std::vector<cv::Point2f>& pts1,
                       const cv::Mat& color0,
                       const cv::Mat& color1,
                       const cv::Mat& depth0,
                       const cv::Mat& depth1,
                       const float focal,
                       const cv::Point2d pp,
                       cv::Mat& R,
                       cv::Mat& t);

 private:
  std::string m_show_debug;
};  // class odometry
}  // namespace room

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_ODOMETRY_H_
