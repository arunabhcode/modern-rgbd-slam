// Copyright 2018 Arunabh Sharma

#include "RGBDSlam/Odometry.h"

#include <memory>
#include <string>
#include <vector>

#include "Components/Frame.h"
#include "Components/Pose.h"
#include "Eigen/Core"
#include "ceres/ceres.h"
#include "opencv2/opencv.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

//
#include "RGBDSlam/ICP.h"

namespace room
{
Odometry::Odometry(const std::string show_debug) : m_show_debug(show_debug)
{
}

Pose Odometry::PoseEstimationICP(Frame& frame0,
                                 Frame& frame1,
                                 std::vector<int>& matches,
                                 Pose initial_pose)
{
  // if (frame0.m_keypoints.size() != frame1.m_keypoints.size())
  // {
  //   spdlog::warn("The size of correspondence vectors does not match");
  // }
  // else
  // {
  //   spdlog::info("Size of correspondence vectors = {}",
  //                frame0.m_keypoints.size());
  // }
  std::unique_ptr<double[]> extrinsics(
      new double[7]);  // 4 for the orientation, 3 for the translation

  spdlog::info("Initial rotation = \n {}",
               initial_pose.m_orientation.toRotationMatrix());
  spdlog::info("Initial translation = \n {}", initial_pose.m_position);

  // convention for the quaternion in this repo will be w, x, y, z
  extrinsics[0] = initial_pose.m_orientation.w();
  extrinsics[1] = initial_pose.m_orientation.x();
  extrinsics[2] = initial_pose.m_orientation.y();
  extrinsics[3] = initial_pose.m_orientation.z();
  extrinsics[4] = initial_pose.m_position[0];
  extrinsics[5] = initial_pose.m_position[1];
  extrinsics[6] = initial_pose.m_position[2];

  ceres::Problem problem;
  std::vector<ceres::ResidualBlockId> all_res_blk_ids;

  std::vector<cv::Point2f>& pts0 = frame0.m_keypoints;
  std::vector<cv::Point2f>& pts1 = frame1.m_keypoints;
  SPDLOG_INFO("Size of pts0 = {}", pts0.size());
  SPDLOG_INFO("Size of pts1 = {}", pts1.size());

  for (const int i : matches)
  {
    SPDLOG_INFO("Idx = {}", i);
    SPDLOG_INFO("Pixel location = {},{}", pts0[i].y, pts0[i].x);
    float pt0_Z = frame0.m_depth_img.at<float>(static_cast<int>(pts0[i].y),
                                               static_cast<int>(pts0[i].x));
    SPDLOG_INFO("");
    Eigen::Vector3f pt0_3d(
        (pts0[i].x - frame0.m_pp[0]) / frame0.m_focal * pt0_Z,
        (pts0[i].y - frame0.m_pp[1]) / frame0.m_focal * pt0_Z,
        pt0_Z);
    SPDLOG_INFO("");

    float pt1_Z = frame1.m_depth_img.at<float>(static_cast<int>(pts1[i].y),
                                               static_cast<int>(pts1[i].x));
    SPDLOG_INFO("");

    Eigen::Vector3f pt1_3d(
        (pts1[i].x - frame1.m_pp[0]) / frame1.m_focal * pt1_Z,
        (pts1[i].y - frame1.m_pp[1]) / frame1.m_focal * pt1_Z,
        pt1_Z);
    SPDLOG_INFO("");

    ICP* icp = new ICP(
        pt0_3d[0], pt0_3d[1], pt0_3d[2], pt1_3d[0], pt1_3d[1], pt1_3d[2]);
    SPDLOG_INFO("");

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ICP, 3, 7>(icp);
    ceres::LossFunction* loss_function = new ceres::HuberLoss(5.591);
    SPDLOG_INFO("");

    all_res_blk_ids.emplace_back(problem.AddResidualBlock(
        cost_function, loss_function, extrinsics.get()));
  }

  spdlog::info("Ceres to start solving");
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  spdlog::info("Ceres summary = {}", summary.BriefReport());
  spdlog::info("Ceres finished solving");

  Pose out_pose;
  out_pose.m_orientation.w() = extrinsics[0];
  out_pose.m_orientation.x() = extrinsics[1];
  out_pose.m_orientation.y() = extrinsics[2];
  out_pose.m_orientation.z() = extrinsics[3];
  out_pose.m_position[0]     = extrinsics[4];
  out_pose.m_position[1]     = extrinsics[5];
  out_pose.m_position[2]     = extrinsics[6];

  spdlog::info("Calculated pose = {}", out_pose);

  return out_pose;
}
}  // namespace room
