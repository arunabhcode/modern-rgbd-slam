// Copyright 2018 Arunabh Sharma

#include "Room/RGBDSlam/Odometry.h"

#include <memory>
#include <string>
#include <vector>

// #include "ceres/ceres.h"
#include "Eigen/Core"
#include "ceres/ceres.h"
#include "opencv2/opencv.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

#include "Room/RGBDSlam/ReprojICP.h"

namespace room
{
Odometry::Odometry(const std::string show_debug) : m_show_debug(show_debug)
{
}

void Odometry::RtEstimationEssential(const std::vector<cv::Point2f>& pts1,
                                     const std::vector<cv::Point2f>& pts2,
                                     const float focal,
                                     const cv::Point2d pp,
                                     cv::Mat& R,
                                     cv::Mat& t)
{
  cv::Mat E, mask, R1;
  E = cv::findEssentialMat(pts1, pts2, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
  cv::recoverPose(E, pts1, pts2, R, t, focal, pp, mask);  // R is from 1 to 2
}

void Odometry::RtEstimationICP(const std::vector<cv::Point2f>& pts0,
                               const std::vector<cv::Point2f>& pts1,
                               const cv::Mat& color0,
                               const cv::Mat& color1,
                               const cv::Mat& depth0,
                               const cv::Mat& depth1,
                               const float focal,
                               const cv::Point2d pp,
                               cv::Mat& R,
                               cv::Mat& t)
{
  if (pts0.size() != pts1.size())
  {
    spdlog::warn("The size of correspondence vectors does not match");
  }
  else
  {
    spdlog::info("Size of correspondence vectors = {}", pts0.size());
  }
  std::unique_ptr<double[]> extrinsics(new double[6]);
  cv::Mat R_init = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat r_init;
  cv::Rodrigues(R_init, r_init);

  cv::Mat R_decode(3, 3, CV_64FC1, cv::Scalar(0.0));
  cv::Mat r_decode(3, 1, CV_64FC1, cv::Scalar(0.0));

  cv::Mat t_init(1, 3, CV_64FC1, cv::Scalar(0.0));
  cv::Mat t_decode(1, 3, CV_64FC1, cv::Scalar(0.0));

  extrinsics[0] = r_init.at<double>(0, 0);
  extrinsics[1] = r_init.at<double>(1, 0);
  extrinsics[2] = r_init.at<double>(2, 0);
  extrinsics[3] = t_init.at<double>(0, 0);
  extrinsics[4] = t_init.at<double>(0, 1);
  extrinsics[5] = t_init.at<double>(0, 2);

  ceres::Problem problem;
  std::vector<ceres::ResidualBlockId> all_res_blk_ids;

  for (std::size_t i = 0; i < pts0.size(); i++)
  {
    float pt0_Z = depth0.at<float>(static_cast<int>(pts0[i].y),
                                   static_cast<int>(pts0[i].x));
    Eigen::Vector3f pt0_3d((pts0[i].x - pp.x) / focal * pt0_Z,
                           (pts0[i].y - pp.y) / focal * pt0_Z,
                           pt0_Z);

    float pt1_Z = depth1.at<float>(static_cast<int>(pts1[i].y),
                                   static_cast<int>(pts1[i].x));

    Eigen::Vector3f pt1_3d((pts1[i].x - pp.x) / focal * pt1_Z,
                           (pts1[i].y - pp.y) / focal * pt1_Z,
                           pt1_Z);

    ReprojICP* reprojICP = new ReprojICP(
        pt0_3d[0], pt0_3d[1], pt0_3d[2], pt1_3d[0], pt1_3d[1], pt1_3d[2]);

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ReprojICP, 3, 6>(reprojICP);
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

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

  r_decode.at<double>(0, 0) = extrinsics[0];
  r_decode.at<double>(1, 0) = extrinsics[1];
  r_decode.at<double>(2, 0) = extrinsics[2];
  cv::Rodrigues(r_decode, R_decode);

  t_decode.at<double>(0, 0) = extrinsics[3];
  t_decode.at<double>(0, 1) = extrinsics[4];
  t_decode.at<double>(0, 2) = extrinsics[5];

  R = R_decode.clone();
  t = t_decode.clone();
}
}  // namespace room
