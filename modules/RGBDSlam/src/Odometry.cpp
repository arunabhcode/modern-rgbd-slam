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

#include "Room/RGBDSlam/ICP.h"
#include "Room/RGBDSlam/Pose.h"

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
    E = cv::findEssentialMat(
        pts1, pts2, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, pts1, pts2, R, t, focal, pp, mask);  // R is from 1 to 2
}

Pose Odometry::PoseEstimationICP(const std::vector<cv::Point2f>& pts0,
                                 const std::vector<cv::Point2f>& pts1,
                                 const cv::Mat& color0,
                                 const cv::Mat& color1,
                                 const cv::Mat& depth0,
                                 const cv::Mat& depth1,
                                 const float focal,
                                 const cv::Point2d pp,
                                 Pose initial_pose)
{
    if (pts0.size() != pts1.size())
    {
        spdlog::warn("The size of correspondence vectors does not match");
    }
    else
    {
        spdlog::info("Size of correspondence vectors = {}", pts0.size());
    }
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

        ICP* icp = new ICP(
            pt0_3d[0], pt0_3d[1], pt0_3d[2], pt1_3d[0], pt1_3d[1], pt1_3d[2]);

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<ICP, 3, 7>(icp);
        ceres::LossFunction* loss_function = new ceres::HuberLoss(5.591);

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

    return out_pose;
}
}  // namespace room
