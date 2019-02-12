// Copyright 2019 Arunabh Sharma

#include "Room/RGBDSlam/BundleAdjustment.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/icp/types_icp.h"
#include "g2o/types/sba/types_sba.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

namespace room
{
BundleAdjustment::BundleAdjustment(const int num_iter,
                                   const bool verbose,
                                   const bool force_stop_flag,
                                   const float terminate_threshold,
                                   const bool robust_loss_function)
    : m_vertex_id(0)
    , m_num_iter(num_iter)
    , m_verbose(verbose)
    , m_force_stop_flag(force_stop_flag)
    , m_terminate_threshold(terminate_threshold)
    , m_robust_loss_function(robust_loss_function)
{
    m_optimizer.setVerbose(&m_verbose);
    m_optimizer.setForceStopFlag(&m_force_stop_flag);

    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver(
        new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());
    std::unique_ptr<g2o::BlockSolver_6_3> solver_uptr(
        new g2o::BlockSolver_6_3(std::move(linear_solver)));
    std::unique_ptr<g2o::OptimizationAlgorithmLevenberg> solver =
        std::make_unique<g2o::OptimizationAlgorithmLevenberg>(std::move(solver_uptr));
    m_optimizer.setAlgorithm(solver.get());

    std::unique_ptr<g2o::SparseOptimizerTerminateAction> terminate_action(
        new g2o::SparseOptimizerTerminateAction());
    terminate_action->setGainThreshold(m_terminate_threshold);
    m_optimizer.addPostIterationAction(terminate_action.get());
}

bool BundleAdjustment::AddVertex(const Frame& frame)
{
    g2o::SBACam sbacam(frame.m_pose.m_orientation.cast<double>(),
                       frame.m_pose.m_position.cast<double>());
    g2o::VertexCam* v_se3 = new g2o::VertexCam();

    v_se3->setId(m_vertex_id++);
    v_se3->setEstimate(sbacam);
    v_se3->setFixed(frame.m_frame_id == 0);

    m_optimizer.addVertex(v_se3);
    return true;
}

bool BundleAdjustment::AddVertex(const MapPoint& mappoint)
{
    g2o::VertexSBAPointXYZ* landmark = new g2o::VertexSBAPointXYZ();

    landmark->setId(m_vertex_id++);
    landmark->setMarginalized(true);
    landmark->setEstimate(mappoint.m_world_position.cast<double>());

    m_optimizer.addVertex(landmark);
    return true;
}

bool BundleAdjustment::AddEdge(
    const int edge_id, const int kp_idx, const Frame& frame, Vertex* v_point, Vertex* v_frame)
{
    g2o::EdgeSE3ProjectXYZ* edge_se3 = new g2o::EdgeSE3ProjectXYZ();
    edge_se3->setId(edge_id);
    edge_se3->vertices()[0] = v_point;
    edge_se3->vertices()[1] = v_frame;
    edge_se3->setMeasurement(
        Eigen::Vector2d(frame.m_keypoints[kp_idx].x, frame.m_keypoints[kp_idx].y));
    edge_se3->setInformation(Eigen::Matrix2d::Identity());

    if (m_robust_loss_function)
    {
        g2o::RobustKernelHuber* loss_function = new g2o::RobustKernelHuber;
        loss_function->setDelta(std::sqrt(5.991));
        edge_se3->setRobustKernel(loss_function);
    }

    edge_se3->fx = frame.m_focal;
    edge_se3->fy = frame.m_focal;
    edge_se3->cx = frame.m_pp[0];
    edge_se3->cy = frame.m_pp[1];

    m_optimizer.addEdge(edge_se3);
    return true;
}

void BundleAdjustment::BuildGraph(std::vector<std::shared_ptr<MapPoint>> mps,
                                  std::vector<std::shared_ptr<Frame>> frames)
{
    for (std::size_t i = 0; i < frames.size(); i++)
    {
        AddVertex(*frames[i]);
    }
    for (std::size_t j = 0; j < mps.size(); j++)
    {
        AddVertex(*mps[j]);
    }
}

void BundleAdjustment::Run()
{
    m_optimizer.initializeOptimization();
    m_optimizer.optimize(m_num_iter);
}

void BundleAdjustment::SetForceStopFlag(const bool& force_stop_flag)
{
    m_force_stop_flag = force_stop_flag;
}

void BundleAdjustment::SetVerbose(const bool& verbose)
{
    m_verbose = verbose;
}

}  // namespace room
