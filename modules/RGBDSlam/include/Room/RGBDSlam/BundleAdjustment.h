// Copyright 2019 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_BUNDLEADJUSTMENT_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_BUNDLEADJUSTMENT_H_

#include <memory>

#include "g2o/core/sparse_optimizer.h"

#include "Room/Components/Frame.h"
#include "Room/Components/MapPoint.h"

namespace room
{
class BundleAdjustment
{
   public:
    typedef g2o::OptimizableGraph::Vertex Vertex;
    typedef g2o::OptimizableGraph::Edge Edge;

    BundleAdjustment() = delete;
    BundleAdjustment(const int num_iter,
                     const bool verbose,
                     const bool force_stop_flag,
                     const float terminate_threshold,
                     const bool robust_loss_function);
    ~BundleAdjustment() = default;

    bool AddVertex(const Frame& frame);
    bool AddVertex(const MapPoint& mappoint);
    bool AddEdge(const int kp_idx, const Frame& frame, Vertex* v_point, Vertex* v_frame);
    void BuildGraph(std::vector<std::shared_ptr<MapPoint>> mps,
                    std::vector<std::shared_ptr<Frame>> frames);
    void Run();

    void SetForceStopFlag(const bool& force_stop_flag);
    void SetVerbose(const bool& verbose);

   private:
    int m_vertex_id;
    int m_num_iter;
    bool m_verbose;
    bool m_force_stop_flag;
    float m_terminate_threshold;
    bool m_robust_loss_function;

    g2o::SparseOptimizer m_optimizer;

};  // class bundleadjustment
}  // namespace room

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_BUNDLEADJUSTMENT_H_
