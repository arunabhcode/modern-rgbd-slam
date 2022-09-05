// Copyright 2019 Arunabh Sharma

#ifndef MODULES_COMPONENTS_INCLUDE_COMPONENTS_MAPPOINT_H_
#define MODULES_COMPONENTS_INCLUDE_COMPONENTS_MAPPOINT_H_

#include <unordered_map>

#include "Eigen/Core"

namespace slam
{
struct MapPoint
{
  MapPoint()  = delete;
  ~MapPoint() = default;

  MapPoint(const int& mappoint_id,
           const int& frame_id,
           const int& keypoint_idx,
           const Eigen::Vector3f& position)
      : mappoint_id_(mappoint_id)
      , reference_frame_id_(frame_id)
      , position_(position)
      , visible_count_(1)
  {
    observations_[frame_id] = keypoint_idx;
  }
  void AddObservation(const int& frame_id, const int& keypoint_idx)
  {
    observations_[frame_id] = keypoint_idx;
    visible_count_++;
  }

  bool operator==(const MapPoint& rhs) const
  {
    return mappoint_id_ == rhs.mappoint_id_;
  }

  int mappoint_id_;
  std::unordered_map<int, int> observations_;  ///< frame id and keypoint idx
  int reference_frame_id_;
  Eigen::Vector3f position_;
  int visible_count_;
};  // struct mappoint

}  // namespace slam

#endif  // MODULES_COMPONENTS_INCLUDE_COMPONENTS_MAPPOINT_H_
