// Copyright 2019 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_MAPPOINT_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_MAPPOINT_H_

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
           const Eigen::Vector3f& world_position)
      : m_mappoint_id(mappoint_id)
      , m_reference_frame_id(frame_id)
      , m_world_position(world_position)
      , m_visible_count(1)
  {
    m_observations[frame_id] = keypoint_idx;
  }
  void AddObservation(const int& frame_id, const int& keypoint_idx)
  {
    m_observations[frame_id] = keypoint_idx;
    m_visible_count++;
  }

  int m_mappoint_id;
  std::unordered_map<int, int> m_observations;  ///< frame id and keypoint idx
  int m_reference_frame_id;
  Eigen::Vector3f m_world_position;
  int m_visible_count;
};  // struct mappoint

}  // namespace slam

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_MAPPOINT_H_
