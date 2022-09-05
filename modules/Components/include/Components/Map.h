// Copyright 2019 Arunabh Sharma

#ifndef MODULES_COMPONENTS_INCLUDE_COMPONENTS_MAP_H_
#define MODULES_COMPONENTS_INCLUDE_COMPONENTS_MAP_H_

#include <algorithm>
#include <unordered_map>

#include "Components/Frame.h"
#include "Components/MapPoint.h"
#include "Eigen/Core"

namespace slam
{
struct Map
{
  Map()  = default;
  ~Map() = default;

  void AddKeyFrame(const Frame& frame)
  {
    keyframes_.emplace_back(frame);
  }

  void RemoveKeyFrame(const Frame& frame)
  {
    keyframes_.erase(std::remove(keyframes_.begin(), keyframes_.end(), frame),
                     keyframes_.end());
  }

  void AddMapPoint(const MapPoint& point)
  {
    points_.emplace_back(point);
  }

  void RemoveMapPoint(const MapPoint& point)
  {
    points_.erase(std::remove(points_.begin(), points_.end(), point),
                  points_.end());
  }

  bool initialized;
  std::vector<MapPoint> points_;
  std::vector<Frame> keyframes_;
};  // struct map

}  // namespace slam

#endif  // MODULES_COMPONENTS_INCLUDE_COMPONENTS_MAP_H_
