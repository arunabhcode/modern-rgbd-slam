// Copyright 2018 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_REPROJICP_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_REPROJICP_H_

#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace room
{
class ReprojICP
{
 public:
  ReprojICP(const double x1,
            const double y1,
            const double z1,
            const double x2,
            const double y2,
            const double z2);

  ~ReprojICP() = default;

  template <typename T>
  bool operator()(const T* extrinsics, T* residuals) const
  {
    T rot13[3];
    T t13[3];
    // T rot33[9];

    rot13[0] = extrinsics[0];
    rot13[1] = extrinsics[1];
    rot13[2] = extrinsics[2];

    t13[0] = extrinsics[3];
    t13[1] = extrinsics[4];
    t13[2] = extrinsics[5];

    // ceres::AngleAxisToRotationMatrix(rot13, rot33);
    T rot_pt[3] = {T(m_x2), T(m_y2), T(m_z2)};

    ceres::AngleAxisRotatePoint(rot13, rot_pt, rot_pt);
    rot_pt[0] += t13[0];
    rot_pt[1] += t13[1];
    rot_pt[2] += t13[2];

    residuals[0] = T(m_x1) - rot_pt[0];
    residuals[1] = T(m_y1) - rot_pt[1];
    residuals[2] = T(m_z1) - rot_pt[2];
    return true;
  }

 private:
  double m_x1;
  double m_y1;
  double m_z1;
  double m_x2;
  double m_y2;
  double m_z2;
};
}  // namespace room

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_REPROJICP_H_
