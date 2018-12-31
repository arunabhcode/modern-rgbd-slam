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
    T rot33[9];

    rot13[0] = extrinsics[0];
    rot13[1] = extrinsics[1];
    rot13[2] = extrinsics[2];

    t13[0] = extrinsics[3];
    t13[1] = extrinsics[4];
    t13[2] = extrinsics[5];

    ceres::AngleAxisToRotationMatrix(rot13, rot33);

    residuals[0] = pow(
        m_x1 - (rot33[0] * m_x2 + rot33[1] * m_y2 + rot33[2] * m_z2) - t13[0],
        2);
    residuals[1] = pow(
        m_y1 - (rot33[3] * m_x2 + rot33[4] * m_y2 + rot33[5] * m_z2) - t13[1],
        2);
    residuals[2] = pow(
        m_z1 - (rot33[6] * m_x2 + rot33[7] * m_y2 + rot33[8] * m_z2) - t13[2],
        2);
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
