// Copyright 2018 Arunabh Sharma

#ifndef MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_ICP_H_
#define MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_ICP_H_

#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace room
{
class ICP
{
   public:
    ICP() = delete;
    ICP(const double x1,
        const double y1,
        const double z1,
        const double x2,
        const double y2,
        const double z2);

    ~ICP() = default;

    template <typename T>
    bool operator()(const T* extrinsics, T* residuals) const
    {
        // ceres quaternion is w, x, y, z
        T rot[4]    = {extrinsics[0], extrinsics[1], extrinsics[2], extrinsics[3]};
        T t[3]      = {extrinsics[4], extrinsics[5], extrinsics[6]};
        T rot_pt[3] = {T(m_x2), T(m_y2), T(m_z2)};

        ceres::QuaternionRotatePoint(rot, rot_pt, rot_pt);
        rot_pt[0] += t[0];
        rot_pt[1] += t[1];
        rot_pt[2] += t[2];

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

#endif  // MODULES_RGBDSLAM_INCLUDE_ROOM_RGBDSLAM_ICP_H_
