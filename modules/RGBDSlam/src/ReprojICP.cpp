// Copyright 2018 Arunabh Sharma

#include "Room/RGBDSlam/ReprojICP.h"

namespace room
{
ReprojICP::ReprojICP(const double x1,
                     const double y1,
                     const double z1,
                     const double x2,
                     const double y2,
                     const double z2)
    : m_x1(x1), m_y1(y1), m_z1(z1), m_x2(x2), m_y2(y2), m_z2(z2)
{
}

}  // namespace room
