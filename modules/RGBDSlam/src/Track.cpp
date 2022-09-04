// Copyright 2022 Arunabh Sharma

#include "RGBDSlam/Track.h"

#include "RGBDSlam/Feature.h"

namespace slam
{
bool Track::Initialize(Frame& frame)
{
  (void)frame;
  return true;
}

}  // namespace slam