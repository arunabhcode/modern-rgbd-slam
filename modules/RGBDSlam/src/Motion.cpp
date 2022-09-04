// Copyright 2019 Arunabh Sharma

#include "RGBDSlam/Motion.h"

#include <cmath>

#include "Components/Pose.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

namespace room
{
Motion::Motion()
    : m_prev_pose(Pose())
    , m_prev_timestamp(0)
    , m_is_initialized(false)
    , m_v_linear(Eigen::Vector3f(0.0f, 0.0f, 0.0f))
    , m_v_angular(0.0f)
    , m_angular_axis(Eigen::Vector3f(0.0f, 0.0f, 0.0f))
    , m_damping_ratio(0.95f)
{
}

Pose& Motion::GetCurrentPose()
{
  return m_prev_pose;
}

Pose Motion::PredictPose(const uint64_t& current_timestamp)
{
  if (!m_is_initialized)
  {
    m_prev_timestamp = current_timestamp;
    m_is_initialized = true;
    return m_prev_pose;
  }

  uint64_t dt = current_timestamp - m_prev_timestamp;
  Eigen::Quaternionf delta_orientation;
  delta_orientation =
      Eigen::AngleAxisf(m_v_angular * dt * m_damping_ratio, m_angular_axis);

  Pose predicted_pose;
  predicted_pose.m_position =
      predicted_pose.m_position + m_v_linear * dt * m_damping_ratio;
  predicted_pose.m_orientation =
      predicted_pose.m_orientation * delta_orientation;
  return predicted_pose;
}

void Motion::UpdatePose(const Pose& new_pose, const uint64_t& new_timestamp)
{
  uint64_t dt = new_timestamp - m_prev_timestamp;
  if (!m_is_initialized && dt == 0)
  {
    spdlog::warn("The time difference for update pose is 0");
    return;
  }
  m_v_linear = (new_pose.m_position - m_prev_pose.m_position) / dt;

  Eigen::Quaternionf delta_orientation =
      m_prev_pose.m_orientation.inverse() * new_pose.m_orientation;
  delta_orientation.normalize();

  Eigen::AngleAxisf delta_angleaxis;
  delta_angleaxis            = delta_orientation;
  Eigen::Vector3f delta_axis = delta_angleaxis.axis();
  float delta_angle          = delta_angleaxis.angle();

  if (delta_angle > M_PI)
  {
    delta_axis  = -1 * delta_axis;
    delta_angle = 2 * M_PI - delta_angle;
  }
  m_v_angular    = delta_angle / dt;
  m_angular_axis = delta_axis;

  m_prev_timestamp          = new_timestamp;
  m_prev_pose.m_orientation = new_pose.m_orientation;
  m_prev_pose.m_position    = new_pose.m_position;
}

}  // namespace room
