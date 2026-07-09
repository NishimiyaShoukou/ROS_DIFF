#ifndef MYROBOT_DIFF_DRIVE_ODOMETRY_H
#define MYROBOT_DIFF_DRIVE_ODOMETRY_H

#include <cmath>

namespace myrobot
{

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

inline void integrateDifferentialDrive(Pose2D& pose, double linear_velocity,
                                       double angular_velocity, double dt)
{
  if (dt <= 0.0)
  {
    return;
  }

  const double distance = linear_velocity * dt;
  const double delta_yaw = angular_velocity * dt;
  double body_x;
  double body_y;

  if (std::abs(delta_yaw) < 1e-6)
  {
    body_x = distance;
    body_y = 0.0;
  }
  else
  {
    const double radius = distance / delta_yaw;
    body_x = radius * std::sin(delta_yaw);
    body_y = radius * (1.0 - std::cos(delta_yaw));
  }

  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  pose.x += cos_yaw * body_x - sin_yaw * body_y;
  pose.y += sin_yaw * body_x + cos_yaw * body_y;
  pose.yaw += delta_yaw;
}

}  // namespace myrobot

#endif  // MYROBOT_DIFF_DRIVE_ODOMETRY_H
