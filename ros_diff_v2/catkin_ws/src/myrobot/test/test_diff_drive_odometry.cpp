#include <gtest/gtest.h>

#include <cmath>

#include "diff_drive_odometry.h"

TEST(DiffDriveOdometry, IntegratesStraightMotion)
{
  myrobot::Pose2D pose;
  myrobot::integrateDifferentialDrive(pose, 1.0, 0.0, 2.0);

  EXPECT_NEAR(2.0, pose.x, 1e-9);
  EXPECT_NEAR(0.0, pose.y, 1e-9);
  EXPECT_NEAR(0.0, pose.yaw, 1e-9);
}

TEST(DiffDriveOdometry, IntegratesQuarterCircleExactly)
{
  myrobot::Pose2D pose;
  const double half_pi = 1.5707963267948966;
  myrobot::integrateDifferentialDrive(pose, 1.0, 1.0, half_pi);

  EXPECT_NEAR(1.0, pose.x, 1e-9);
  EXPECT_NEAR(1.0, pose.y, 1e-9);
  EXPECT_NEAR(half_pi, pose.yaw, 1e-9);
}

TEST(DiffDriveOdometry, RotatesAroundDriveAxleCenter)
{
  myrobot::Pose2D pose;
  myrobot::integrateDifferentialDrive(pose, 0.0, 1.0, 1.0);

  EXPECT_NEAR(0.0, pose.x, 1e-9);
  EXPECT_NEAR(0.0, pose.y, 1e-9);
  EXPECT_NEAR(1.0, pose.yaw, 1e-9);
}
