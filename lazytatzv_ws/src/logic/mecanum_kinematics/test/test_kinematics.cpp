#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "mecanum_kinematics/kinematics.hpp"

TEST(KinematicsTest, ForwardOnly)
{
  const auto out = mecanum_kinematics::compute_wheel_speeds(1.0, 0.0, 0.0, 0.12, 0.10, 0.05);
  EXPECT_NEAR(out[0], 20.0, 1e-9);
  EXPECT_NEAR(out[1], 20.0, 1e-9);
  EXPECT_NEAR(out[2], 20.0, 1e-9);
  EXPECT_NEAR(out[3], 20.0, 1e-9);
}

TEST(KinematicsTest, RotationOnly)
{
  const auto out = mecanum_kinematics::compute_wheel_speeds(0.0, 0.0, 1.0, 0.12, 0.10, 0.05);
  EXPECT_NEAR(out[0], -(0.12 + 0.10) / 0.05, 1e-9);
  EXPECT_NEAR(out[1], +(0.12 + 0.10) / 0.05, 1e-9);
  EXPECT_NEAR(out[2], -(0.12 + 0.10) / 0.05, 1e-9);
  EXPECT_NEAR(out[3], +(0.12 + 0.10) / 0.05, 1e-9);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
