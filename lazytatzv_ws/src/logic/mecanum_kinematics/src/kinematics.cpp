// Copyright 2026 Tatsukiyano
#include "mecanum_kinematics/kinematics.hpp"
#include <algorithm>
#include <cmath>

// Pure Kinematics computations for mecanum wheels.

namespace mecanum_kinematics
{

std::array<double, 4> compute_wheel_speeds(
  double linear_x,
  double linear_y,
  double angular_z,
  double half_length,
  double half_width,
  double wheel_radius)
{
  const double k = half_length + half_width;
  const double inv_r = 1.0 / wheel_radius;

  // Wheel angular speeds [rad/s]
  const double fl = inv_r * (linear_x - linear_y - k * angular_z);
  const double fr = inv_r * (linear_x + linear_y + k * angular_z);
  const double rl = inv_r * (linear_x + linear_y - k * angular_z);
  const double rr = inv_r * (linear_x - linear_y + k * angular_z);

  return {fl, fr, rl, rr};
}

std::array<double, 3> compute_body_twist(
  const std::array<double, 4> & wheel_speeds,
  double half_length,
  double half_width,
  double wheel_radius)
{
  const double fl = wheel_speeds[0];
  const double fr = wheel_speeds[1];
  const double rl = wheel_speeds[2];
  const double rr = wheel_speeds[3];

  const double k = half_length + half_width;

  // Body linear velocities [m/s] and angular velocity [rad/s]
  // Forward kinematics based on mecanum geometry
  double vx = (fl + fr + rl + rr) * wheel_radius / 4.0;
  double vy = (-fl + fr + rl - rr) * wheel_radius / 4.0;
  double omega = (-fl + fr - rl + rr) * wheel_radius / (4.0 * k);

  return {vx, vy, omega};
}

}  // namespace mecanum_kinematics
