#include "mecanum_kinematics/kinematics.hpp"
#include <algorithm>
#include <cmath>

// Pure Kinematics computations for mecanum wheels.

namespace mecanum_kinematics {

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

}  // namespace mecanum_kinematics
