#ifndef MECANUM_KINEMATICS__KINEMATICS_HPP_
#define MECANUM_KINEMATICS__KINEMATICS_HPP_

#include <array>

namespace mecanum_kinematics {

// Compute wheel angular speeds (rad/s) from body twist and geometry.
// linear_x, linear_y are m/s; angular_z is rad/s.
// half_length and half_width are meters; wheel_radius is meters.
std::array<double, 4> compute_wheel_speeds(
	double linear_x,
	double linear_y,
	double angular_z,
	double half_length,
	double half_width,
	double wheel_radius);

}  // namespace mecanum_kinematics

#endif  // MECANUM_KINEMATICS__KINEMATICS_HPP_
