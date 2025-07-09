#ifndef CONTROLLERS_HPP
#define CONTROLLERS_HPP

// #include "diff_gear_controller/diff_gear_controller.hpp"
// #include "gantry_controller/gantry_controller.hpp"
// #include "joint_motor_controller/joint_motor_controller.hpp"
#include "mecanum_controller/mecanum_controller.hpp"
#include "pids.hpp"

inline MecanumController mecanum_chassis(
  chassis::WHEEL_RADIUS, chassis::CHASSIS_LENGTH / 2, chassis::CHASSIS_WIDTH / 2, false, false,
  true, true, chassis::motor_lf, chassis::motor_lr, chassis::motor_rf, chassis::motor_rr,
  chassis::motor_lf_speed_pid, chassis::motor_lr_speed_pid, chassis::motor_rf_speed_pid,
  chassis::motor_rr_speed_pid);

#endif  // CONTROLLERS_HPP