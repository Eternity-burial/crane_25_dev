#ifndef MECANUM_CONTROLLER_HPP
#define MECANUM_CONTROLLER_HPP

#include "controllers/control_mode.hpp"
#include "motor/zdt_motor/zdt_motor.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

class MecanumController
{
public:
  MecanumController(
    float wheel_radius, float half_length, float half_width, bool reverse_lf, bool reverse_lr,
    bool reverse_rf, bool reverse_rr, crane::ZDT_Motor & motor_lf, crane::ZDT_Motor & motor_lr,
    crane::ZDT_Motor & motor_rf, crane::ZDT_Motor & motor_rr, crane::PID & motor_lf_speed_pid,
    crane::PID & motor_lr_speed_pid, crane::PID & motor_rf_speed_pid,
    crane::PID & motor_rr_speed_pid);

  void enable();
  void disable();
  void cmd_v(float vx, float vy, float wz);
  void control();

private:
  crane::Mecanum wheels_;

  crane::ZDT_Motor & motor_lf_;
  crane::ZDT_Motor & motor_lr_;
  crane::ZDT_Motor & motor_rf_;
  crane::ZDT_Motor & motor_rr_;

  crane::PID & motor_lf_speed_pid_;
  crane::PID & motor_lr_speed_pid_;
  crane::PID & motor_rf_speed_pid_;
  crane::PID & motor_rr_speed_pid_;

  ControlMode mode_ = ControlMode::DISABLE;
};

#endif  // MECANUM_CONTROLLER_HPP