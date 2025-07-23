#include "mecanum_controller.hpp"

#include "cmsis_os.h"
MecanumController::MecanumController(
  float wheel_radius, float half_length, float half_width, bool reverse_lf, bool reverse_lr,
  bool reverse_rf, bool reverse_rr, crane::ZDT_Motor & motor_lf, crane::ZDT_Motor & motor_lr,
  crane::ZDT_Motor & motor_rf, crane::ZDT_Motor & motor_rr, crane::PID & motor_lf_speed_pid,
  crane::PID & motor_lr_speed_pid, crane::PID & motor_rf_speed_pid, crane::PID & motor_rr_speed_pid)
: wheels_(wheel_radius, half_length, half_width, reverse_lf, reverse_lr, reverse_rf, reverse_rr),
  motor_lf_(motor_lf),
  motor_lr_(motor_lr),
  motor_rf_(motor_rf),
  motor_rr_(motor_rr),
  motor_lf_speed_pid_(motor_lf_speed_pid),
  motor_lr_speed_pid_(motor_lr_speed_pid),
  motor_rf_speed_pid_(motor_rf_speed_pid),
  motor_rr_speed_pid_(motor_rr_speed_pid)
{
}

void MecanumController::enable()
{
  mode_ = ControlMode::VELOCITY;
  motor_lf_.enableMotor(true);
  osDelay(10);
  motor_lr_.enableMotor(true);
  osDelay(10);
  motor_rf_.enableMotor(true);
  osDelay(10);
  motor_rr_.enableMotor(true);
  osDelay(10);
}
void MecanumController::disable() { mode_ = ControlMode::DISABLE; }

void MecanumController::cmd_v(float vx, float vy, float wz)
{
  mode_ = ControlMode::VELOCITY;
  wheels_.calc(vx, vy, wz);
}

void MecanumController::control()
{
  // if (mode_ == ControlMode::DISABLE) {
  //   motor_lf_.setVelocity(0, 0, 0, 0);
  //   motor_lr_.setVelocity(0, 0, 0, 0);
  //   motor_rf_.setVelocity(0, 0, 0, 0);
  //   motor_rr_.setVelocity(0, 0, 0, 0);
  //   return;
  // }
  motor_lf_.readCurrentSpeed();
  osDelay(1);
  motor_lf_speed_pid_.calc(wheels_.speed_lf, motor_lf_.speed);

  motor_lr_.readCurrentSpeed();
  osDelay(1);
  motor_lr_speed_pid_.calc(wheels_.speed_lr, motor_lr_.speed);

  motor_rf_.readCurrentSpeed();
  osDelay(1);
  motor_rf_speed_pid_.calc(wheels_.speed_rf, motor_rf_.speed);

  motor_rr_.readCurrentSpeed();
  osDelay(1);
  motor_rr_speed_pid_.calc(wheels_.speed_rr, motor_rr_.speed);

  // motor_lf_.setVelocity(0, motor_lf_speed_pid_.out, 0, 0);
  // motor_lr_.setVelocity(0, motor_lr_speed_pid_.out, 0, 0);
  // motor_lr_.setVelocity(0, 100, 0, 0);
  // osDelay(5000);
  // motor_lr_.setVelocity(0, 0, 0, 0);
  // osDelay(5000);
  // motor_rf_.setVelocity(0, motor_rf_speed_pid_.out, 0, 0);
  // motor_rr_.setVelocity(0, motor_rr_speed_pid_.out, 0, 0);
}