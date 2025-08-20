#include <cmath>  // For std::fabs

#include "cmsis_os.h"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_para.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"
#include "velocity_smoother.hpp"  //  <-- 1. 包含新头文件

//  电机
sp::RM_Motor motoryl(1, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motoryr(2, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motorx(3, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motorzf(4, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motorzr(5, sp::RM_Motors::M3508, sp::M3508_P19);

//  PID
sp::PID motoryl_position_pid(T, 20, 0, 0.0001, MAX_Y_SPEED, MAX_Y_SPEED, 0.001);
sp::PID motoryr_position_pid(T, 20, 0, 0.0001, MAX_Y_SPEED, MAX_Y_SPEED, 0.001);
sp::PID motoryl_speed_pid(T, 0.15, 0.03, 0.0001, 3.6, 0.4, 0.001);
sp::PID motoryr_speed_pid(T, 0.15, 0.03, 0.0001, 3.6, 0.4, 0.001);

sp::PID motorx_position_pid(T, 10, 0, 0.0001, MAX_X_SPEED, MAX_X_SPEED, 0.001);
sp::PID motorx_speed_pid(T, 0.05, 0., 0.0001, 3.6, 0.4, 0.001);

sp::PID motorzf_position_pid(T, 20, 0, 0.0001, MAX_ZF_SPEED, MAX_ZF_SPEED, 0.001);
sp::PID motorzr_position_pid(T, 15, 0, 0.0001, MAX_ZR_SPEED, MAX_ZR_SPEED, 0.001);
sp::PID motorzf_speed_pid(T, 0.07, 0.01, 0.0001, 3.6, 0.4, 0.001);
sp::PID motorzr_speed_pid(T, 0.10, 0., 0.0001, 3.6, 0.4, 0.001);

//  VelocitySmoother
sp::VelocitySmoother motoryl_smoother(MAX_Y_SPEED, Y_ACCEL, T);
sp::VelocitySmoother motoryr_smoother(MAX_Y_SPEED, Y_ACCEL, T);
sp::VelocitySmoother motorx_smoother(MAX_X_SPEED, X_ACCEL, T);
sp::VelocitySmoother motorzf_smoother(MAX_ZF_SPEED, ZF_ACCEL, T);
sp::VelocitySmoother motorzr_smoother(MAX_ZR_SPEED, ZR_ACCEL, T);

move_t move_data;
coordinate_t coordinate_data;

// 函数声明
void angle_calc(float coord_y, float coord_x, float coord_zf, float coord_zr);
void speed_calc();
void plan_speed();
void torque_calc();
void move_calc(float coord_y, float coord_x, float coord_zf, float coord_zr);

extern "C" void motor_task()
{
  osDelay(1000);
  while (true) {
    coordinate_data.y = 0.0f;
    coordinate_data.x = -0.0f;
    coordinate_data.zf = 0.00f;
    coordinate_data.zr = 0.00f;
    move_calc(coordinate_data.y, coordinate_data.x, coordinate_data.zf, coordinate_data.zr);
    osDelay(1);
  }
}

void angle_calc(float coord_y, float coord_x, float coord_zf, float coord_zr)
{
  move_data.set_motor_angle[0] = -coord_y / Y_RADIUS;
  move_data.set_motor_angle[1] = coord_y / Y_RADIUS;

  move_data.set_motor_angle[2] = coord_x / X_RADIUS;

  move_data.set_motor_angle[3] = -coord_zf / ZF_RADIUS;
  move_data.set_motor_angle[4] = coord_zr / ZR_RADIUS;
}

void speed_calc()
{
  motoryl_position_pid.calc(move_data.set_motor_angle[0], motoryl.angle);
  motoryr_position_pid.calc(move_data.set_motor_angle[1], motoryr.angle);
  move_data.set_motor_speed[0] = motoryl_position_pid.out;
  move_data.set_motor_speed[1] = motoryr_position_pid.out;

  motorx_position_pid.calc(move_data.set_motor_angle[2], motorx.angle);
  move_data.set_motor_speed[2] = motorx_position_pid.out;

  motorzf_position_pid.calc(move_data.set_motor_angle[3], motorzf.angle);
  motorzr_position_pid.calc(move_data.set_motor_angle[4], motorzr.angle);
  move_data.set_motor_speed[3] = motorzf_position_pid.out;
  move_data.set_motor_speed[4] = motorzr_position_pid.out;
}

// <-- 4. 实现新的速度规划函数
void plan_speed()
{
  // 对 speed_calc() 输出的原始目标速度进行规划
  // 只在加速时（目标速度绝对值 > 当前速度绝对值）使用斜坡
  // 减速时直接采用目标值，以实现最快响应

  // Y轴左电机
  float raw_target_yl = move_data.set_motor_speed[0];
  if (std::fabs(raw_target_yl) > std::fabs(motoryl_smoother.current_velocity)) {
    motoryl_smoother.update(raw_target_yl);
  }
  else {
    motoryl_smoother.current_velocity = raw_target_yl;
  }
  move_data.set_motor_speed[0] = motoryl_smoother.current_velocity;

  // Y轴右电机
  float raw_target_yr = move_data.set_motor_speed[1];
  if (std::fabs(raw_target_yr) > std::fabs(motoryr_smoother.current_velocity)) {
    motoryr_smoother.update(raw_target_yr);
  }
  else {
    motoryr_smoother.current_velocity = raw_target_yr;
  }
  move_data.set_motor_speed[1] = motoryr_smoother.current_velocity;

  // X轴电机
  float raw_target_x = move_data.set_motor_speed[2];
  if (std::fabs(raw_target_x) > std::fabs(motorx_smoother.current_velocity)) {
    motorx_smoother.update(raw_target_x);
  }
  else {
    motorx_smoother.current_velocity = raw_target_x;
  }
  move_data.set_motor_speed[2] = motorx_smoother.current_velocity;

  // Z轴前电机
  float raw_target_zf = move_data.set_motor_speed[3];
  if (std::fabs(raw_target_zf) > std::fabs(motorzf_smoother.current_velocity)) {
    motorzf_smoother.update(raw_target_zf);
  }
  else {
    motorzf_smoother.current_velocity = raw_target_zf;
  }
  move_data.set_motor_speed[3] = motorzf_smoother.current_velocity;

  // Z轴后电机
  float raw_target_zr = move_data.set_motor_speed[4];
  if (std::fabs(raw_target_zr) > std::fabs(motorzr_smoother.current_velocity)) {
    motorzr_smoother.update(raw_target_zr);
  }
  else {
    motorzr_smoother.current_velocity = raw_target_zr;
  }
  move_data.set_motor_speed[4] = motorzr_smoother.current_velocity;
}

void torque_calc()
{
  motoryl_speed_pid.calc(move_data.set_motor_speed[0], motoryl.speed);
  motoryr_speed_pid.calc(move_data.set_motor_speed[1], motoryr.speed);
  move_data.set_motor_torque[0] = motoryl_speed_pid.out;
  move_data.set_motor_torque[1] = motoryr_speed_pid.out;

  motorx_speed_pid.calc(move_data.set_motor_speed[2], motorx.speed);
  move_data.set_motor_torque[2] = motorx_speed_pid.out;

  motorzf_speed_pid.calc(move_data.set_motor_speed[3], motorzf.speed);
  motorzr_speed_pid.calc(move_data.set_motor_speed[4], motorzr.speed);
  move_data.set_motor_torque[3] = motorzf_speed_pid.out;
  move_data.set_motor_torque[4] = motorzr_speed_pid.out;
}

void move_calc(float coord_y, float coord_x, float coord_zf, float coord_zr)
{
  // <-- 5. 调整函数调用顺序
  angle_calc(coord_y, coord_x, coord_zf, coord_zr);
  speed_calc();
  // plan_speed();  // 在速度环之前进行速度规划
  torque_calc();
}