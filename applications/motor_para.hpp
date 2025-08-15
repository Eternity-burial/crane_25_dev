#ifndef _CHASSIS_PARA_HPP
#define _CHASSIS_PARA_HPP
#include <cstdint>

// 任务开始空闲一段时间
static const uint16_t CHASSIS_TASK_INIT_TIME = 357;

//底盘控制模式
enum class Mode
{
  zero_force_mode,
  rc_gimbal_follow_mode,
  up_control_mode,
  topple_mode
};

static const float wheel_radius = 0.0705f;

typedef struct
{
  //速度滤波，暂未用到
  float vx_last = 0.0f;
  float vy_last = 0.0f;
  float vw_last = 0.0f;

  float vx_set = 0.0f;
  float vy_set = 0.0f;
  float vw_set = 0.0f;

  float set_buffer = 600.f;
  float limit_power = 30.f;

  bool under_attack = true;

  bool varying = true;
  uint16_t hp_last = 0;

  uint16_t hp_cnt = 400;
  // 期望电机速度
  float set_motor_speed[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  // 期望电机力矩
  float set_motor_torque[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

} move_t;

typedef struct
{
  float vx_last = 0.0f;
  float vy_last = 0.0f;
  float vw_last = 0.0f;

  float vx_set = 0.0f;
  float vy_set = 0.0f;
  float vw_set = 0.0f;

  float imu_yaw = 0.0f;

  uint16_t chassis_follow = true;

} navigation_t;

#endif  // _CHASSIS_PARA_HPP