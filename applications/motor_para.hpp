#ifndef _CHASSIS_PARA_HPP
#define _CHASSIS_PARA_HPP
#include <cstdint>

// 任务开始空闲一段时间
static const uint16_t CHASSIS_TASK_INIT_TIME = 357;

constexpr int CONTROL_T_MS = 1;           // ms
constexpr float T = CONTROL_T_MS * 1e-3;  // s

// --- Y轴 ---
constexpr float Y_RADIUS = 0.068f;              // m
constexpr float MAX_Y_SPEED = 0.5f / Y_RADIUS;  // rad/s
constexpr float Y_ACCEL = 2.0f / Y_RADIUS;      // rad/s^2 (假设线性加速度为2.0 m/s^2)

// --- X轴 ---
constexpr float X_RADIUS = 0.01f;               // m
constexpr float MAX_X_SPEED = 0.5f / X_RADIUS;  // rad/s
constexpr float X_ACCEL = 2.0f / X_RADIUS;      // rad/s^2

// --- Z轴前 ---
constexpr float ZF_RADIUS = 0.01495f;               // m
constexpr float MAX_ZF_SPEED = 0.5f / ZF_RADIUS;  // rad/s
constexpr float ZF_ACCEL = 2.0f / ZF_RADIUS;      // rad/s^2

// --- Z轴后 ---
constexpr float ZR_RADIUS = 0.0082f;               // m
constexpr float MAX_ZR_SPEED = 0.07f / ZR_RADIUS;  // rad/s
constexpr float ZR_ACCEL = 2.0f / ZR_RADIUS;      // rad/s^2

typedef struct
{
  // 期望电机角度
  float set_motor_angle[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  // 期望电机速度
  float set_motor_speed[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  // 期望电机力矩
  float set_motor_torque[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

} move_t;

typedef struct
{
  float y = 0.0f;
  float x = 0.0f;
  float zf = 0.0f;
  float zr = 0.0f;
} coordinate_t;

#endif  // _CHASSIS_PARA_HPP