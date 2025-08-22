#pragma once
#include <cstdint>

struct ServoTarget
{
  float angle_deg{0};  // 目标角度（度）
  float speed_dps{0};  // 0 => 沿用上次
  float acc_dps2{0};   // 0 => 沿用上次
  volatile bool pending{false};
};

// 在 motor_task.cpp 里定义
extern ServoTarget g_servo_target[2];
