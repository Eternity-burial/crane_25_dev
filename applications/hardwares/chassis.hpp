#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include "motor/zdt_motor/zdt_motor.hpp"

namespace chassis
{
// -------------------- 机械参数 --------------------
constexpr float WHEEL_RADIUS = 77e-3;     // m
constexpr float CHASSIS_LENGTH = 454e-3;  // m
constexpr float CHASSIS_WIDTH = 390e-3;   // m
constexpr size_t BUFF_SIZE = 255;

// -------------------- 对外硬件 --------------------
inline crane::ZDT_Motor motor_lf(&huart1, 1, true);
inline crane::ZDT_Motor motor_lr(&huart1, 2);
inline crane::ZDT_Motor motor_rf(&huart1, 3);
inline crane::ZDT_Motor motor_rr(&huart1, 4);

extern uint8_t huart1_rx_buff[];

}  // namespace chassis

#endif  // CHASSIS_HPP