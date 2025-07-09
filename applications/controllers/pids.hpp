#ifndef PIDS_HPP
#define PIDS_HPP

// #include "hardwares/arm.hpp"
#include "hardwares/chassis.hpp"
#include "tools/pid/pid.hpp"

constexpr int CONTROL_T_MS = 1;           // ms
constexpr float T = CONTROL_T_MS * 1e-3;  // s

namespace arm
{

}  // namespace arm

namespace chassis
{
inline crane::PID motor_lf_speed_pid(T, 2, 0, 0, 3, 3);
inline crane::PID motor_lr_speed_pid(T, 2, 0, 0, 3, 3);
inline crane::PID motor_rf_speed_pid(T, 2, 0, 0, 3, 3);
inline crane::PID motor_rr_speed_pid(T, 2, 0, 0, 3, 3);

}  // namespace chassis

#endif  // PIDS_HPP