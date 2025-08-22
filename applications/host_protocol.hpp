#pragma once
#include <cstdint>

namespace host
{

// ---- CAN IDs ----
static constexpr uint32_t ID_BLDCPOS = 0x310;    // X/Y/ZF/ZR 位置指令（米）
static constexpr uint32_t ID_SERVO_POS = 0x350;  // Z1/Z2 舵机角度（度）

// ---- 3508 轴编码 ----
enum Axis : uint8_t
{
  AX_X = 0,
  AX_Y = 1,
  AX_ZF = 2,
  AX_ZR = 3
};

// ---- 舵机索引 ----
enum Servo : uint8_t
{
  SV_Z1 = 0,
  SV_Z2 = 1
};

}  // namespace host
