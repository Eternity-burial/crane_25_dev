#include "zdt_motor.hpp"

#include <cstring>

#include "usart.h"

namespace crane
{

ZDT_Motor::ZDT_Motor(UART_HandleTypeDef * huart, uint8_t addr, bool use_dma)
: huart(huart), use_dma_(use_dma), addr_(addr), angle(0.0f), speed(0.0f)
{
}

void ZDT_Motor::request()
{
  if (use_dma_) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, buff_, BUFF_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else {
    HAL_UARTEx_ReceiveToIdle_IT(huart, buff_, BUFF_SIZE);
  }
}

void ZDT_Motor::update(uint16_t size)
{
  if (buff_[0] != addr_) return;

  uint8_t cmd_id = buff_[1];

  switch (cmd_id) {
    case 0x35:  // 实时速度
      if (size == 6) {
        uint16_t raw_speed = (static_cast<uint16_t>(buff_[3]) << 8) | buff_[4];
        this->speed = buff_[2] ? -static_cast<float>(raw_speed) : static_cast<float>(raw_speed);
      }
      break;

    case 0x36:  // 实时位置
      if (size == 8) {
        int32_t raw_pos = (buff_[2] << 24) | (buff_[3] << 16) | (buff_[4] << 8) | buff_[5];
        angle = static_cast<float>(raw_pos) * 360.0f / 65536.0f;  // 可根据编码器细分换算角度
      }
      break;

    case 0x37:  // 位置误差
      if (size == 8) {
        pos_error = static_cast<int16_t>((buff_[3] << 8) | buff_[4]);
      }
      break;

    case 0x33:  // 目标位置
    case 0x34:  // 当前目标位置
      if (size == 7) {
        target_pos = (buff_[2] << 24) | (buff_[3] << 16) | (buff_[4] << 8) | buff_[5];
      }
      break;

    case 0x3A:  // 状态标志位
      if (size == 4) {
        status_flag = buff_[2];
      }
      break;

    case 0x31:  // 编码器线性化值
      if (size == 6) {
        encoder = (buff_[3] << 8) | buff_[4];
      }
      break;

    case 0x24:  // 电压
      if (size == 5) {
        uint16_t vbus = (buff_[2] << 8) | buff_[3];
        voltage = static_cast<float>(vbus) / 100.0f;  // 单位换算，假设返回的是×100倍
      }
      break;

    default:
      break;
  }
}

void ZDT_Motor::sendCommand(const uint8_t * cmd, uint16_t len)
{
  memcpy(tx_buffer_, cmd, len);
  if (use_dma_) {
    HAL_UART_Transmit_DMA(huart, tx_buffer_, len);
  }
  else {
    HAL_UART_Transmit(huart, tx_buffer_, len, 100);
  }
}

#define TX(cmd_array) sendCommand(cmd_array, sizeof(cmd_array))

void ZDT_Motor::resetCurPosToZero()
{
  uint8_t cmd[] = {addr_, 0x0A, 0x6D, 0x6B};
  TX(cmd);
}

void ZDT_Motor::resetClogProtection()
{
  uint8_t cmd[] = {addr_, 0x0E, 0x52, 0x6B};
  TX(cmd);
}

void ZDT_Motor::readSysParams(SysParams_t param)
{
  uint8_t cmd[16] = {addr_};
  uint8_t i = 1;
  switch (param) {
    case S_VER:
      cmd[i++] = 0x1F;
      break;
    case S_RL:
      cmd[i++] = 0x20;
      break;
    case S_PID:
      cmd[i++] = 0x21;
      break;
    case S_VBUS:
      cmd[i++] = 0x24;
      break;
    case S_CPHA:
      cmd[i++] = 0x27;
      break;
    case S_ENCL:
      cmd[i++] = 0x31;
      break;
    case S_TPOS:
      cmd[i++] = 0x33;
      break;
    case S_VEL:
      cmd[i++] = 0x35;
      break;
    case S_CPOS:
      cmd[i++] = 0x36;
      break;
    case S_PERR:
      cmd[i++] = 0x37;
      break;
    case S_FLAG:
      cmd[i++] = 0x3A;
      break;
    case S_ORG:
      cmd[i++] = 0x3B;
      break;
    case S_Conf:
      cmd[i++] = 0x42;
      cmd[i++] = 0x6C;
      break;
    case S_State:
      cmd[i++] = 0x43;
      cmd[i++] = 0x7A;
      break;
  }
  cmd[i++] = 0x6B;
  sendCommand(cmd, i);
}

void ZDT_Motor::setCtrlMode(bool save, uint8_t mode)
{
  uint8_t cmd[] = {addr_, 0x46, 0x69, static_cast<uint8_t>(save), mode, 0x6B};
  TX(cmd);
}

void ZDT_Motor::enableMotor(bool state, bool sync)
{
  uint8_t cmd[] = {addr_, 0xF3, 0xAB, static_cast<uint8_t>(state), static_cast<uint8_t>(sync),
                   0x6B};
  TX(cmd);
}

void ZDT_Motor::setVelocity(uint8_t dir, uint16_t vel, uint8_t acc, bool sync)
{
  uint8_t cmd[] = {
    addr_,
    0xF6,
    dir,
    static_cast<uint8_t>(vel >> 8),
    static_cast<uint8_t>(vel),
    acc,
    static_cast<uint8_t>(sync),
    0x6B};
  TX(cmd);
}

void ZDT_Motor::setPosition(
  uint8_t dir, uint16_t vel, uint8_t acc, uint32_t pulses, bool absolute, bool sync)
{
  uint8_t cmd[] = {
    addr_,
    0xFD,
    dir,
    static_cast<uint8_t>(vel >> 8),
    static_cast<uint8_t>(vel),
    acc,
    static_cast<uint8_t>(pulses >> 24),
    static_cast<uint8_t>(pulses >> 16),
    static_cast<uint8_t>(pulses >> 8),
    static_cast<uint8_t>(pulses),
    static_cast<uint8_t>(absolute),
    static_cast<uint8_t>(sync),
    0x6B};
  TX(cmd);
}

void ZDT_Motor::stopNow(bool sync)
{
  uint8_t cmd[] = {addr_, 0xFE, 0x98, static_cast<uint8_t>(sync), 0x6B};
  TX(cmd);
}

void ZDT_Motor::triggerSyncMotion()
{
  uint8_t cmd[] = {addr_, 0xFF, 0x66, 0x6B};
  TX(cmd);
}

void ZDT_Motor::setOrigin(bool save)
{
  uint8_t cmd[] = {addr_, 0x93, 0x88, static_cast<uint8_t>(save), 0x6B};
  TX(cmd);
}

void ZDT_Motor::setOriginParams(
  bool save, uint8_t mode, uint8_t dir, uint16_t vel, uint32_t timeout, uint16_t detectVel,
  uint16_t detectCurrent, uint16_t detectTime, bool powerOnAuto)
{
  uint8_t cmd[] = {
    addr_,
    0x4C,
    0xAE,
    static_cast<uint8_t>(save),
    mode,
    dir,
    static_cast<uint8_t>(vel >> 8),
    static_cast<uint8_t>(vel),
    static_cast<uint8_t>(timeout >> 24),
    static_cast<uint8_t>(timeout >> 16),
    static_cast<uint8_t>(timeout >> 8),
    static_cast<uint8_t>(timeout),
    static_cast<uint8_t>(detectVel >> 8),
    static_cast<uint8_t>(detectVel),
    static_cast<uint8_t>(detectCurrent >> 8),
    static_cast<uint8_t>(detectCurrent),
    static_cast<uint8_t>(detectTime >> 8),
    static_cast<uint8_t>(detectTime),
    static_cast<uint8_t>(powerOnAuto),
    0x6B};
  TX(cmd);
}

void ZDT_Motor::triggerOrigin(uint8_t mode, bool sync)
{
  uint8_t cmd[] = {addr_, 0x9A, mode, static_cast<uint8_t>(sync), 0x6B};
  TX(cmd);
}

void ZDT_Motor::interruptOrigin()
{
  uint8_t cmd[] = {addr_, 0x9C, 0x48, 0x6B};
  TX(cmd);
}

void ZDT_Motor::readCurrent()
{
  uint8_t cmd[] = {addr_, 0x27, 0x6B};
  TX(cmd);
}

void ZDT_Motor::readPositionError()
{
  uint8_t cmd[] = {addr_, 0x37, 0x6B};
  TX(cmd);
}

void ZDT_Motor::factoryReset()
{
  uint8_t cmd[] = {addr_, 0x0F, 0x5F, 0x6B};
  TX(cmd);
}

void ZDT_Motor::readEncoderValue()
{
  uint8_t cmd[] = {addr_, 0x31, 0x6B};
  TX(cmd);
}

void ZDT_Motor::readTargetPosition()
{
  uint8_t cmd[] = {addr_, 0x33, 0x6B};
  TX(cmd);
}

void ZDT_Motor::readCurrentSpeed()
{
  uint8_t cmd[] = {addr_, 0x35, 0x6B};
  TX(cmd);
}
void ZDT_Motor::readCurrentPosition()
{
  uint8_t cmd[] = {addr_, 0x36, 0x6B};
  TX(cmd);
}

void ZDT_Motor::readMotorStatusFlag()
{
  uint8_t cmd[] = {addr_, 0x3A, 0x6B};
  TX(cmd);
}

bool ZDT_Motor::checkAck(uint8_t expected_cmd_id)
{
  if (buff_[0] != addr_) return false;
  if (buff_[1] != expected_cmd_id) return false;
  if (buff_[2] == 0x02) return true;   // 正确
  if (buff_[2] == 0xE2) return false;  // 条件不满足
  if (buff_[2] == 0xEE) return false;  // 格式错误
  return false;
}

#undef TX

}  // namespace crane
