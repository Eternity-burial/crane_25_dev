#include "zdt_motor.hpp"

#include "usart.h"

namespace crane
{
ZDT_Motor::ZDT_Motor(UART_HandleTypeDef * huart, uint8_t addr, bool use_dma)
: huart(huart), use_dma_(use_dma), addr_(addr)
{
}

void ZDT_Motor::request()
{
  if (use_dma_) {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(huart, buff_, BUFF_SIZE);

    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(huart, buff_, BUFF_SIZE);
  }
}
void ZDT_Motor::update(uint16_t size)
{
  // if (len < 6) return; // 最小长度校验
  if (buff_[0] != addr_) return;  // 地址不匹配

  uint8_t cmd_id = buff_[1];
  switch (cmd_id) {
    case 0x35:  // 速度反馈
      if (size == 6 && buff_[0] == addr_ && buff_[1] == 0x35) {
        uint16_t raw_speed = ((uint16_t)buff_[3] << 8) | buff_[4];
        speed = (float)raw_speed;
        if (buff_[2]) speed = -speed;
      }
      break;

      // case 0x36:  // 位置反馈（示例）
      //   parsePositionFrame(data, len);
      //   break;

    default:
      break;
  }
}
void ZDT_Motor::resetCurPosToZero()
{
  uint8_t cmd[4] = {addr_, 0x0A, 0x6D, 0x6B};
  HAL_UART_Transmit_DMA(huart, cmd, 4);
}

void ZDT_Motor::resetClogProtection()
{
  uint8_t cmd[4] = {addr_, 0x0E, 0x52, 0x6B};
  HAL_UART_Transmit_DMA(huart, cmd, 4);
}

void ZDT_Motor::readSysParams(SysParams_t param)
{
  uint8_t cmd[16] = {0};
  uint8_t i = 0;
  cmd[i++] = addr_;

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
  HAL_UART_Transmit_DMA(huart, cmd, i);
}

void ZDT_Motor::setCtrlMode(bool save, uint8_t mode)
{
  uint8_t cmd[6] = {addr_, 0x46, 0x69, static_cast<uint8_t>(save), mode, 0x6B};
  HAL_UART_Transmit_DMA(huart, cmd, 6);
}

void ZDT_Motor::enableMotor(bool state, bool sync)
{
  uint8_t cmd[6] = {addr_, 0xF3, 0xAB, static_cast<uint8_t>(state), static_cast<uint8_t>(sync),
                    0x6B};
  HAL_UART_Transmit_DMA(huart, cmd, 6);
}

void ZDT_Motor::setVelocity(uint8_t dir, uint16_t vel, uint8_t acc, bool sync)
{
  uint8_t cmd[8] = {
    addr_,
    0xF6,
    dir,
    static_cast<uint8_t>(vel >> 8),
    static_cast<uint8_t>(vel),
    acc,
    static_cast<uint8_t>(sync),
    0x6B};
  HAL_UART_Transmit_DMA(huart, cmd, 8);
}

void ZDT_Motor::setPosition(
  uint8_t dir, uint16_t vel, uint8_t acc, uint32_t pulses, bool absolute, bool sync)
{
  uint8_t cmd[13] = {
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
  HAL_UART_Transmit_DMA(huart, cmd, 13);
}

void ZDT_Motor::stopNow(bool sync)
{
  uint8_t cmd[5] = {addr_, 0xFE, 0x98, static_cast<uint8_t>(sync), 0x6B};
  HAL_UART_Transmit_DMA(huart, cmd, 5);
}

void ZDT_Motor::triggerSyncMotion()
{
  uint8_t cmd[4] = {addr_, 0xFF, 0x66, 0x6B};
  HAL_UART_Transmit_DMA(huart, cmd, 4);
}

void ZDT_Motor::setOrigin(bool save)
{
  uint8_t cmd[5] = {addr_, 0x93, 0x88, static_cast<uint8_t>(save), 0x6B};
  HAL_UART_Transmit_DMA(huart, cmd, 5);
}

void ZDT_Motor::setOriginParams(
  bool save, uint8_t mode, uint8_t dir, uint16_t vel, uint32_t timeout, uint16_t detectVel,
  uint16_t detectCurrent, uint16_t detectTime, bool powerOnAuto)
{
  uint8_t cmd[20] = {
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
  HAL_UART_Transmit_DMA(huart, cmd, 20);
}

void ZDT_Motor::triggerOrigin(uint8_t mode, bool sync)
{
  uint8_t cmd[5] = {addr_, 0x9A, mode, static_cast<uint8_t>(sync), 0x6B};
  HAL_UART_Transmit_DMA(huart, cmd, 5);
}

void ZDT_Motor::interruptOrigin()
{
  uint8_t cmd[4] = {addr_, 0x9C, 0x48, 0x6B};
  HAL_UART_Transmit_DMA(huart, cmd, 4);
}
}  // namespace crane