#ifndef ZDT__MOTOR_HPP
#define ZDT__MOTOR_HPP

#include <cstdint>

#include "usart.h"

namespace crane
{

constexpr size_t BUFF_SIZE = 255;
constexpr size_t TX_BUF_SIZE = 32;  // 最大发送命令长度，按需可调

enum SysParams_t
{
  S_VER,
  S_RL,
  S_PID,
  S_VBUS,
  S_CPHA,
  S_ENCL,
  S_TPOS,
  S_VEL,
  S_CPOS,
  S_PERR,
  S_FLAG,
  S_ORG,
  S_Conf,
  S_State
};

class ZDT_Motor
{
public:
  ZDT_Motor(UART_HandleTypeDef * huart, uint8_t addr, bool use_dma = true);

  UART_HandleTypeDef * huart;

  float angle;              // rad
  float speed;              // rad/s
  float voltage = 0.0f;     // VBUS 电压
  uint16_t encoder = 0;     // 编码器值（0x31）
  int32_t target_pos = 0;   // 目标位置（0x33 / 0x34）
  int16_t pos_error = 0;    // 位置误差（0x37）
  uint8_t status_flag = 0;  // 电机状态字节（0x3A）

  void request();
  void update(uint16_t size);

  void enableMotor(bool state, bool sync = false);
  void setVelocity(uint8_t dir, uint16_t vel, uint8_t acc, bool sync = false);
  void setPosition(
    uint8_t dir, uint16_t vel, uint8_t acc, uint32_t pulses, bool absolute = false,
    bool sync = false);
  void stopNow(bool sync = false);
  void triggerSyncMotion();
  void setOrigin(bool save = false);
  void setOriginParams(
    bool save, uint8_t mode, uint8_t dir, uint16_t vel, uint32_t timeout, uint16_t detectVel,
    uint16_t detectCurrent, uint16_t detectTime, bool powerOnAuto = false);
  void triggerOrigin(uint8_t mode, bool sync = false);
  void interruptOrigin();
  // === TO DO ===
  void resetCurPosToZero();
  void resetClogProtection();
  void readSysParams(SysParams_t param);
  void setCtrlMode(bool save, uint8_t mode);
  // === TO DO ===

  void readCurrent();
  void readPositionError();
  void factoryReset();
  void readEncoderValue();
  void readTargetPosition();
  void readCurrentSpeed();
  void readCurrentPosition();
  void readMotorStatusFlag();

  bool stopNowWithAck(bool sync = false);
  bool checkAck(uint8_t expected_cmd_id);

private:
  void sendCommand(const uint8_t * data, uint16_t len);

  const bool use_dma_;
  const uint8_t addr_;

  uint8_t buff_[BUFF_SIZE];         // 接收缓冲区
  uint8_t tx_buffer_[TX_BUF_SIZE];  // 发送缓冲区
};

}  // namespace crane

#endif  // ZDT__MOTOR_HPP
