#ifndef ZDT__MOTOR_HPP
#define ZDT__MOTOR_HPP

#include <cstdint>

#include "usart.h"

namespace crane
{
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
constexpr size_t BUFF_SIZE = 255;
constexpr size_t TX_BUF_SIZE = 32;  // ���������ȣ�����ɵ�

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
  float voltage = 0.0f;     // VBUS ��ѹ
  uint16_t encoder = 0;     // ������ֵ��0x31��
  int32_t target_pos = 0;   // Ŀ��λ�ã�0x33 / 0x34��
  int16_t pos_error = 0;    // λ����0x37��
  uint8_t status_flag = 0;  // ���״̬�ֽڣ�0x3A��

  uint8_t buff_[BUFF_SIZE];  // ���ջ�����  //ֻ����

  void request();
  void update(uint16_t size);

  void enableMotor(bool state, bool sync = false);
  void setVelocity(uint8_t dir, uint16_t vel, uint8_t acc, bool sync = false);
  void setPosition(
    uint8_t dir, uint16_t vel, uint8_t acc, uint32_t pulses, bool absolute = false,
    bool sync = false);
  void setPositionWithRadUnits(
    double angle_rad, double velocity_rad_s, uint8_t acc, bool absolute = false, bool sync = false);
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
  static constexpr double PULSES_PER_REVOLUTION = 3200.0;
  static constexpr double VELOCITY_RADS_TO_RPM = 30.0 / M_PI;  // ת������: (60 / 2��)
  // ���巽���������Ӵ���ɶ���
  static const uint8_t DIR_CW = 0x00;   // ˳ʱ��
  static const uint8_t DIR_CCW = 0x01;  // ��ʱ��
  uint8_t tx_buffer_[TX_BUF_SIZE];      // ���ͻ�����
};

}  // namespace crane

#endif  // ZDT__MOTOR_HPP
