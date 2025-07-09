#ifndef ZDT_MOTOR_HPP
#define ZDT_MOTOR_HPP

#include <array>
#include <cstdint>

#include "usart.h"

namespace crane
{  // ϵͳ��������ö�٣����ڶ�ȡ��ͬ��ϵͳ����
constexpr size_t BUFF_SIZE = 255;
typedef enum
{
  S_VER,   // �汾��
  S_RL,    // ����״̬
  S_PID,   // PID����
  S_VBUS,  // ���ߵ�ѹ
  S_CPHA,  // ������λ
  S_ENCL,  // ������ֵ
  S_TPOS,  // Ŀ��λ��
  S_VEL,   // ��ǰ�ٶ�
  S_CPOS,  // ��ǰλ��
  S_PERR,  // λ�����
  S_FLAG,  // ״̬��־
  S_ORG,   // ԭ��λ��
  S_Conf,  // ���ò���
  S_State  // ϵͳ״̬
} SysParams_t;

class ZDT_Motor
{
public:
  /**
     * @brief ���캯��
     * @param addr �����ַ��1~255��
     */
  ZDT_Motor(UART_HandleTypeDef * huart, uint8_t addr, bool use_dma = true);
  UART_HandleTypeDef * huart;

  float angle;  // ֻ��! ��λ: rad
  float speed;  // ֻ��! ��λ: rad/s

  void request();
  void update(uint16_t size);

  /**
     * @brief ����ǰλ������
     */
  void resetCurPosToZero();

  /**
     * @brief �����ת����
     */
  void resetClogProtection();

  /**
     * @brief ��ȡϵͳ����
     * @param param �������ͣ�SysParams_t ö�٣�
     */
  void readSysParams(SysParams_t param);

  /**
     * @brief ���ÿ���ģʽ������/�ջ���
     * @param save �Ƿ񱣴������Flash
     * @param mode ����ģʽ��
     *             0���ر���������
     *             1������ģʽ
     *             2���ջ�ģʽ
     *             3��En������λ��Dir���õ�λ���
     */
  void setCtrlMode(bool save, uint8_t mode);

  /**
     * @brief ���Ƶ��ʹ��״̬
     * @param state ʹ��״̬��true��ʹ�ܣ�false���رգ�
     * @param sync �Ƿ����ö��ͬ����Ĭ��false��
     */
  void enableMotor(bool state, bool sync = false);

  /**
     * @brief �ٶ�ģʽ����
     * @param dir ����0��CW��������CCW��
     * @param vel �ٶȣ�0~5000 RPM��
     * @param acc ���ٶȣ�0~255��0��ʾֱ��������
     * @param sync �Ƿ����ö��ͬ����Ĭ��false��
     */
  void setVelocity(uint8_t dir, uint16_t vel, uint8_t acc, bool sync = false);

  /**
     * @brief λ��ģʽ����
     * @param dir ����0��CW��������CCW��
     * @param vel �ٶȣ�0~5000 RPM��
     * @param acc ���ٶȣ�0~255��0��ʾֱ��������
     * @param pulses ��������0~2^32-1��
     * @param absolute �Ƿ�Ϊ����λ��ģʽ��Ĭ��false�����λ�ã�
     * @param sync �Ƿ����ö��ͬ����Ĭ��false��
     */
  void setPosition(
    uint8_t dir, uint16_t vel, uint8_t acc, uint32_t pulses, bool absolute = false,
    bool sync = false);

  /**
     * @brief ����ֹͣ���
     * @param sync �Ƿ����ö��ͬ����Ĭ��false��
     */
  void stopNow(bool sync = false);

  /**
     * @brief �������ͬ���˶�
     */
  void triggerSyncMotion();

  /**
     * @brief ���õ�ǰλ��Ϊ��Ȧ�������
     * @param save �Ƿ񱣴浽Flash��Ĭ��false��
     */
  void setOrigin(bool save = false);

  /**
     * @brief �޸Ļ������
     * @param save �Ƿ񱣴浽Flash
     * @param mode ����ģʽ��
     *             0����Ȧ�ͽ�����
     *             1����Ȧ�������
     *             2����Ȧ����λ��ײ����
     *             3����Ȧ����λ���ػ���
     * @param dir ���㷽��0��CW��������CCW��
     * @param vel �����ٶȣ�RPM��
     * @param timeout ���㳬ʱʱ�䣨ms��
     * @param detectVel ��ײ����ٶȣ�RPM��
     * @param detectCurrent ��ײ��������mA��
     * @param detectTime ��ײ���ʱ�䣨ms��
     * @param powerOnAuto �Ƿ��ϵ��Զ����㣨Ĭ��false��
     */
  void setOriginParams(
    bool save, uint8_t mode, uint8_t dir, uint16_t vel, uint32_t timeout, uint16_t detectVel,
    uint16_t detectCurrent, uint16_t detectTime, bool powerOnAuto = false);

  /**
     * @brief �������㶯��
     * @param mode ����ģʽ��ͬ�ϣ�
     * @param sync �Ƿ����ö��ͬ����Ĭ��false��
     */
  void triggerOrigin(uint8_t mode, bool sync = false);

  /**
     * @brief ǿ���жϲ��˳�����
     */
  void interruptOrigin();

private:
  const bool use_dma_;
  const uint8_t addr_;  // �����ַ

  uint8_t buff_[BUFF_SIZE];
};
}  // namespace crane

#endif  // ZDT_MOTOR_HPP