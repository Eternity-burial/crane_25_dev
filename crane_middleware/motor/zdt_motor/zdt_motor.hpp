#ifndef ZDT_MOTOR_HPP
#define ZDT_MOTOR_HPP

#include <array>
#include <cstdint>

#include "usart.h"

namespace crane
{  // 系统参数类型枚举，用于读取不同的系统参数
constexpr size_t BUFF_SIZE = 255;
typedef enum
{
  S_VER,   // 版本号
  S_RL,    // 运行状态
  S_PID,   // PID参数
  S_VBUS,  // 总线电压
  S_CPHA,  // 电流相位
  S_ENCL,  // 编码器值
  S_TPOS,  // 目标位置
  S_VEL,   // 当前速度
  S_CPOS,  // 当前位置
  S_PERR,  // 位置误差
  S_FLAG,  // 状态标志
  S_ORG,   // 原点位置
  S_Conf,  // 配置参数
  S_State  // 系统状态
} SysParams_t;

class ZDT_Motor
{
public:
  /**
     * @brief 构造函数
     * @param addr 电机地址（1~255）
     */
  ZDT_Motor(UART_HandleTypeDef * huart, uint8_t addr, bool use_dma = true);
  UART_HandleTypeDef * huart;

  float angle;  // 只读! 单位: rad
  float speed;  // 只读! 单位: rad/s

  void request();
  void update(uint16_t size);

  /**
     * @brief 将当前位置清零
     */
  void resetCurPosToZero();

  /**
     * @brief 解除堵转保护
     */
  void resetClogProtection();

  /**
     * @brief 读取系统参数
     * @param param 参数类型（SysParams_t 枚举）
     */
  void readSysParams(SysParams_t param);

  /**
     * @brief 设置控制模式（开环/闭环）
     * @param save 是否保存参数到Flash
     * @param mode 控制模式：
     *             0：关闭脉冲输入
     *             1：开环模式
     *             2：闭环模式
     *             3：En复用限位，Dir复用到位输出
     */
  void setCtrlMode(bool save, uint8_t mode);

  /**
     * @brief 控制电机使能状态
     * @param state 使能状态（true：使能，false：关闭）
     * @param sync 是否启用多机同步（默认false）
     */
  void enableMotor(bool state, bool sync = false);

  /**
     * @brief 速度模式控制
     * @param dir 方向（0：CW，其他：CCW）
     * @param vel 速度（0~5000 RPM）
     * @param acc 加速度（0~255，0表示直接启动）
     * @param sync 是否启用多机同步（默认false）
     */
  void setVelocity(uint8_t dir, uint16_t vel, uint8_t acc, bool sync = false);

  /**
     * @brief 位置模式控制
     * @param dir 方向（0：CW，其他：CCW）
     * @param vel 速度（0~5000 RPM）
     * @param acc 加速度（0~255，0表示直接启动）
     * @param pulses 脉冲数（0~2^32-1）
     * @param absolute 是否为绝对位置模式（默认false：相对位置）
     * @param sync 是否启用多机同步（默认false）
     */
  void setPosition(
    uint8_t dir, uint16_t vel, uint8_t acc, uint32_t pulses, bool absolute = false,
    bool sync = false);

  /**
     * @brief 立即停止电机
     * @param sync 是否启用多机同步（默认false）
     */
  void stopNow(bool sync = false);

  /**
     * @brief 触发多机同步运动
     */
  void triggerSyncMotion();

  /**
     * @brief 设置当前位置为单圈回零零点
     * @param save 是否保存到Flash（默认false）
     */
  void setOrigin(bool save = false);

  /**
     * @brief 修改回零参数
     * @param save 是否保存到Flash
     * @param mode 回零模式：
     *             0：单圈就近回零
     *             1：单圈方向回零
     *             2：多圈无限位碰撞回零
     *             3：多圈有限位开关回零
     * @param dir 回零方向（0：CW，其他：CCW）
     * @param vel 回零速度（RPM）
     * @param timeout 回零超时时间（ms）
     * @param detectVel 碰撞检测速度（RPM）
     * @param detectCurrent 碰撞检测电流（mA）
     * @param detectTime 碰撞检测时间（ms）
     * @param powerOnAuto 是否上电自动回零（默认false）
     */
  void setOriginParams(
    bool save, uint8_t mode, uint8_t dir, uint16_t vel, uint32_t timeout, uint16_t detectVel,
    uint16_t detectCurrent, uint16_t detectTime, bool powerOnAuto = false);

  /**
     * @brief 触发回零动作
     * @param mode 回零模式（同上）
     * @param sync 是否启用多机同步（默认false）
     */
  void triggerOrigin(uint8_t mode, bool sync = false);

  /**
     * @brief 强制中断并退出回零
     */
  void interruptOrigin();

private:
  const bool use_dma_;
  const uint8_t addr_;  // 电机地址

  uint8_t buff_[BUFF_SIZE];
};
}  // namespace crane

#endif  // ZDT_MOTOR_HPP