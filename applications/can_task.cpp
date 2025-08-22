#include <cstring>

#include "cmsis_os.h"
#include "host_protocol.hpp"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_para.hpp"
#include "servo_link.hpp"

// --- 全局对象和变量 ---
sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);

extern sp::RM_Motor motoryl, motoryr, motorx, motorzf, motorzr;
extern move_t move_data;
extern coordinate_t coordinate_data;
extern ServoTarget g_servo_target[2];  // 舵机“邮箱”

// --- PC通信相关定义 ---
constexpr uint32_t CAN_ID_FROM_PC = 0xA1;
constexpr uint32_t CAN_ID_TO_PC_HIGH_PRIORITY = 0x001;

// 用于中断和主循环安全通信的标志位和缓冲区
volatile bool pc_reply_pending = false;
volatile uint8_t pc_received_data_buffer[8];

// 小工具：LE -> float
static inline float le_f32(const uint8_t * p)
{
  float v;
  std::memcpy(&v, p, 4);
  return v;
}

// ===== 前向声明（重要：在 can_task() 调用之前声明）=====
void send_motors_1_to_4();
void send_motor_5();
void send_pc_reply();

// 解析来自上位机的 CAN 指令（8B，标准帧）
static inline void handle_host_can(uint32_t id, const uint8_t * d)
{
  if (id == host::ID_BLDCPOS) {
    const uint8_t axis = d[0];
    const uint8_t mode = d[1];
    const float val = le_f32(&d[4]);  // 位置（米）
    if (mode == 0) {
      switch (axis) {
        case host::AX_X:
          coordinate_data.x = val;
          break;
        case host::AX_Y:
          coordinate_data.y = val;
          break;
        case host::AX_ZF:
          coordinate_data.zf = val;
          break;
        case host::AX_ZR:
          coordinate_data.zr = val;
          break;
        default:
          break;
      }
    }
    return;
  }

  if (id == host::ID_SERVO_POS) {
    const uint8_t sv = d[0];  // 0=Z1, 1=Z2
    const uint16_t spd10 = (uint16_t)d[1] | ((uint16_t)d[2] << 8);
    const uint16_t acc10 = (uint16_t)d[3] | ((uint16_t)d[4] << 8);
    const uint16_t ang100 = (uint16_t)d[5] | ((uint16_t)d[6] << 8);
    if (sv < 2) {
      g_servo_target[sv].angle_deg = (float)ang100 / 100.0f;
      g_servo_target[sv].speed_dps = (spd10 == 0) ? 0.0f : (float)spd10 / 10.0f;
      g_servo_target[sv].acc_dps2 = (acc10 == 0) ? 0.0f : (float)acc10 / 10.0f;
      g_servo_target[sv].pending = true;  // motor_task 1ms 内执行
    }
    return;
  }
}

/**
 * @brief CAN通信主任务：只负责按计划发送数据
 */
extern "C" void can_task()
{
  osDelay(200);

  can1.config();
  can1.start();
  can2.config();
  can2.start();

  // 调度器计数器，用于交替发送
  static uint32_t send_scheduler_step = 0;

  while (1) {
    switch (send_scheduler_step) {
      case 0:  // 发送 1~4 号
        send_motors_1_to_4();
        break;
      case 1:  // 发送 5 号
        send_motor_5();
        break;
      case 2:  // 向 PC 回声（可选）
        send_pc_reply();
        break;
    }
    send_scheduler_step = (send_scheduler_step + 1) % 3;
    osDelay(3);  // 任务周期（约 3ms），总调度周期 ~9ms
  }
}

/**
 * @brief CAN接收中断回调函数：接收+分发
 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  if (hcan == &hcan1) {
    can1.recv();

    // 先尝试解析来自上位机的控制帧（不会阻塞）
    handle_host_can(can1.rx_id, can1.rx_data);

    if (can1.rx_id == CAN_ID_FROM_PC) {
      // PC 回声示例（保留）
      if (!pc_reply_pending) {
        for (int i = 0; i < 8; ++i) pc_received_data_buffer[i] = can1.rx_data[i];
        pc_reply_pending = true;
      }
    }
    // 处理电机反馈
    else if (can1.rx_id == motoryl.rx_id)
      motoryl.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == motoryr.rx_id)
      motoryr.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == motorx.rx_id)
      motorx.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == motorzf.rx_id)
      motorzf.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == motorzr.rx_id)
      motorzr.read(can1.rx_data, stamp_ms);

    return;
  }

  if (hcan == &hcan2) {
    can2.recv();
    // 如需从 CAN2 也接收主机指令，可加：
    // handle_host_can(can2.rx_id, can2.rx_data);
    return;
  }
}

/**
 * @brief 发送电机组1 (ID 1-4) 的指令
 */
void send_motors_1_to_4()
{
  motoryl.cmd(move_data.set_motor_torque[0]);
  motoryr.cmd(move_data.set_motor_torque[1]);
  motorx.cmd(move_data.set_motor_torque[2]);
  motorzf.cmd(move_data.set_motor_torque[3]);

  motoryl.write(can1.tx_data);
  motoryr.write(can1.tx_data);
  motorx.write(can1.tx_data);
  motorzf.write(can1.tx_data);

  can1.send(motorx.tx_id);  // 发送 0x200 帧
}

/**
 * @brief 发送电机组2 (ID 5) 的指令
 */
void send_motor_5()
{
  motorzr.cmd(move_data.set_motor_torque[4]);
  motorzr.write(can1.tx_data);
  can1.send(motorzr.tx_id);  // 发送 0x1FF 帧
}

/**
 * @brief 检查并发送给PC的回复（示例）
 */
void send_pc_reply()
{
  if (pc_reply_pending) {
    for (int i = 0; i < 8; ++i) can1.tx_data[i] = pc_received_data_buffer[i];
    can1.tx_data[0]++;  // 回声计数 +1
    can1.send(CAN_ID_TO_PC_HIGH_PRIORITY);
    pc_reply_pending = false;
  }
}
