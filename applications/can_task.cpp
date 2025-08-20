#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_para.hpp"

// --- 全局对象和变量 ---
sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);

extern sp::RM_Motor motoryl, motoryr, motorx, motorzf, motorzr;
extern move_t move_data;

// --- PC通信相关定义 ---
constexpr uint32_t CAN_ID_FROM_PC = 0xA1;
constexpr uint32_t CAN_ID_TO_PC_HIGH_PRIORITY = 0x001;

// 用于中断和主循环安全通信的标志位和缓冲区
// volatile关键字防止编译器进行不当优化
volatile bool pc_reply_pending = false;
volatile uint8_t pc_received_data_buffer[8];

// --- 函数声明 ---
void send_motors_1_to_4();
void send_motor_5();
void send_pc_reply();

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
    // 使用switch实现简单的分时调度，避免同时发送
    switch (send_scheduler_step) {
      case 0:
        send_motors_1_to_4();
        break;
      case 1:
        send_motor_5();
        break;
      case 2:
        send_pc_reply();  // 尝试发送给PC的回复
        break;
    }

    // 更新调度器，使其在 0, 1, 2 之间循环
    send_scheduler_step = (send_scheduler_step + 1) % 3;

    // 任务周期，例如2ms。总调度周期为 3*2=6ms
    osDelay(3);
  }
}

/**
 * @brief CAN接收中断回调函数：只负责接收和设置标志
 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  if (hcan == &hcan1) {
    can1.recv();

    if (can1.rx_id == CAN_ID_FROM_PC) {
      // 收到PC消息：复制数据，设置标志，然后立即退出中断
      if (!pc_reply_pending) {  // 防止主循环还未处理完时被重复覆盖
        for (int i = 0; i < 8; ++i) {
          pc_received_data_buffer[i] = can1.rx_data[i];
        }
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
    // CAN2 的接收逻辑...

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
 * @brief 检查并发送给PC的回复
 */
void send_pc_reply()
{
  // 只有当中断设置了标志位时才发送
  if (pc_reply_pending) {
    // 准备回复数据
    for (int i = 0; i < 8; ++i) {
      can1.tx_data[i] = pc_received_data_buffer[i];
    }
    can1.tx_data[0]++;  // 回声测试：计数器+1

    // 以高优先级ID=1发送
    can1.send(CAN_ID_TO_PC_HIGH_PRIORITY);

    // 关键一步：清除标志，表示已处理，等待下一次中断请求
    pc_reply_pending = false;
  }
}