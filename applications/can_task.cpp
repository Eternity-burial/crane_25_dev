#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_para.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);

extern sp::RM_Motor motor1;
extern sp::RM_Motor motor2;

extern move_t move_data;
void send_motor_1to2();

extern "C" void can_task()
{
  // 启动延时，等系统稳定
  osDelay(2000);
  can1.config();
  can2.config();
  while (1) {
    send_motor_1to2();
    osDelay(1);  // 约 1 ms ⇒ 循环速率 ~1000 Hz
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  if (hcan == &hcan1) {
    can1.recv();
    if (can1.rx_id == motor1.rx_id)
      motor1.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == motor2.rx_id)
      motor2.read(can1.rx_data, stamp_ms);
    return;
  }

  if (hcan == &hcan2) {
    can2.recv();
    return;
  }
}

void send_motor_1to2()
{
  motor1.cmd(move_data.set_motor_torque[0]);
  motor1.write(can1.tx_data);
  motor2.cmd(move_data.set_motor_torque[1]);
  motor2.write(can1.tx_data);
}