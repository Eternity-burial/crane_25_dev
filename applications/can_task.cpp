#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_para.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);

extern sp::RM_Motor motoryl;
extern sp::RM_Motor motoryr;

extern move_t move_data;
void send_motory();

extern "C" void can_task()
{
  // 启动延时，等系统稳定
  osDelay(200);
  can1.config();
  can1.start();

  can2.config();
  can2.start();
  while (1) {
    send_motory();
    osDelay(1);  // 约 1 ms ⇒ 循环速率 ~1000 Hz
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  if (hcan == &hcan1) {
    can1.recv();
    if (can1.rx_id == motoryl.rx_id)
      motoryl.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == motoryr.rx_id)
      motoryr.read(can1.rx_data, stamp_ms);
    return;
  }

  if (hcan == &hcan2) {
    can2.recv();
    return;
  }
}

void send_motory()
{
  motoryl.cmd(move_data.set_motor_torque[0]);
  motoryl.write(can1.tx_data);
  motoryr.cmd(move_data.set_motor_torque[1]);
  motoryr.write(can1.tx_data);

  can1.send(motoryl.tx_id);
}