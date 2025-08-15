#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

sp::CAN can_1(&hcan1);
sp::CAN can_2(&hcan2);
extern "C" void can_task()
{
  // 启动延时，等系统稳定
  osDelay(2000);

  osDelay(1); // 约 1 ms ⇒ 循环速率 ~1000 Hz
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  auto stamp_ms = osKernelSysTick();

  if (hcan == &hcan1)
  {
    can_1.recv();

    return;
  }

  if (hcan == &hcan2)
  {
    can_2.recv();
    return;
  }
}
