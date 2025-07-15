#include "cmsis_os.h"
#include "controllers/control_mode.hpp"
#include "controllers/controllers.hpp"
#include "controllers/pids.hpp"
#include "hardwares/chassis.hpp"
#include "hardwares/uart/uart.hpp"

// uint8_t cmd[6];

extern "C" void control_task()
{
  auto enable = false;
  auto has_enabled = false;
  auto last_mode = ControlMode::DISABLE;

  osDelay(2000);
  chassis::motor_lf.request();
  chassis::motor_lr.request();
  chassis::motor_rf.request();
  chassis::motor_rr.request();

  ops_9.request();
  mecanum_chassis.enable();
  while (true) {
    // enable = ((!has_enabled && arm_calibrated) || (last_mode != vt03.mode));
    // enable = true;
    // last_mode = vt03.mode;
    // last_mode = ControlMode::VELOCITY;

    // if (enable) {
    // mecanum_chassis.enable();
    //   has_enabled = true;
    //   continue;
    // }
    // mecanum_chassis.enable();
    // osDelay(1000);
    mecanum_chassis.control();
    // static uint8_t cmd[] = {0x02, 0xF6, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x6B};
    // HAL_UART_Transmit_DMA(&huart1, cmd, 8);

    // osDelay(CONTROL_T_MS);

    // // cmd[0] = 0x01;
    // // cmd[1] = 0x02;
    // // cmd[2] = 0x03;
    // // cmd[3] = 0x04;
    // // cmd[4] = 0x05;
    // // cmd[5] = 0x06;
    // const uint8_t cmd[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

    // // HAL_UART_Transmit_DMA(&huart1, cmd, 6);
    // HAL_UART_Transmit(&huart1, (uint8_t *)cmd, 6, 100);
    osDelay(1);
  }

  return;
}

// #include "hardwares/uart/uart_recv.hpp"
// extern "C" void control_task()
// {
//   osDelay(2000);

//   const uint8_t cmd[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

//   uart_recv_start();

//   while (true) {
//     HAL_UART_Transmit(&huart1, (uint8_t *)cmd, 6, 100);  // 发数据给上位机
//     osDelay(1000);                                       // 1 秒发一次
//   }

//   return;
// }
