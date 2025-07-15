#include <iostream>

#include "cmsis_os.h"
#include "hardwares/chassis.hpp"
#include "uart.hpp"

#define RX_BUF_LEN 64
uint8_t rx_buf[RX_BUF_LEN];
uint16_t rx_size = 0;

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == chassis::motor_lf.huart) {
    chassis::motor_lf.update(Size);
    chassis::motor_lf.request();
    chassis::motor_lr.update(Size);
    chassis::motor_lr.request();

    chassis::motor_rf.update(Size);
    chassis::motor_rf.request();
    chassis::motor_rr.update(Size);
    chassis::motor_rr.request();
  }

  if (huart == ops_9.huart) {
    ops_9.update(ops_9.rx_buf_, Size);
    ops_9.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == chassis::motor_lf.huart) {
    chassis::motor_lf.request();
  }

  if (huart == ops_9.huart) {
    ops_9.request();
  }
}
