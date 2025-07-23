#include <cstring>
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
    switch (chassis::huart1_rx_buff[0]) {
      case 0x01:
        std::memcpy(chassis::motor_lf.buff_, chassis::huart1_rx_buff, Size);
        chassis::motor_lf.update(Size);
        break;

      case 0x02:
        std::memcpy(chassis::motor_lr.buff_, chassis::huart1_rx_buff, Size);
        chassis::motor_lr.update(Size);
        break;

      case 0x03:
        std::memcpy(chassis::motor_rf.buff_, chassis::huart1_rx_buff, Size);
        chassis::motor_rf.update(Size);
        break;

      case 0x04:
        std::memcpy(chassis::motor_rr.buff_, chassis::huart1_rx_buff, Size);
        chassis::motor_rr.update(Size);
        break;

      default:
        break;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, chassis::huart1_rx_buff, chassis::BUFF_SIZE);
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
