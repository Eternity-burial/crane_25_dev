#include "cmsis_os.h"
#include "hardwares/chassis.hpp"

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
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == chassis::motor_lf.huart) {
    chassis::motor_lf.request();
  }
}