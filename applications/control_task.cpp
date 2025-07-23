#include "cmsis_os.h"
#include "controllers/control_mode.hpp"
#include "controllers/controllers.hpp"
#include "controllers/pids.hpp"
#include "hardwares/chassis.hpp"
#include "hardwares/uart/uart.hpp"

uint8_t chassis::huart1_rx_buff[chassis::BUFF_SIZE];
extern "C" void control_task()
{
  auto enable = false;
  auto has_enabled = false;
  auto last_mode = ControlMode::DISABLE;

  osDelay(2000);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, chassis::huart1_rx_buff, chassis::BUFF_SIZE);

  ops_9.request();
  mecanum_chassis.enable();

  while (true) {
    mecanum_chassis.control();

    osDelay(1);
  }

  return;
}
