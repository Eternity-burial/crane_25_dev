#include "cmsis_os.h"
#include "controllers/controllers.hpp"
#include "hardwares/uart/uart.hpp"

extern "C" void chassis_task()
{
  while (true) {
    if (true) {
      mecanum_chassis.cmd_v(0, 0, 0);
    }
  }
}
