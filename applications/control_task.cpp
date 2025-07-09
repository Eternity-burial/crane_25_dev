#include "cmsis_os.h"
#include "controllers/control_mode.hpp"
#include "controllers/controllers.hpp"
#include "controllers/pids.hpp"
#include "hardwares/chassis.hpp"

extern "C" void control_task()
{
  chassis::motor_lf.request();

  auto enable = false;
  auto has_enabled = false;
  auto last_mode = ControlMode::DISABLE;

  while (true) {
    // enable = ((!has_enabled && arm_calibrated) || (last_mode != vt03.mode));
    enable = true;
    // last_mode = vt03.mode;
    last_mode = ControlMode::VELOCITY;

    if (enable) {
      mecanum_chassis.enable();
      has_enabled = true;
      continue;
    }

    mecanum_chassis.control();

    osDelay(CONTROL_T_MS);
  }

  return;
}
