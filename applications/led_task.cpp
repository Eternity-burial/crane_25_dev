#include "cmsis_os.h"
#include "led/led.hpp"

extern "C" void led_task()
{
  crane::LED red(GPIOF, GPIO_PIN_9);
  crane::LED green(GPIOF, GPIO_PIN_10);

  while (true) {
    red.set(true);
    green.set(false);
    osDelay(200);
    red.set(false);
    // green.set(true);
    osDelay(200);
  }

  return;
}
