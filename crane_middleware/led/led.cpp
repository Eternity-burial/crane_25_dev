#include "led.hpp"

namespace crane
{
LED::LED(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin) : GPIOx_(GPIOx), GPIO_Pin_(GPIO_Pin) {}

void LED::set(bool state)
{
  if (!state) {
    HAL_GPIO_WritePin(GPIOx_, GPIO_Pin_, GPIO_PIN_SET);
  }

  else {
    HAL_GPIO_WritePin(GPIOx_, GPIO_Pin_, GPIO_PIN_RESET);
  }
}

}  // namespace crane