#ifndef CRANE__LED_HPP
#define CRANE__LED_HPP

#include "gpio.h"
#include "stm32f4xx_hal_gpio.h"
namespace crane
{

class LED
{
public:
  LED(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin);

  void set(bool state);

private:
  GPIO_TypeDef * GPIOx_;
  uint16_t GPIO_Pin_;
};

}  // namespace crane

#endif  // CRANE__LED_HPP