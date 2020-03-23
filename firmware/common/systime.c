#include "systime.h"
#include "stm32f405xx.h"

void systime_init()
{
  // use TIM2 since it's a 32-bit counter. just have it count
  // microseconds since powerup.
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  // APB1 speed is sysclock/4 = 42 MHz
  // surprisingly, the TIM2 clock frequency is actually 2x APB1 = 84 MHz
  TIM2->PSC = 42000000 * 2 / 1000000 - 1;  // comes out to 83
  TIM2->ARR = 0xffffffff; // count as long as possible
  TIM2->EGR = TIM_EGR_UG; // load the PSC register immediately
  TIM2->CR1 = TIM_CR1_CEN; // start counter
}

