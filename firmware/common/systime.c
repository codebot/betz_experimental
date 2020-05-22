/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "systime.h"
#include "soc.h"

void systime_init()
{
  // use TIM2 since it's a 32-bit counter. just have it count
  // microseconds since powerup.

#if defined(BOARD_blue)

  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  // APB1 speed is sysclock/4 = 42 MHz
  // surprisingly, the TIM2 clock frequency is actually 2x APB1 = 84 MHz
  const uint32_t psc = 42000000 * 2 / 1000000 - 1;  // comes out to 83

#elif defined(BOARD_mini)

  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  // APB1 speed is 168 MHz, TIM2 clock is also 168 MHz
  const uint32_t psc = 168000000 / 1000000 - 1;  // equals 167

#endif

  TIM2->PSC = psc;
  TIM2->ARR = 0xffffffff; // count as long as possible
  TIM2->EGR = TIM_EGR_UG; // load the PSC register immediately
  TIM2->CR1 = TIM_CR1_CEN; // start counter
}
