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

#include <stdio.h>

#include "control.h"
#include "pin.h"
#include "pwm.h"
#include "soc.h"

void pwm_init()
{
  printf("pwm_init()\r\n");
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  TIM1->PSC = 0;  // no prescalar
  TIM1->ARR = PWM_MAX;
  TIM1->CCR1 = PWM_MAX / 2;
  TIM1->CCR2 = PWM_MAX / 2;
  TIM1->CCR3 = PWM_MAX / 2;

  TIM1->CCMR1 =
      TIM_CCMR1_OC1M_1 |  // PWM mode on output compare 1 (OC1)
      TIM_CCMR1_OC1M_2 |  // PWM mode on output compare 1 (OC1)
      TIM_CCMR1_OC1CE  |  // clear output 1 on compare match
      TIM_CCMR1_OC2M_1 |  // PWM mode on output compare 2 (OC2)
      TIM_CCMR1_OC2M_2 |  // PWM mode on output compare 2 (OC2)
      TIM_CCMR1_OC2CE  ;  // clear output 2 on compare match

  TIM1->CCMR2 =
      TIM_CCMR2_OC3M_1 |  // PWM mode on output compare 3 (OC3)
      TIM_CCMR2_OC3M_2 |  // PWM mode on output compare 3 (OC3)
      TIM_CCMR2_OC3CE  ;  // clear output 3 on compare match

  TIM1->CCER =
      TIM_CCER_CC1E |  // enable compare output 1
      TIM_CCER_CC2E |  // enable compare output 2
      TIM_CCER_CC3E ;  // enable compare output 3

#if defined(BOARD_blue)

  // PWM_FAULT = PC6 (open-drain)
  // PWM_OCTW = PB15 (open-drain)

  // PWM_RSTA = PC7
  // PWM_RSTB = PC8
  // PWM_RSTC = PC9

  // PWM_PWMA = PA10  TIM1_CH3 via AF1
  // PWM_PWMB = PA9   TIM1_CH2 via AF1
  // PWM_PWMC = PA8   TIM1_CH1 via AF1

  // drive NRESET pins low to disable the drivers
  pin_set_output(GPIOC, 7, 0);
  pin_set_output(GPIOC, 8, 0);
  pin_set_output(GPIOC, 9, 0);

  pin_set_alternate_function(GPIOA, 8, 1);
  pin_set_alternate_function(GPIOA, 9, 1);
  pin_set_alternate_function(GPIOA, 10, 1);

#elif defined(BOARD_mini)

  // PWM_SLEEP = PC6
  // PWM_FAULT = PA12  (open-drain)

  // PWM_HSA = PA10, AF6 = TIM1_CH3
  // PWM_HSB = PA9, AF6 = TIM1_CH2
  // PWM_HSC = PA8, AF6 = TIM1_CH1

  // PWM_LSA = PB9, AF12 = TIM1_CH3N
  // PWM_LSB = PB0, AF6 = TIM1_CH2N
  // PWM_LSC = PA11, AF6 = TIM1_CH1N

  pin_set_output(GPIOC, 9, 0);  // assert SLEEP

  pin_set_alternate_function(GPIOA, 10, 6);
  pin_set_alternate_function(GPIOA, 9, 6);
  pin_set_alternate_function(GPIOA, 8, 6);

  pin_set_alternate_function(GPIOB, 9, 12);
  pin_set_alternate_function(GPIOB, 0, 6);
  pin_set_alternate_function(GPIOA, 11, 6);

  TIM1->CCER |=
      TIM_CCER_CC1NE |  // enable compare output 1 complementary output
      TIM_CCER_CC2NE |  // enable compare output 2 complementary output
      TIM_CCER_CC3NE ;  // enable compare output 3 complementary output

#endif

  TIM1->EGR = TIM_EGR_UG;  // generate update event to load registers
  TIM1->DIER |= TIM_DIER_UIE;  // fire interrupt at top and bottom

  TIM1->CR1 =
      TIM_CR1_CMS_0 |  // enable center-aligned PWM mode
      TIM_CR1_ARPE  |  // enable auto-reload preload (buffered)
      TIM_CR1_CEN   ;  // enable counter

  TIM1->BDTR = TIM_BDTR_MOE;  // master output enable

#if defined(BOARD_blue)
  NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2); // lower than comms and ADC
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
#elif defined(BOARD_mini)
  NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 2); // lower than comms and ADC
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
#endif
}

void pwm_set(const uint32_t a, const uint32_t b, const uint32_t c)
{
  TIM1->CCR1 = a;
  TIM1->CCR2 = b;
  TIM1->CCR3 = c;
}

uint32_t pwm_max()
{
  // in the future, this could become a parameter, I suppose.
  return PWM_MAX;
}

void pwm_enable(const bool enable)
{
#if defined(BOARD_blue)
  if (enable)
  {
    // drive NRESET pins high to enable the drivers
    pin_set_output(GPIOC, 7, 1);
    pin_set_output(GPIOC, 8, 1);
    pin_set_output(GPIOC, 9, 1);
  }
  else
  {
    // drive NRESET pins low to disable the drivers
    pin_set_output(GPIOC, 7, 0);
    pin_set_output(GPIOC, 8, 0);
    pin_set_output(GPIOC, 9, 0);
  }
#elif defined(BOARD_mini)
  if (enable)
  {
    pin_set_output(GPIOC, 6, 1);
  }
  else
  {
    pin_set_output(GPIOC, 6, 0);
  }
#endif
}
