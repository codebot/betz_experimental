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

#include "adc.h"
#include "pin.h"
#include "state.h"
#include "status_led.h"

#include "soc.h"

volatile bool g_adc_read_complete = false;

void adc_init()
{
#if defined(BOARD_blue)

  // PC0 = ISENSE_C = ADC123_IN10
  // PC1 = ISENSE_B = ADC123_IN11
  // PC2 = ISENSE_A = ADC123_IN12
  // PC3 = VSENSE_VIN = ADC123_IN13  (vbus through 49.9k / 2.21k divider)

  // turn on ADC clock trees
  RCC->APB2ENR |=
      RCC_APB2ENR_ADC1EN |
      RCC_APB2ENR_ADC2EN |
      RCC_APB2ENR_ADC3EN ;

  // wake the ADC's from sleep
  ADC1->CR2 |= ADC_CR2_ADON;
  ADC2->CR2 |= ADC_CR2_ADON;
  ADC3->CR2 |= ADC_CR2_ADON;

  // set the common control register
  ADC->CCR =
      ADC_CCR_TSVREFE  |  // enable internal VREF
      ADC_CCR_ADCPRE_1 |  // apb2 is 84 MHz. we'll set ADC = 84/6 = 14 MHz
      ADC_CCR_MULTI_4  |  // set the MULTI bits for "regular simultaneous mode"
      ADC_CCR_MULTI_2  |
      ADC_CCR_MULTI_1  ;

  // set the single-element conversion "sequences" for the ADC's
  ADC1->SQR3 = 10;
  ADC2->SQR3 = 11;
  ADC3->SQR3 = 12;

  // could play more games here and have it also digitize VIN someday

  pin_set_analog(GPIOC, 0);
  pin_set_analog(GPIOC, 1);
  pin_set_analog(GPIOC, 2);
  pin_set_analog(GPIOC, 3);

  ADC1->CR1 |= ADC_CR1_EOCIE;  // enable end-of-conversion interrupt
  NVIC_SetPriority(ADC_IRQn, 1);  // must be higher priority than PWM
  NVIC_EnableIRQ(ADC_IRQn);
#elif defined(BOARD_mini)

  // PA0 = IC = ADC1_IN1
  // PA6 = IB = ADC2_IN4
  // PB1 = IA = ADC3_IN1

  RCC->AHB2ENR |=
      RCC_AHB2ENR_ADC12EN  |
      RCC_AHB2ENR_ADC345EN ;

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;  // powers up VREFBUF

  VREFBUF->CSR =
      VREFBUF_CSR_VRS_1 |  // set voltage reference to 2.9 v
      VREFBUF_CSR_ENVR  ;  // enable the voltage reference

  // TODO: clear ADC_CR DEEPPWD bit
  // TODO: ADC_CR ADVREGEN=1 
  // wait a bit (T_ADCVREG_STUP)
  // start calibration sequence by setting ADCALDIF=0 and ADCAL=1
  // set ADEN=1
  // wait for ADCAL=0
  // (cal factor is ADC_CALFACT)

  // PLL input P is selected because we're leaving CLKSEL = b00
  ADC12_COMMON->CCR =
      ADC_CCR_DUAL_2 |  // set DUAL bits for "regular simultaneous mode"
      ADC_CCR_DUAL_1 ;

  ADC345_COMMON->CCR = 0;  // set DUAL bits to zero for independent mode

#endif
}

void adc_start_nonblocking_read()
{
#if defined(BOARD_blue)
  ADC1->CR2 |= ADC_CR2_SWSTART;
  g_adc_read_complete = false;
#elif defined(BOARD_mini)
  g_adc_read_complete = true;
#endif
}

void adc_vector()
{
#if defined(BOARD_blue)
  // clear the flag that got us here
  ADC1->SR &= ~ADC_SR_EOC;
  g_adc_read_complete = true;
#elif defined(BOARD_mini)
#endif
}

void adc_blocking_read()
{
#if defined(BOARD_blue)
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while (!(ADC1->SR & ADC_SR_EOC)) { }  // spin until it's done
  g_state.raw_adc[0] = ADC1->DR;
  g_state.raw_adc[1] = ADC2->DR;
  g_state.raw_adc[2] = ADC3->DR;
#elif defined(BOARD_mini)
#endif
}
