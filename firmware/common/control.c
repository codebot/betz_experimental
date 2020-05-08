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
#include <math.h>

#include "adc.h"
#include "control.h"
#include "enc.h"
#include "param.h"
#include "pwm.h"
#include "systime.h"
#include "state.h"
#include "status_led.h"

#ifndef EMULATOR
#include "pin.h"
#include "stm32f405xx.h"
#endif

static float g_control_position_target = 0;
static float g_control_voltage_target = 0;
static float g_control_pole_count = 21;
static int g_control_mode = 0;

void control_init()
{
  param_int(
      "control_mode",
      &g_control_mode,
      CONTROL_MODE_VOLTAGE,
      PARAM_TRANSIENT);

  param_float(
      "position_target",
      &g_control_position_target,
      0,
      PARAM_TRANSIENT);

  param_float(
      "voltage_target",
      &g_control_voltage_target,
      0,
      PARAM_TRANSIENT);

  param_float(
      "pole_count",
      &g_control_pole_count,
      12,
      PARAM_PERSISTENT);

#ifndef EMULATOR
  pin_set_output(GPIOA, 3, 0);
#endif
}

void control_tick()
{
}

#ifndef EMULATOR
// strangely, ramfunc seems to result in slightly higher jitter and no
// real improvement in speed (?)
// void tim1_up_tim10_vector() __attribute__((section(".ramfunc")));

void tim1_up_tim10_vector()
{
  TIM1->SR &= ~TIM_SR_UIF;  // clear the interrupt flag that got us here
  if (TIM1->CR1 & TIM_CR1_DIR)
  {
    // all shunts are ready to measure. trigger the ADC's.
    adc_start_nonblocking_read();

    pin_set_output_high(GPIOA, 3);

    g_state.t = systime_read();


    // todo: other stuff...

    // now spinlock until ADC is complete
    while (!g_adc_read_complete) { }

    g_state.raw_adc[0] = (uint16_t)ADC1->DR;
    g_state.raw_adc[1] = (uint16_t)ADC2->DR;
    g_state.raw_adc[2] = (uint16_t)ADC3->DR;

    for (int i = 0; i < 3; i++)
      g_state.phase_currents[i] =
          ((float)(g_state.raw_adc[i]) - 2048.0f) / 2047.0f * 1.65f * 5.0f;

    // do more control stuff

    float v_a = 0;
    float v_b = 0;
    float v_c = 0;

    if (g_control_mode == CONTROL_MODE_POSITION)
    {
    }

    if (g_control_mode == CONTROL_MODE_VOLTAGE ||
        g_control_mode == CONTROL_MODE_POSITION)
    {
      const float deg120 = (float)(M_PI * 2.0f / 3.0f);
      const float elec_angle =
          fmodf(g_state.enc * g_control_pole_count, 2.0f * M_PI);
      v_a = g_control_voltage_target * sinf(elec_angle);
      v_b = g_control_voltage_target * sinf(elec_angle + deg120);
      v_c = g_control_voltage_target * sinf(elec_angle + 2.0f * deg120);
    }

    const float PWM_MID = PWM_MAX / 2.0f;
    if (g_control_mode == CONTROL_MODE_IDLE)
    {
      pwm_set(PWM_MID, PWM_MID, PWM_MID);
    }
    else
    {
      const float BUS_VOLTAGE = 24.0f;
      int32_t pwm_a = (int32_t)(v_a * PWM_MID / BUS_VOLTAGE + PWM_MID);
      int32_t pwm_b = (int32_t)(v_b * PWM_MID / BUS_VOLTAGE + PWM_MID);
      int32_t pwm_c = (int32_t)(v_c * PWM_MID / BUS_VOLTAGE + PWM_MID);

      // clamp to sane values
      const int32_t PWM_TOO_LOW = 100;  // preserve "quiet time" at top/bottom
      const int32_t PWM_TOO_HIGH = PWM_MAX - PWM_TOO_LOW;

      pwm_a = pwm_a < PWM_TOO_LOW ? PWM_TOO_LOW : pwm_a;
      pwm_b = pwm_b < PWM_TOO_LOW ? PWM_TOO_LOW : pwm_b;
      pwm_c = pwm_c < PWM_TOO_LOW ? PWM_TOO_LOW : pwm_c;

      pwm_a = pwm_a >= PWM_TOO_HIGH ? PWM_TOO_HIGH : pwm_a;
      pwm_b = pwm_b >= PWM_TOO_HIGH ? PWM_TOO_HIGH : pwm_b;
      pwm_c = pwm_c >= PWM_TOO_HIGH ? PWM_TOO_HIGH : pwm_c;
      
      pwm_set(pwm_a, pwm_b, pwm_c);
    }

    pin_set_output_low(GPIOA, 3);
  }
  else
  {
    // We're at the top of the PWM cycle. Time to talk to the encoder.
    enc_start_nonblocking_read_to_state();
  }
}
#endif
