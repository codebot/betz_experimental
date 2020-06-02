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
#include "soc.h"
#endif

#if defined(BOARD_blue)

#define CONTROL_BUSY_GPIO GPIOA
#define CONTROL_BUSY_PIN 3

#elif defined(BOARD_mini)

#define CONTROL_BUSY_GPIO GPIOB
#define CONTROL_BUSY_PIN 3

#endif

static float g_control_position_target = 0;
static float g_control_voltage_target = 0;
static int g_control_pole_count = 21;
static int g_control_mode = 0;
static float g_control_bus_voltage = 24.0;
static float g_control_position_kp = 0;
static float g_control_max_voltage = 0;

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

  param_int(
      "pole_count",
      &g_control_pole_count,
      12,
      PARAM_PERSISTENT);

  param_float(
      "bus_voltage",
      &g_control_bus_voltage,
      24.0,
      PARAM_PERSISTENT);

  param_float(
      "position_kp",
      &g_control_position_kp,
      0.0,
      PARAM_PERSISTENT);

  param_float(
      "max_voltage",
      &g_control_max_voltage,
      1.0,
      PARAM_PERSISTENT);

#ifndef EMULATOR
  pin_set_output(CONTROL_BUSY_GPIO, CONTROL_BUSY_PIN, 0);
#endif
}

void control_tick()
{
}

// strangely, ramfunc seems to result in slightly higher jitter and no
// real improvement in speed (?)
// void tim1_up_tim10_vector() __attribute__((section(".ramfunc")));

#if defined(BOARD_blue)
void tim1_up_tim10_vector()
#elif defined(BOARD_mini)
void tim1_up_tim16_vector()
#elif defined(EMULATOR)
void control_timer()
#endif
{

#ifndef EMULATOR
  TIM1->SR &= ~TIM_SR_UIF;  // clear the interrupt flag that got us here
  if (TIM1->CR1 & TIM_CR1_DIR)
#else
  static int emulator_toggle_idx = 0;
  if (emulator_toggle_idx++ % 2 == 0)
#endif
  {
    // all shunts are ready to measure. trigger the ADC's.
    adc_start_nonblocking_read();

#ifndef EMULATOR
    pin_set_output_high(CONTROL_BUSY_GPIO, CONTROL_BUSY_PIN);
#endif

    g_state.t = systime_read();

    // busy-wait until ADC is complete
    while (!g_adc_read_complete) { }

#if 0
    g_state.raw_adc[0] = (uint16_t)ADC1->DR;
    g_state.raw_adc[1] = (uint16_t)ADC2->DR;
    g_state.raw_adc[2] = (uint16_t)ADC3->DR;

    for (int i = 0; i < 3; i++)
      g_state.phase_currents[i] =
          ((float)(g_state.raw_adc[i]) - 2048.0f) / 2047.0f * 1.65f * 5.0f;
#endif

    // do more control stuff

    float v_a = 0;
    float v_b = 0;
    float v_c = 0;

    if (g_control_mode == CONTROL_MODE_POSITION)
    {
      float pos_error = g_control_position_target - g_state.enc;
      if (pos_error > (float)(M_PI))
        pos_error -= (float)(2.0f * M_PI);
      else if (pos_error < (float)(-M_PI))
        pos_error += (float)(2.0f * M_PI);

      g_control_voltage_target = g_control_position_kp * pos_error;
    }

    // clamp voltage target to the parameterized range
    if (g_control_voltage_target > g_control_max_voltage)
      g_control_voltage_target = g_control_max_voltage;
    else if (g_control_voltage_target < -g_control_max_voltage)
      g_control_voltage_target = -g_control_max_voltage;

    if (g_control_mode == CONTROL_MODE_VOLTAGE ||
        g_control_mode == CONTROL_MODE_POSITION)
    {
      const float deg120 = (float)(M_PI * 2.0f / 3.0f);
      const float elec_angle =
          fmodf(g_state.enc * (float)g_control_pole_count, 2.0f * M_PI);
      v_a = g_control_voltage_target * sinf(elec_angle);
      v_b = g_control_voltage_target * sinf(elec_angle + deg120);
      v_c = g_control_voltage_target * sinf(elec_angle + 2.0f * deg120);

      g_state.phase_currents[0] = v_a;
      g_state.phase_currents[1] = v_b;
      g_state.phase_currents[2] = v_c;
    }

    const float PWM_MID = PWM_MAX / 2.0f;
    if (g_control_mode == CONTROL_MODE_IDLE)
    {
      pwm_set(PWM_MID, PWM_MID, PWM_MID);
    }
    else
    {
      int32_t pwm_a =
          (int32_t)(v_a * PWM_MID / g_control_bus_voltage + PWM_MID);

      int32_t pwm_b =
          (int32_t)(v_b * PWM_MID / g_control_bus_voltage + PWM_MID);

      int32_t pwm_c =
          (int32_t)(v_c * PWM_MID / g_control_bus_voltage + PWM_MID);

      // clamp pwm to sane values
      const int32_t PWM_TOO_LOW = 100;  // preserve "quiet time" at top/bot
      const int32_t PWM_TOO_HIGH = PWM_MAX - PWM_TOO_LOW;

      pwm_a = pwm_a < PWM_TOO_LOW ? PWM_TOO_LOW : pwm_a;
      pwm_b = pwm_b < PWM_TOO_LOW ? PWM_TOO_LOW : pwm_b;
      pwm_c = pwm_c < PWM_TOO_LOW ? PWM_TOO_LOW : pwm_c;

      pwm_a = pwm_a >= PWM_TOO_HIGH ? PWM_TOO_HIGH : pwm_a;
      pwm_b = pwm_b >= PWM_TOO_HIGH ? PWM_TOO_HIGH : pwm_b;
      pwm_c = pwm_c >= PWM_TOO_HIGH ? PWM_TOO_HIGH : pwm_c;

      pwm_set(pwm_a, pwm_b, pwm_c);
    }

#ifndef EMULATOR
    pin_set_output_low(CONTROL_BUSY_GPIO, CONTROL_BUSY_PIN);
#endif
  }
  else
  {
    // We're at the top of the PWM cycle. Time to talk to the encoder.
    enc_start_nonblocking_read_to_state();
  }
}
