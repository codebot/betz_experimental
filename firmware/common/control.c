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
#include "cog.h"
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
static float g_control_velocity_target = 0;
static int g_control_pole_count = 21;
static int g_control_mode = 0;
static float g_control_bus_voltage = 24.0;
static float g_control_position_kp = 0;
static float g_control_position_ki = 0;
static float g_control_position_kd = 0;
static float g_control_velocity_kp = 0;
static float g_control_velocity_kd = 0;
static float g_control_velocity_min = -10.0;
static float g_control_velocity_max = 10.0;
static float g_control_max_voltage = 0;
static float g_control_joint_offset = 0;
static int g_control_joint_dir = 1;
static float g_control_joint_pos_min = -0.2;
static float g_control_joint_pos_max = 0.2;
static float g_control_prev_unfilt_pos = 0.0;
static float g_control_joint_pos_lpf_alpha = 0.5;
static float g_control_joint_vel_lpf_alpha = 0.1;
static bool g_control_init_complete = false;
static float g_control_prev_pos_error = 0;
static float g_control_prev_vel_error = 0;
static float g_control_vel_damp = 0;
static int g_control_joint_wraps = 0;
static float g_control_prev_unwrapped_pos = 0;
static float g_control_integrator = 0;
static float g_control_integrator_max_windup = 0;
static float g_control_integrator_bleed = 1.0f;
static float g_control_cog_scale = 0;

void control_init()
{
  param_int(
      "control_mode",
      &g_control_mode,
      CONTROL_MODE_IDLE,
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
      "joint_vel_target",
      &g_control_velocity_target,
      0,
      PARAM_TRANSIENT);

  param_float(
      "cog_scale",
      &g_control_cog_scale,
      0,
      PARAM_PERSISTENT);

  param_int(
      "wraps",
      &g_control_joint_wraps,
      0,
      PARAM_TRANSIENT);

  param_int(
      "pole_count",
      &g_control_pole_count,
      12,
      PARAM_PERSISTENT);

  param_float(
      "max_windup",
      &g_control_integrator_max_windup,
      0,
      PARAM_PERSISTENT);

  param_float(
      "integrator_bleed",
      &g_control_integrator_bleed,
      1.0,
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
      "position_ki",
      &g_control_position_ki,
      0.0,
      PARAM_PERSISTENT);

  param_float(
      "position_kd",
      &g_control_position_kd,
      0.0,
      PARAM_PERSISTENT);

  param_float(
      "velocity_kp",
      &g_control_velocity_kp,
      0.0,
      PARAM_PERSISTENT);

  param_float(
      "velocity_kd",
      &g_control_velocity_kd,
      0.0,
      PARAM_PERSISTENT);

  param_float(
      "vel_damp",
      &g_control_vel_damp,
      0.0,
      PARAM_PERSISTENT);

  param_float(
      "max_voltage",
      &g_control_max_voltage,
      1.0,
      PARAM_PERSISTENT);

  param_float(
      "joint_offset",
      &g_control_joint_offset,
      0.0,
      PARAM_PERSISTENT);

  param_int(
      "joint_dir",
      &g_control_joint_dir,
      1,
      PARAM_PERSISTENT);

  param_float(
      "velocity_min",
      &g_control_velocity_min,
      -10.0,
      PARAM_PERSISTENT);

  param_float(
      "velocity_max",
      &g_control_velocity_max,
      10.0,
      PARAM_PERSISTENT);

  param_float(
      "joint_pos_min",
      &g_control_joint_pos_min,
      -0.2,
      PARAM_PERSISTENT);

  param_float(
      "joint_pos_max",
      &g_control_joint_pos_max,
      0.2,
      PARAM_PERSISTENT);

  param_float(
      "joint_pos_lpf_alpha",
      &g_control_joint_pos_lpf_alpha,
      0.5,
      PARAM_PERSISTENT);

  param_float(
      "joint_vel_lpf_alpha",
      &g_control_joint_vel_lpf_alpha,
      0.1,
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

    float unfilt_pos = g_state.enc + g_control_joint_offset;
    if (g_control_joint_dir < 0)
      unfilt_pos *= -1.0f;

    /*
    if (unfilt_pos > (float)(M_PI))
      unfilt_pos -= (float)(2.0f * M_PI);
    else if (unfilt_pos < (float)(-M_PI))
      unfilt_pos += (float)(2.0f * M_PI);
    */

    if (!g_control_init_complete)
    {
      // wait initialize the filters
      g_control_init_complete = true;
      g_state.joint_pos = unfilt_pos;
      g_control_prev_unfilt_pos = unfilt_pos;
      g_state.joint_vel = 0;
    }
    else
    {
      float raw_diff = unfilt_pos - g_control_prev_unfilt_pos;
      if (raw_diff > (float)M_PI)
      {
        //raw_diff -= (float)(2.0 * M_PI);
        g_control_joint_wraps--;
      }
      else if (raw_diff < (float)-M_PI)
      {
        //raw_diff += (float)(2.0 * M_PI);
        g_control_joint_wraps++;
      }

      const float unwrapped_pos =
        unfilt_pos + g_control_joint_wraps * (float)(2 * M_PI);

      const float raw_vel =
        (unwrapped_pos - g_control_prev_unwrapped_pos) * 20000.0f;  // 20 kHz
      g_control_prev_unwrapped_pos = unwrapped_pos;

      const float a_vel = g_control_joint_vel_lpf_alpha;  // save typing
      g_state.joint_vel =
          g_state.joint_vel * (1.0f - a_vel) + raw_vel * a_vel;

      const float a_pos = g_control_joint_pos_lpf_alpha;  // save typing
      g_state.joint_pos =
          g_state.joint_pos * (1.0f - a_pos) + unwrapped_pos * a_pos;
    }

    g_control_prev_unfilt_pos = unfilt_pos;

    /////////////////////////////////////////////

    if (g_control_mode == CONTROL_MODE_POSITION)
    {
      // clamp the position target
      float clamped_pos_target = g_control_position_target;
      if (clamped_pos_target < g_control_joint_pos_min)
        clamped_pos_target = g_control_joint_pos_min;
      else if (clamped_pos_target > g_control_joint_pos_max)
        clamped_pos_target = g_control_joint_pos_max;

      const float pos_error = clamped_pos_target - g_state.joint_pos;

      g_control_integrator += pos_error;
      g_control_integrator *= g_control_integrator_bleed;

      if (g_control_integrator > g_control_integrator_max_windup)
        g_control_integrator = g_control_integrator_max_windup;
      else if (g_control_integrator < -g_control_integrator_max_windup)
        g_control_integrator = -g_control_integrator_max_windup;

      const float deriv_error =
        (pos_error - g_control_prev_pos_error) * 20000.0f;

      g_control_voltage_target =
        g_control_position_kp * pos_error +
        g_control_position_ki * g_control_integrator +
        g_control_position_kd * deriv_error +
        g_control_vel_damp * g_state.joint_vel;

      g_control_prev_pos_error = pos_error;
    }
    else if (g_control_mode == CONTROL_MODE_VELOCITY)
    {
      // clamp the velocity target
      float clamped_vel_target = g_control_velocity_target;
      if (clamped_vel_target < g_control_velocity_min)
        clamped_vel_target = g_control_velocity_min;
      else if (clamped_vel_target > g_control_velocity_max)
        clamped_vel_target = g_control_velocity_max;

      const float vel_error = clamped_vel_target - g_state.joint_vel;

      const float vel_deriv_error =
        (vel_error - g_control_prev_vel_error) * 20000.0f;

      g_control_voltage_target =
        g_control_velocity_kp * vel_error +
        g_control_velocity_kd * vel_deriv_error;

      g_control_prev_vel_error = vel_error;
    }

    // clamp voltage target to the parameterized range
    if (g_control_voltage_target > g_control_max_voltage)
      g_control_voltage_target = g_control_max_voltage;
    else if (g_control_voltage_target < -g_control_max_voltage)
      g_control_voltage_target = -g_control_max_voltage;

    if (g_control_mode == CONTROL_MODE_VOLTAGE ||
        g_control_mode == CONTROL_MODE_POSITION ||
        g_control_mode == CONTROL_MODE_VELOCITY)
    {
      float decogged_voltage_target = g_control_voltage_target;
      if (g_control_cog_scale != 0)
      {
        decogged_voltage_target +=
          g_control_cog_scale * cog_effort(g_state.joint_pos);
      }

      const float deg120 = (float)(M_PI * 2.0f / 3.0f);
      const float elec_angle =
          fmodf(g_state.enc * (float)g_control_pole_count, 2.0f * M_PI);
      v_a = decogged_voltage_target * sinf(elec_angle);
      v_b = decogged_voltage_target * sinf(elec_angle + deg120);
      v_c = decogged_voltage_target * sinf(elec_angle + 2.0f * deg120);

      g_state.effort = g_control_voltage_target;

      g_state.phase_currents[0] = v_a;
      g_state.phase_currents[1] = v_b;
      g_state.phase_currents[2] = v_c;
    }
    else
    {
      g_state.effort = 0.0f;
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

void control_set_position_target(const float target)
{
  g_control_position_target = target;
}
