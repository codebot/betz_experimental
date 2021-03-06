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

#ifndef STATE_H
#define STATE_H

#include <stdint.h>

struct state_t
{
  uint32_t t;  // systime at instant of PWM cycle start
  float enc;  // encoder (radians)
  float joint_pos;  // joint position (radians), typically offset from encoder
  float joint_vel;  // joint velocity (radians/second)
  uint16_t raw_adc[3];
  float phase_currents[3];
  float effort;
};

extern struct state_t g_state;

void state_init();

#endif
