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

#include "delay.h"
#include "systime.h"

// these functions assume that systime is running at microsecond resolution

void delay_ns(uint32_t ns)
{
  // TODO: actually tune this better on an oscilloscope
  for (volatile uint32_t i = 0; i < ns/10; i++) { }
}

void delay_us(uint32_t us)
{
  // todo: deal with wraparound
  volatile uint32_t t_start = systime_read();
  while (t_start + us > systime_read()) { }
}

void delay_ms(uint32_t ms)
{
  // todo: deal with wraparound
  volatile uint32_t t_start = systime_read();
  while (t_start + 1000 * ms > systime_read()) { }
}
