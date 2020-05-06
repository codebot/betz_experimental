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
#include "param.h"

#ifndef EMULATOR
#include "stm32f405xx.h"
#endif

static int g_tick_count = 0;

void control_init()
{
}

#ifndef EMULATOR
// todo: consider using the ramfunc section
void tim1_up_tim10_vector()
{
  TIM1->SR &= ~TIM_SR_UIF;  // clear the interrupt flag that got us here
  if (TIM1->CR1 & TIM_CR1_DIR)
  {
    // all shunts are ready to measure
    g_tick_count++;
    if (g_tick_count % 20000 == 0)
    {
    }
  }
  else
  {
    // we're at the top of the cycle, might as well kick off an encoder read
  }
}
#endif
