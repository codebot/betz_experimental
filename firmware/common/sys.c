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
#include "sys.h"
#include "stm32f405xx.h"

void sys_run_application()
{
  printf("sys_run_application()\r\n");
  __disable_irq();
  SCB->VTOR = 0x20000;  // move the vector table offset
  // next, we do some extreme low level action and load the new stack
  // pointer, set the PC to the application, and brace for impact
  __asm volatile("ldr r0, =0x08020000 \n"
                 "ldr sp, [r0]        \n"
                 "ldr pc, [r0, #4]    \n");
  // BOOM we have now teleported into the application
}

void sys_reset()
{
  NVIC_SystemReset();  // BOOM
}
