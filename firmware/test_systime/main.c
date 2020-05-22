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
#include "delay.h"
#include "systime.h"
#include "status_led.h"

int main(int argc, char **argv)
{
  while (1)
  {
    for (int i = 0; i < 10; i++)
      delay_ms(100);

    status_led_toggle();
    const volatile uint32_t systime = systime_read();
    printf("systime = %d\r\n", (int)systime);
  }
  return 0;
}
