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

#include "status_led.h"
#include "pin.h"
#include "soc.h"

#if defined(BOARD_blue)
#  define LED_GPIO GPIOB
#  define LED_PIN  8
#elif defined(BOARD_mini)
#  define LED_GPIO GPIOC
#  define LED_PIN 13
#endif

void status_led_init()
{
  pin_set_output(LED_GPIO, LED_PIN, 0);
}

void status_led_on()
{
  pin_set_output_low(LED_GPIO, LED_PIN);
}

void status_led_off()
{
  pin_set_output_high(LED_GPIO, LED_PIN);
}

void status_led_toggle()
{
  pin_toggle_state(LED_GPIO, LED_PIN);
}
