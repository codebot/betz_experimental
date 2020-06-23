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

#ifndef PWM_H
#define PWM_H

#include <stdbool.h>
#include <stdint.h>

#if defined(BOARD_blue)

// 84 MHz / 20 KHz = 4200
#define PWM_MAX 4200

#elif defined(BOARD_mini)

// 168 MHz / 20 KHz = 8400 for half cycle, need 4200 for full cycle
#define PWM_MAX 4200

#elif defined(EMULATOR)

#define PWM_MAX 4200

#endif

void pwm_init();
void pwm_set(const uint32_t a, const uint32_t b, const uint32_t c);
uint32_t pwm_max();
void pwm_enable(const bool enable);

#endif
