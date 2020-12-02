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

#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>

void control_init();
void control_tick();
void control_set_position_target(const float target);

#define CONTROL_MODE_IDLE     0
#define CONTROL_MODE_VOLTAGE  1
#define CONTROL_MODE_POSITION 2
#define CONTROL_MODE_VELOCITY 3

#endif
