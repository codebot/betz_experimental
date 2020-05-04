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

#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>
#include <stdint.h>

void comms_init(void (*tx_fptr)(const uint8_t *, const uint32_t));

void comms_tick();
void comms_rx_byte(const uint8_t byte);

void comms_set_bootloader_mode(const bool is_bootloader);

#endif
