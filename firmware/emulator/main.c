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

#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "comms.h"
#include "control.h"
#include "flash.h"
#include "multicast.h"
#include "param.h"
#include "rng.h"
#include "state.h"
#include "uuid.h"
#include "systime.h"

#include "emulator_uuid.h"

void emulator_physics_init()
{
  g_state.enc = 0;
  srandom(0);
}

void emulator_physics_tick()
{
  float denc = ((random() % 1001 - 500) * 0.001) * 0.01;
  g_state.enc += denc;
  if (g_state.enc < 0)
    g_state.enc += 2 * M_PI;
  else if (g_state.enc >= 2 * M_PI)
    g_state.enc -= 2 * M_PI;
}

int usage()
{
  printf("syntax: comms_emulator SERIAL_NUMBER\n");
  return 1;
}

int main(int argc, char **argv)
{
  // find the serial number from the params
  if (argc < 2)
    return usage();

  const uint64_t serial_number = strtoull(argv[1], NULL, 16);
  emulator_uuid_set_serial_number(serial_number);

  flash_init();
  systime_init();
  rng_init();

  if (!multicast_init())
    return 1;

  uuid_init();
  state_init();
  param_init();
  control_init();
  comms_init(multicast_tx);
  comms_set_bootloader_mode(true);

  while (true)
  {
    multicast_listen(0);
    emulator_physics_tick();
    comms_tick();
    control_tick();
    usleep(10000);
  }
  return 0;
}
