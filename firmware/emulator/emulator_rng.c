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

#include "rng.h"
#include <stdlib.h>
#include <time.h>

void rng_init()
{
  srandom(time(NULL));
}

uint32_t rng_read()
{
  // make sure we get a full 32-bit random, to be like the STM32 RNG
  // otherwise we might just get a 15-bit random number and be sad.
  uint32_t rands[4] = {0};
  for (int i = 0; i < 4; i++)
    rands[i] = random() & 0xff;
  return rands[0] | (rands[1] << 8) | (rands[2] << 16) | (rands[3] << 24);
}
