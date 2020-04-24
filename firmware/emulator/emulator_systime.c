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

#include "systime.h"
#include <stdlib.h>
#include <time.h>

static struct timespec g_systime_t_start;

void systime_init()
{
  clock_gettime(CLOCK_REALTIME, &g_systime_t_start);
}

uint32_t systime_read()
{
  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);

  int32_t nsec_diff = t.tv_nsec - g_systime_t_start.tv_nsec;
  int32_t sec_diff = t.tv_sec - g_systime_t_start.tv_sec;

  if (nsec_diff < 0)
  {
    nsec_diff += 1000000000;
    sec_diff -= 1;
  }

  return (nsec_diff / 1000) + sec_diff * 1000000;
}
