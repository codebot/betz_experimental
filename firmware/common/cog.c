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

#include <stdbool.h>
#include <math.h>

#include "cog.h"
#include "param.h"

static int g_cog_table_valid = 0;
static float g_cog_table_start = 0;
static float g_cog_table_end = 0;
static int g_cog_table_size = 0;
static float *g_cog_table;

void cog_init()
{
  g_cog_table = (float *)0x08070000;  // do you believe in magic?

  param_int(
      "cog_table_size",
      &g_cog_table_size,
      0,
      PARAM_PERSISTENT);

  param_float(
      "cog_table_start",
      &g_cog_table_start,
      0,
      PARAM_PERSISTENT);

  param_float(
      "cog_table_end",
      &g_cog_table_end,
      0,
      PARAM_PERSISTENT);

  param_int(
      "cog_table_valid",
      &g_cog_table_valid,
      0,
      PARAM_TRANSIENT);

  // spin through the cog table and make sure the values are sane
  if (g_cog_table_size > 0 && g_cog_table_size <= 16384)
  {
    bool all_ok = true;
    for (int i = 0; i < g_cog_table_size; i++)
    {
      if (g_cog_table[i] < -20 || g_cog_table[i] > 20)
      {
        all_ok = false;
        break;
      }
    }

    g_cog_table_valid = all_ok ? 1 : 0;
  }
}

float cog_effort(const float joint_angle)
{
  if (!g_cog_table_valid || g_cog_table_end <= g_cog_table_start)
    return 0;

  const float cog_table_distance = g_cog_table_end - g_cog_table_start;

  float unwrapped_joint_angle =
    fmodf(joint_angle - g_cog_table_start, cog_table_distance);
  if (unwrapped_joint_angle < 0)
    unwrapped_joint_angle += (float)(2.0 * M_PI);

  // for now, just use the nearest table entry. interpolation can happen
  // later if it looks promising.
  const int table_idx =
      unwrapped_joint_angle * g_cog_table_size / cog_table_distance;
  if (table_idx < 0 || table_idx >= g_cog_table_size)
    return 0;  // last sanity check

  return g_cog_table[table_idx];
}
