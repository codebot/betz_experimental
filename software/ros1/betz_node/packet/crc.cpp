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

#include <cstdio>
#include "crc.h"

CRC::CRC()
{
}

CRC::~CRC()
{
}

void CRC::add_byte(const uint8_t b)
{
  // todo: replace with a real CRC-16 someday. this is a placeholder to
  // get unstuck while standing up other stuff
  crc = ((crc & 0x8000) >> 15) | (crc << 1);
  crc ^= b;
  //printf("crc = %02x\n", static_cast<unsigned>(crc));
}

void CRC::reset()
{
  crc = 0;
}
