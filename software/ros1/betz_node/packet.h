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

#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include <vector>

class Packet
{
public:
  Packet();
  ~Packet();

  uint8_t flags = 0;
  uint8_t drive_id = 0;
  uint8_t packet_id = 0;
  uint8_t addr[12] = {0};

  std::vector<uint8_t> payload;
};

#endif
