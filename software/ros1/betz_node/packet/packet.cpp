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
#include "packet.h"
using betz::Packet;

Packet::Packet()
{
}

Packet::~Packet()
{
}

void Packet::clear()
{
  flags = 0;
  drive_id = 0;
  expected_length = 0;
  rx_csum = 0;
  address.clear();
  payload.clear();
}

size_t Packet::size() const
{
  return payload.size();
}

int Packet::serialize(uint8_t *buffer, size_t buffer_len) const
{
  if (buffer_len < payload.size() + 20)
  {
    printf("ERROR: packet too long for buffer!\n");
    return -1;
  }

  buffer[0] = 0xbe;
  buffer[1] = flags;
  buffer[2] = payload.size();

  int wr_idx = 3;
  if (!(flags & FLAG_BCAST))
  {
    if (flags & FLAG_ADDR_LONG)
    {
      if (address.size() != LONG_ADDR_LEN)
      {
        printf("ERROR: requested long address, but was not provided.\n");
        return -1;
      }
      for (size_t i = 0; i < LONG_ADDR_LEN; i++)
        buffer[wr_idx++] = address[i];
    }
    else
      buffer[wr_idx++] = drive_id;
  }
  for (size_t i = 0; i < payload.size(); i++)
    buffer[wr_idx++] = payload[i];

  CRC crc;
  for (size_t i = 0; i < wr_idx; i++)
    crc.add_byte(buffer[i]);
  buffer[wr_idx++] = static_cast<uint8_t>(crc.get_crc() & 0xff);
  buffer[wr_idx++] = static_cast<uint8_t>(crc.get_crc() >> 8);
  return wr_idx;
}

uint8_t Packet::packet_id() const
{
  if (!payload.empty())
    return payload[0];
  else
    return 0;
}
