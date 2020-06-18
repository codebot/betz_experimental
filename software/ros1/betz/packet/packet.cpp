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
#include <cstring>
#include "betz/crc.h"
#include "betz/packet.h"
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
  uuid.clear();
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
  int wr_idx = 2;
  if (!(flags & FLAG_BCAST))
  {
    if (flags & FLAG_ADDR_UUID)
    {
      if (!uuid.is_valid())
      {
        printf("ERROR: requested long address, but was not provided.\n");
        return -1;
      }
      for (size_t i = 0; i < UUID::UUID_LEN; i++)
        buffer[wr_idx++] = uuid.bytes[i];
    }
    else
      buffer[wr_idx++] = drive_id;
  }
  buffer[wr_idx++] = payload.size();
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

void Packet::append(const uint8_t b)
{
  payload.push_back(b);
}

void Packet::append(const uint16_t s)
{
  // little-endian. I'm sure there is a faster way someday, but it's nice
  // to be explicit also
  payload.push_back(s & 0xff);
  payload.push_back((s >> 8) & 0xff);
}

void Packet::append(const uint32_t i)
{
  // little-endian. I'm sure there is a faster way someday, but it's nice
  // to be explicit also
  payload.push_back(i & 0xff);
  payload.push_back((i >> 8) & 0xff);
  payload.push_back((i >> 16) & 0xff);
  payload.push_back((i >> 24) & 0xff);
}

void Packet::append(const int32_t i)
{
  payload.push_back(i & 0xff);
  payload.push_back((i >> 8) & 0xff);
  payload.push_back((i >> 16) & 0xff);
  payload.push_back((i >> 24) & 0xff);
}

void Packet::append(const float f)
{
  uint32_t i = 0;
  memcpy(&i, &f, 4);
  payload.push_back(i & 0xff);
  payload.push_back((i >> 8) & 0xff);
  payload.push_back((i >> 16) & 0xff);
  payload.push_back((i >> 24) & 0xff);
}

void Packet::print() const
{
  for (size_t i = 0; i < payload.size(); i++)
  {
    printf("%3d: 0x%02x\r\n", (int)i, (unsigned)payload[i]);
  }
}
