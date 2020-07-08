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

#ifndef PACKET_PACKET_H
#define PACKET_PACKET_H

#include <cstdint>
#include <string>
#include <vector>
#include "betz/uuid.h"

namespace betz {

class Packet
{
public:
  Packet();
  virtual ~Packet();

  uint8_t flags = 0;
  uint8_t drive_id = 0;
  uint8_t expected_length = 0;
  uint16_t rx_csum = 0;

  UUID uuid;
  std::vector<uint8_t> payload;

  std::string uuid_str() const;  // returns the uuid as a string

  static const uint8_t FLAG_DIR = 0x1;
  static const uint8_t FLAG_DIR_HOST_PERIPH = 0x0;
  static const uint8_t FLAG_DIR_PERIPH_HOST = 0x1;

  static const uint8_t FLAG_BCAST = 0x2;

  static const uint8_t FLAG_ADDR = 0x4;
  static const uint8_t FLAG_ADDR_ID = 0x0;
  static const uint8_t FLAG_ADDR_UUID = 0x4;

  static const uint8_t FLAG_SENTINEL = 0x50;

  enum
  {
    ID_NUM_PARAMS = 0x01,
    ID_PARAM_NAME_VALUE = 0x02,
    ID_PARAM_SET_VALUE = 0x03,
    ID_PARAM_WRITE_FLASH = 0x04,
    ID_COG_WRITE_FLASH = 0x05,

    ID_STATE_POLL = 0x10,
    ID_TERSE_STATE_POLL = 0x12,

    ID_SET_POSITION_TARGET = 0x20,

    ID_DISCOVERY  = 0xf0,
    ID_FLASH_READ = 0xf1,
    ID_FLASH_WRITE = 0xf2,
    ID_BOOT = 0xf3,
    ID_RESET = 0xf4,
  };

  void clear();
  size_t size() const;
  uint8_t packet_id() const;

  int serialize(uint8_t *buffer, size_t buffer_len) const;

  void append(const uint8_t b);
  void append(const uint16_t s);
  void append(const uint32_t i);
  void append(const int32_t i);
  void append(const float f);

  virtual void print() const;
};

}  // namespace betz

#endif
