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

#include <stdint.h>
#include <vector>

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

  static const size_t UUID_LEN = 12;

  std::vector<uint8_t> uuid;
  std::vector<uint8_t> payload;

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
    ID_PARAM_NAME = 0x02,
    ID_PARAM_VALUE = 0x03,

    ID_DISCOVERY  = 0xf0,
    ID_FLASH_READ = 0xf1,
  };
  //static const uint8_t 

  void clear();
  size_t size() const;
  uint8_t packet_id() const;

  int serialize(uint8_t *buffer, size_t buffer_len) const;

  void append(const uint8_t b);
  void append(const uint16_t s);
  void append(const uint32_t i);
};

}  // namespace betz

#endif
