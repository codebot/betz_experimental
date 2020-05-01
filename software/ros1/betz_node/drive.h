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

#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>
#include <string>

#include "packet/packet.h"
#include "transport.h"

namespace betz {

class Drive
{
public:
  uint8_t id = 0;
  std::vector<uint8_t> uuid;
  std::string uuid_str;  // for printing messages to console

  std::vector<uint8_t> flash_last_read;
  uint32_t flash_last_addr;

  bool is_bootloader = false;
  int num_params = 0;

  void rx_packet(const Packet& packet);
  void rx_num_params(const Packet& packet);
  void rx_flash_read(const Packet& packet);
  void rx_discovery(const Packet& packet);

  Drive();
  ~Drive();

  bool uuid_equals(const std::vector<uint8_t>& _uuid) const;
  void set_uuid(const std::vector<uint8_t>& _uuid);
  void print() const;
};

}  // namespace betz

#endif
