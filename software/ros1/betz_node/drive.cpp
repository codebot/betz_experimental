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

#include "drive.h"
#include "ros/ros.h"
#include <string.h>  // for memcpy
using betz::Drive;


Drive::Drive()
{
}

Drive::~Drive()
{
}

void Drive::rx_num_params(const Packet& packet)
{
  ROS_INFO("Drive::rx_num_params()");
  if (packet.payload.size() != 4) {
    ROS_ERROR("unexpected payload len: %d", (int)packet.payload.size());
    return;
  }
  num_params =
      packet.payload[0]         |
      (packet.payload[1] << 8)  |
      (packet.payload[2] << 16) |
      (packet.payload[3] << 24) ;
}

void Drive::rx_flash_read(const Packet& packet)
{
  ROS_INFO("rs485_rx_flash_read(%d)", static_cast<int>(packet.payload.size()));
}

void Drive::rx_packet(const Packet& packet)
{
  if (packet.payload.size() == 0)
  {
    ROS_ERROR("received zero-length rs485 packet");
    return;
  }

  // dispatch to packet handler, to keep the function size sane
  const uint8_t pkt_id = packet.payload[0];
  switch(pkt_id)
  {
    case 0x01: rx_num_params(packet); break;
    case 0xf0: rx_flash_read(packet); break;
    default: ROS_INFO("unrecognized packet ID: %02x", (int)pkt_id);
  }
}

bool Drive::uuid_equals(const std::vector<uint8_t>& _uuid) const
{
  for (size_t i = 0; i < uuid.size() && i < _uuid.size(); i++)
    if (uuid[i] != _uuid[i])
      return false;
  return true;
}

void Drive::set_uuid(const std::vector<uint8_t>& _uuid)
{
  uuid.resize(_uuid.size());
  for (size_t i = 0; i < _uuid.size(); i++)
    uuid[i] = _uuid[i];
}
