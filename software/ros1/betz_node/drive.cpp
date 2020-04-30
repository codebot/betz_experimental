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
using std::string;


Drive::Drive()
{
}

Drive::~Drive()
{
}

void Drive::rx_num_params(const Packet& packet)
{
  ROS_INFO("Drive::rx_num_params()");
  if (packet.payload.size() != 5)
  {
    ROS_ERROR("unexpected payload len: %d", (int)packet.payload.size());
    return;
  }
  memcpy(&num_params, &packet.payload[1], 4);
  printf("num_params = %u\n", num_params);
}

void Drive::rx_flash_read(const Packet& packet)
{
  ROS_INFO("rs485_rx_flash_read(%d)", static_cast<int>(packet.payload.size()));
}

void Drive::rx_packet(const Packet& packet)
{
  if (packet.payload.size() == 0)
  {
    ROS_ERROR("received zero-length packet");
    return;
  }

  // dispatch to packet handler, to keep the function size sane
  const uint8_t packet_id = packet.payload[0];
  switch(packet_id)
  {
    case Packet::ID_DISCOVERY:  rx_discovery(packet); break;
    case Packet::ID_NUM_PARAMS: rx_num_params(packet); break;
    // case 0xf0: rx_flash_read(packet); break;
    default:
      ROS_INFO(
          "unrecognized packet ID: %02x",
          static_cast<unsigned>(packet_id));
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
  uuid_str = string();
  for (size_t i = 0; i < _uuid.size(); i++)
  {
    uuid[i] = _uuid[i];

    char byte_buf[10] = {0};
    snprintf(
        byte_buf,
        sizeof(byte_buf),
        "%02x",
        static_cast<unsigned>(_uuid[i]));
    uuid_str += string(byte_buf);
    if (i % 4 == 3 && i != 11)
      uuid_str += ":";
  }
}

void Drive::rx_discovery(const Packet& packet)
{
  ROS_INFO("discovered %s", uuid_str.c_str());
  if (packet.payload.size() != 2)
  {
    ROS_ERROR("unexpected payload len: %zu", packet.payload.size());
    return;
  }
  is_bootloader = packet.payload[1];
}
