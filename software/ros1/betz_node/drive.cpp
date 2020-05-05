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
  params.resize(num_params);
}

void Drive::rx_flash_read(const Packet& packet)
{
  // ROS_INFO("rx_flash_read(%d)", static_cast<int>(packet.payload.size()));
  if (packet.payload.size() < 10)
  {
    ROS_ERROR("unexpected payload len: %zu", packet.payload.size());
    return;
  }
  memcpy(&flash_last_addr, &packet.payload[1], 4);
  uint32_t read_len = 0;
  memcpy(&read_len, &packet.payload[5], 4);
  // sanity-check the read length (!!)
  if (read_len > 128)
  {
    ROS_ERROR("unexpected read len: %u", read_len);
    return;
  }
  flash_last_read.resize(read_len);
  memcpy(&flash_last_read[0], &packet.payload[9], read_len);
  // ROS_INFO("received %u bytes from 0x%08x", read_len, flash_last_addr);
}

void Drive::rx_flash_write(const Packet& packet)
{
  // nothing to do (for now, at least) with this notification of
  // a successful write
}

void Drive::rx_boot(const Packet& packet)
{
  is_bootloader = false;
}

void Drive::rx_param_name_value(const Packet& packet)
{
  packet.print();
  Param param;
  memcpy(&param.idx, &packet.payload[1], sizeof(param.idx));
  param.type = static_cast<Param::Type>(packet.payload[5]);
  const int name_len = packet.payload[6];
  if (name_len > packet.payload.size() - 11)
  {
    ROS_ERROR("bogus param name+value packet!");
    return;
  }
  param.name = string(packet.payload[7], name_len);
  const int value_pos = 7 + name_len;
  if (param.type == Param::Type::INT)
  {
    int32_t v = 0;
    memcpy(&v, &packet.payload[value_pos], sizeof(v));
    param.i_value = v;
  }
  else if (param.type == Param::Type::FLOAT)
  {
    float v = 0;
    memcpy(&v, &packet.payload[value_pos], sizeof(v));
    param.f_value = v;
  }
  params.push_back(param);
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
    case Packet::ID_NUM_PARAMS:       rx_num_params(packet); break;
    case Packet::ID_PARAM_NAME_VALUE: rx_param_name_value(packet); break;
    case Packet::ID_DISCOVERY:        rx_discovery(packet); break;
    case Packet::ID_FLASH_READ:       rx_flash_read(packet); break;
    case Packet::ID_FLASH_WRITE:      rx_flash_write(packet); break;
    case Packet::ID_BOOT:             rx_boot(packet); break;
    default:
      ROS_INFO(
          "unrecognized packet ID: %02x",
          static_cast<unsigned>(packet_id));
  }
}

void Drive::rx_discovery(const Packet& packet)
{
  ROS_INFO("discovered %s", uuid.to_string().c_str());
  if (packet.payload.size() != 2)
  {
    ROS_ERROR("unexpected payload len: %zu", packet.payload.size());
    return;
  }
  is_bootloader = packet.payload[1];
}
