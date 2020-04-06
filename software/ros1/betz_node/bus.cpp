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

#include "bus.h"
#include "crc.h"
#include "ros/ros.h"
using std::string;
using betz::Bus;
using betz::Drive;

Bus::Bus()
{
}

Bus::~Bus()
{
}

bool Bus::open_device(const string& device_name)
{
  serial = new LightweightSerial(device_name.c_str(), 1000000);
  if (!serial)
  {
    ROS_FATAL("couldn't open serial device");
    return false;
  }
  ROS_INFO("opened %s", device_name.c_str());
  return true;
}

bool Bus::send_packet(const uint8_t *data, const uint32_t len)
{
  if (len >= 252)
  {
    ROS_ERROR("refusing to send %d-byte packet. must be <252.",
        static_cast<int>(len));
    return false;
  }
  // ROS_INFO("sending %d-byte packet", static_cast<int>(len));
  uint8_t framed_pkt[len+5];
  framed_pkt[0] = 0xbe;
  framed_pkt[1] = 0xef;
  framed_pkt[2] = static_cast<uint8_t>(len);
  for (uint32_t i = 0; i < len; i++)
    framed_pkt[i+3] = data[i];

  CRC crc;
  for (uint32_t i = 0; i < len; i++)
    crc.add_byte(framed_pkt[i]);
  framed_pkt[len+3] = static_cast<uint8_t>(crc.get_crc() & 0xff);
  framed_pkt[len+4] = static_cast<uint8_t>(crc.get_crc() >> 8);
  return serial->write_block(framed_pkt, len+5);
}

// this function spins max_seconds or until packet_id arrives
bool Bus::wait_for_packet(
    const double max_seconds,
    const uint8_t drive_id,
    const uint8_t packet_id)
{
  ros::Rate loop_rate(1000);
  ros::Time t_end = ros::Time::now() + ros::Duration(max_seconds);
  while (ros::ok())
  {
    if (max_seconds > 0 && t_end < ros::Time::now())
      break;
    loop_rate.sleep();
    ros::spinOnce();

    Packet packet;
    uint8_t b = 0;
    while (serial->read(&b))
    {
      //printf("rx: %02x\n", b);
      if (rx_byte(b, packet))
      {
        if (packet.drive_id == drive_id && packet.packet_id == packet_id)
          return true;
      }
    }
  }
  ROS_WARN("rs485_spin timeout :(");
  return false;
}

bool Bus::rx_byte(const uint8_t b, Packet& rx_pkt)
{
  switch (parser_state)
  {
    case ParserState::PREAMBLE:
      if (b == 0xbe)
      {
        parser_crc.reset();
        parser_crc.add_byte(b);
        parser_state = ParserState::FLAGS;
        parser_packet.clear();
      }
      break;

    case ParserState::FLAGS:
      parser_crc.add_byte(b);
      if ((b & 0xf0) != Packet::FLAG_SENTINEL)
      {
        parser_state = ParserState::PREAMBLE;
        break;
      }
      parser_packet.flags = b;
      if (!(parser_packet.flags & Packet::FLAG_BCAST))
        parser_state = ParserState::ADDRESS;
      else
        parser_state = ParserState::LENGTH;
      break;

    case ParserState::ADDRESS:
      parser_crc.add_byte(b);
      if ((parser_packet.flags & Packet::FLAG_ADDR) == Packet::FLAG_ADDR_LONG)
      {
        parser_packet.address.push_back(b);
        if (parser_packet.address.size() == Packet::LONG_ADDR_LEN)
          parser_state = ParserState::LENGTH;
      }
      else
      {
        // short id = single byte
        parser_packet.packet_id = b;
        parser_state = ParserState::LENGTH;
      }
      break;

    case ParserState::LENGTH:
      parser_crc.add_byte(b);
      parser_packet.expected_length = b;
      if (parser_packet.expected_length == 0)
        parser_state = ParserState::PREAMBLE;  // bogus packet
      else
        parser_state = ParserState::PAYLOAD;
      break;

    case ParserState::PAYLOAD:
      parser_crc.add_byte(b);
      parser_packet.payload.push_back(b);
      if (parser_packet.payload.size() ==
          static_cast<size_t>(parser_packet.expected_length))
        parser_state = ParserState::CSUM_0;
      break;

    case ParserState::CSUM_0:
      parser_packet.rx_csum = b;
      parser_state = ParserState::CSUM_1;
      break;

    case ParserState::CSUM_1:
      parser_state = ParserState::PREAMBLE;
      parser_packet.rx_csum |= (b << 8);
      if (parser_packet.rx_csum == parser_crc.get_crc())
      {
        printf("received packet, hooray\n");
        rx_pkt = parser_packet;
        return true;
      }
      else
      {
        printf("csum fail. 0x%0x != 0x%0x\n",
            parser_crc.get_crc(),
            parser_packet.rx_csum);
      }
      break;

    default:
      parser_state = ParserState::PREAMBLE;  // shouldn't ever get here...
      break;
  }

  return false;
}

void Bus::rx_packet(Packet& packet)
{
  for (auto& drive : drives)
    if (drive.id == packet.drive_id)
    {
      drive.rx_packet(packet);
      break;
    }
}

bool Bus::read_flash(
    const uint8_t drive_id,
    const uint32_t start_addr,
    const uint32_t len)
{
  add_drive_id(drive_id);

  uint8_t pkt[9] = {0};
  pkt[0] = 0xf0;
  memcpy(&pkt[1], &start_addr, sizeof(uint32_t));
  memcpy(&pkt[5], &len, sizeof(uint32_t));
  send_packet(pkt, sizeof(pkt));

  if (wait_for_packet(1.0, drive_id, 0xf0) == 0xf0)
    return true;
  return false;
}

bool Bus::set_led(const uint8_t drive_id, const bool on)
{
  add_drive_id(drive_id);

  uint8_t pkt[2] = { 0x10, 0x00 };
  if (on)
    pkt[1] = 1;
  return send_packet(pkt, sizeof(pkt));
}

int Bus::get_num_params(const uint8_t drive_id)
{
  add_drive_id(drive_id);

  uint8_t pkt[1] = { 0x01 };
  send_packet(pkt, sizeof(pkt));
  if (wait_for_packet(1.0, drive_id, 0x01))
  {
    Drive *drive = find_drive_by_id(drive_id);
    return drive->num_params;
  }
  return -1;
}

Drive *Bus::find_drive_by_id(const uint8_t drive_id)
{
  for (size_t i = 0; i < drives.size(); i++)
    if (drives[i].id == drive_id)
      return &drives[i];
  return nullptr;
}

void Bus::add_drive_id(const uint8_t drive_id)
{
  for (auto& drive : drives)
    if (drive.id == drive_id)
      return;  // nothing to do; drive already exists

  Drive drive;
  drive.id = drive_id;
  drives.push_back(drive);
}
