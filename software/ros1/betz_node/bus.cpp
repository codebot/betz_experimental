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

#include <memory>

#include "bus.h"
#include "ros/ros.h"
#include "transport_serial.h"
#include "packet/discovery.h"
#include "packet/num_params.h"

using std::make_shared;
using std::make_unique;
using std::shared_ptr;
using std::string;

using betz::Bus;
using betz::Drive;

Bus::Bus()
{
}

Bus::~Bus()
{
}

bool Bus::send_packet(std::unique_ptr<Packet> packet)
{
  uint8_t buffer[256];
  const int serialized_length = packet->serialize(buffer, sizeof(buffer));
  if (serialized_length < 0)
  {
    ROS_ERROR("packet serialization error!");
    return false;
  }
  ROS_INFO("sending %d-byte packet:", static_cast<int>(serialized_length));
  // for (size_t i = 0; i < serialized_length; i++)
  //   printf("  %2zu: 0x%02x\n", i, static_cast<unsigned>(buffer[i]));
  return transport->send(buffer, serialized_length);
}

// this function spins max_seconds or until packet_id arrives
bool Bus::wait_for_packet(
    const double max_seconds,
    const uint8_t drive_id,
    const uint8_t packet_id)
{
  ros::Rate loop_rate(1000);
  ros::Time t_end = ros::Time::now() + ros::Duration(max_seconds);
  static uint8_t rx_buf[1024] = {0};
  while (ros::ok())
  {
    if (max_seconds > 0 && t_end < ros::Time::now())
      break;
    loop_rate.sleep();
    ros::spinOnce();

    Packet packet;

    int n_rx = transport->recv_nonblocking(rx_buf, sizeof(rx_buf));
    printf("n_rx = %d\n", n_rx);

    for (int i = 0; i < n_rx; i++)
    {
      const uint8_t b = rx_buf[i];
      printf("  rx: %02x\n", b);
      if (rx_byte(b, packet))
      {
        if (packet.drive_id == drive_id && packet.packet_id() == packet_id)
          return true;
      }
    }
  }
  ROS_WARN("Bus::wait_for_packet timeout :(");
  return false;
}

bool Bus::rx_byte(const uint8_t b, Packet& rx_pkt)
{
#if 0
  printf("    rx: 0x%02x  parser_state = %d\n",
      static_cast<unsigned>(b),
      static_cast<int>(parser_state));
#endif

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
      if (parser_packet.flags & Packet::FLAG_ADDR_UUID)
      {
        parser_packet.uuid.push_back(b);
        if (parser_packet.uuid.size() == Packet::UUID_LEN)
          parser_state = ParserState::LENGTH;
      }
      else
      {
        // short id = single byte
        parser_packet.drive_id = b;
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

#if 0
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

#endif

shared_ptr<Drive> Bus::drive_by_uuid(const std::vector<uint8_t>& uuid)
{
  for (auto drive : drives)
    if (drive->uuid_equals(uuid))
      return drive;

  // if we get here, that means we didn't find this UUID. create a new one.
  shared_ptr<Drive> drive = make_shared<Drive>();
  drive->set_uuid(uuid);
  drives.push_back(drive);

  return drive;
}

/*
Drive *Bus::find_drive_by_id(const uint8_t drive_id)
{
  for (size_t i = 0; i < drives.size(); i++)
    if (drives[i].id == drive_id)
      return &drives[i];
  return nullptr;
}
*/

void Bus::set_transport(std::unique_ptr<Transport> _transport)
{
  transport = std::move(_transport);
}

void Bus::spin_once()
{
  if (!transport)
  {
    ROS_ERROR("Bus::spin_once() with null transport!");
    return;
  }

  Packet packet;
  while (true)
  {
    int n_rx = transport->recv_nonblocking(rx_buf, sizeof(rx_buf));
    if (n_rx == 0)
      break;

    for (int i = 0; i < n_rx; i++)
    {
      const uint8_t b = rx_buf[i];
      // printf("rx 0x%02x\n", static_cast<unsigned>(b));
      if (rx_byte(b, packet) && (packet.flags & Packet::FLAG_DIR_PERIPH_HOST))
      {
        printf("received packet from peripheral\n");
        shared_ptr<Drive> drive = drive_by_uuid(packet.uuid);
        drive->rx_packet(packet);
      }
    }
  }

  if (!discovery_complete)
    discovery_tick();
}

void Bus::discovery_begin()
{
  printf("Bus::discovery_begin()\n");
  discovery_time = ros::Time::now();
  discovery_complete = false;
  discovery_broadcast_count = 0;
  send_packet(std::make_unique<Discovery>());
}

void Bus::discovery_tick()
{
  const double elapsed = (ros::Time::now() - discovery_time).toSec();

  /*
  printf(
      "Bus::discovery_tick() elapsed = %.3f %zu drives\n",
      elapsed,
      drives.size());
  */

  switch (discovery_state)
  {
    case DiscoveryState::UUID:
      // send out broadcast discovery requests to learn drive UUID's
      if (elapsed > 0.5)
      {
        if (discovery_broadcast_count > 1)
        {
          discovery_state = DiscoveryState::NUM_PARAMS;
          break;
        }
        discovery_broadcast_count++;
        discovery_time = ros::Time::now();
        send_packet(std::make_unique<Discovery>());
      }
      break;

    case DiscoveryState::NUM_PARAMS:
      for (auto& drive : drives)
      {
        if (drive->num_params == 0)
        {
          send_packet(std::make_unique<NumParams>(*drive));
          return;
        }
      }
      break;

    default:
      printf("unhandled discovery state\n");
      break;
  }
}
