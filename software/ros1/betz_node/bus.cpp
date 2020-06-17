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
#include "transport_multicast.h"
#include "transport_serial.h"

#include "packet/boot.h"
#include "packet/discovery.h"
#include "packet/flash_read.h"
#include "packet/flash_write.h"
#include "packet/num_params.h"
#include "packet/param_name_value.h"
#include "packet/reset.h"

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
  // ROS_INFO("sending %d-byte packet:", static_cast<int>(serialized_length));
  // for (size_t i = 0; i < serialized_length; i++)
  //   printf("  %2zu: 0x%02x\n", i, static_cast<unsigned>(buffer[i]));
  return transport->send(buffer, serialized_length);
}

// this function spins max_seconds or until packet_id arrives
bool Bus::wait_for_packet(const double max_seconds, const uint8_t packet_id)
{
  ros::Rate loop_rate(10000);
  ros::Time t_end = ros::Time::now() + ros::Duration(max_seconds);
  static uint8_t rx_buf[1024] = {0};
  while (ros::ok())
  {
    if (max_seconds > 0 && t_end < ros::Time::now())
      break;
    ros::spinOnce();
    if (spin_once(packet_id))
      return true;
    loop_rate.sleep();
  }
  ROS_WARN("Bus::wait_for_packet timeout :(");
  return false;
}

bool Bus::rx_byte(const uint8_t b, Packet& rx_pkt)
{
  /*
  printf("    rx: 0x%02x  parser_state = %d\n",
      static_cast<unsigned>(b),
      static_cast<int>(parser_state));
  */

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
        parser_packet.uuid.bytes.push_back(b);
        if (parser_packet.uuid.bytes.size() == UUID::UUID_LEN)
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
        parser_packet.uuid.generate_string();
        rx_pkt = parser_packet;
        return true;
      }
      else
      {
        printf("csum fail. expected 0x%0x, received 0x%0x\n",
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
bool Bus::set_led(const uint8_t drive_id, const bool on)
{
  add_drive_id(drive_id);

  uint8_t pkt[2] = { 0x10, 0x00 };
  if (on)
    pkt[1] = 1;
  return send_packet(pkt, sizeof(pkt));
}
#endif

shared_ptr<Drive> Bus::drive_by_uuid(const UUID& uuid)
{
  for (auto drive : drives)
    if (drive->uuid == uuid)
      return drive;

  // if we get here, that means we didn't find this UUID. create a new one.

  // todo: sort by ID and UUID during insertion
  shared_ptr<Drive> drive = make_shared<Drive>();
  drive->uuid = uuid;
  drives.push_back(drive);

  return drive;
}

// drive_by_uuid_str() is used by the GUI. It does not create a new drive
// if one doesn't match the requested UUID string
shared_ptr<Drive> Bus::drive_by_uuid_str(const string& uuid_str)
{
  for (auto drive : drives)
    if (drive->uuid.to_string() == uuid_str)
      return drive;

  return nullptr;
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

bool Bus::spin_once(const uint8_t watch_packet_id)
{
  if (!transport)
  {
    ROS_ERROR("Bus::spin_once() with null transport!");
    return false;
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
        // printf("received packet from peripheral\n");

        shared_ptr<Drive> drive = drive_by_uuid(packet.uuid);
        drive->rx_packet(packet);

        if (packet_listener)
          packet_listener(packet);

        if (watch_packet_id && (watch_packet_id == packet.packet_id()))
          return true;
      }
    }
  }

  if (discovery_state != DiscoveryState::DONE)
    discovery_tick();

  return false;
}

void Bus::discovery_begin(const bool _enumerate_params)
{
  printf("Bus::discovery_begin(%s)\n", _enumerate_params ? "true" : "false");

  discovery_time = ros::Time::now();
  discovery_state = DiscoveryState::PROBING;
  discovery_attempt = 0;
  enumerate_params = _enumerate_params;

  // default args create a random response time between 0 and 100 milliseconds
  send_packet(std::make_unique<Discovery>(500));
}

void Bus::discovery_tick()
{
  const double elapsed = (ros::Time::now() - discovery_time).toSec();
  switch (discovery_state)
  {
    case DiscoveryState::PROBING:
      // send out broadcast discovery requests to retrieve drive UUID's
      if (elapsed > 0.5)
      {
        discovery_attempt++;
        if (discovery_attempt < 3)
        {
          discovery_time = ros::Time::now();
          send_packet(std::make_unique<Discovery>(500));
        }
        else
        {
          if (enumerate_params)
          {
            discovery_state = DiscoveryState::NUM_PARAMS;
            discovery_drive_idx = 0;
            discovery_attempt = 0;
          }
          else
          {
            discovery_state = DiscoveryState::DONE;
            ROS_INFO("discovery complete");
          }
        }
      }
      break;

    case DiscoveryState::NUM_PARAMS:
      if (elapsed > 0.1)
      {
        printf("param enumeration\n");
        if (discovery_drive_idx >= drives.size())
        {
          if (discovery_drive_idx > 0)
            discovery_state = DiscoveryState::PARAMS;
          else
            discovery_state = DiscoveryState::DONE;

          discovery_attempt = 0;
          discovery_drive_idx = 0;
          discovery_param_idx = 0;
          break;
        }

        auto& drive = drives[discovery_drive_idx];
        if (drive->is_bootloader)
        {
          discovery_drive_idx++;
          discovery_attempt = 0;
          break;
        }

        if (drive->num_params != 0)
        {
          discovery_drive_idx++;
          discovery_attempt = 0;
          break;
        }

        if (discovery_attempt > 3)
        {
          ROS_ERROR(
              "no response to num_params of drive %d",
              static_cast<int>(discovery_drive_idx));
          discovery_drive_idx++;
          break;
        }

        send_packet(std::make_unique<NumParams>(*drive));
        discovery_time = ros::Time::now();
        discovery_attempt++;
      }
      break;

    case DiscoveryState::PARAMS:
      if (elapsed > 0.1)
      {
        if (discovery_drive_idx >= drives.size() ||
            discovery_param_idx >= drives[discovery_drive_idx]->params.size())
        {
          if (drives[discovery_drive_idx]->is_bootloader)
            ROS_WARN("found drive in bootloader mode. Please click 'Boot'");
          else
            ROS_ERROR("WOAH invalid drive or param idx in discovery");
          discovery_state = DiscoveryState::DONE;
          break;
        }

        const Drive& drive = *drives[discovery_drive_idx];
        if (drive.params[discovery_param_idx].is_valid())
        {
          discovery_param_idx++;
          if (discovery_param_idx >= drive.params.size())
          {
            discovery_drive_idx++;
            discovery_param_idx = 0;
            if (discovery_drive_idx >= drives.size())
            {
              discovery_state = DiscoveryState::DONE;
              ROS_INFO("discovery complete");
              break;
            }
          }
        }
        //if (drives[discovery_drive_idx].params[
        send_packet(
            std::make_unique<ParamNameValue>(
                drive,
                discovery_param_idx));
        discovery_time = ros::Time::now();
        break;
      }
      break;

      default:
        break;
  }
}

bool Bus::burn_firmware(const std::string& firmware_filename)
{
  for (auto drive : drives)
    if (!burn_firmware(*drive, firmware_filename))
      return false;
  return true;
}

bool Bus::burn_firmware(Drive& drive, const std::string& firmware_filename)
{
  ROS_INFO(
      "burning firmware [%s] to drive %s",
      firmware_filename.c_str(),
      drive.uuid.to_string().c_str());

  FILE *f = fopen(firmware_filename.c_str(), "r");
  if (!f)
  {
    ROS_FATAL("couldn't open firmware: [%s]", firmware_filename.c_str());
    return false;
  }

  ROS_INFO("resetting drive %s", drive.uuid.to_string().c_str());
  send_packet(std::make_unique<Reset>(drive));
  ros::Duration(1.0).sleep();
  ROS_INFO("done resetting %s", drive.uuid.to_string().c_str());

  ROS_INFO("reading image...");

  bool burn_needed = false;

  // first make sure we actually need to do this: read back flash image
  // and bail the first time we see something is not looking right
  const int CHUNK_LEN = 128;
  for (int addr = 0x08020000;
      !burn_needed && !feof(f) && addr < 0x08080000;
      addr += CHUNK_LEN)
  {
    uint8_t chunk[CHUNK_LEN] = {0};
    size_t nread = fread(chunk, 1, CHUNK_LEN, f);
    if (nread < CHUNK_LEN)
      printf("last read: only got %d bytes from file\n", (int)nread);

    // read this address from the MCU flash

    send_packet(std::make_unique<FlashRead>(drive, addr, CHUNK_LEN));
    if (!wait_for_packet(1.0, Packet::ID_FLASH_READ))
    {
      ROS_ERROR(
          "couldn't read drive %s addr 0x%08x",
          drive.uuid.to_string().c_str(),
          static_cast<unsigned>(addr));
      return false;
    }

    // check that we received the expected chunk
    if (drive.flash_last_addr != addr)
    {
      ROS_ERROR("woah! didn't seem to read the requested address!");
      return false;
    }

    if (drive.flash_last_read.size() != CHUNK_LEN)
    {
      ROS_ERROR("woah! unexpected read size");
      return false;
    }

    for (int i = 0; i < CHUNK_LEN; i++)
    {
      if (chunk[i] != drive.flash_last_read[i])
      {
        ROS_INFO("mismatch at address 0x%08x", addr + i);
        burn_needed = true;
        break;
      }
    }
  }

  if (!burn_needed)
  {
    ROS_INFO("image verified successfully; not burning it.");
    return true;
  }

  ROS_INFO("proceeding with burn...");
  rewind(f);

  // highest flash address: 0x0807_ffff (end of page 127)

  for (int addr = 0x08020000;
      !feof(f) && addr < 0x08080000;
      addr += CHUNK_LEN)
  {
    ROS_INFO("burning chunk at addr 0x%08x", static_cast<unsigned>(addr));

    uint8_t chunk[CHUNK_LEN] = {0};
    size_t nread = fread(chunk, 1, CHUNK_LEN, f);
    if (nread < CHUNK_LEN)
      printf("last read: only got %d bytes from file\n", (int)nread);

    // erasing page is triggered in MCU bootloader when writing the first
    // chunk of a page

    std::vector<uint8_t> chunk_vec;
    for (int i = 0; i < CHUNK_LEN; i++)
      chunk_vec.push_back(chunk[i]);
    send_packet(std::make_unique<FlashWrite>(drive, addr, chunk_vec));

    if (!wait_for_packet(3.0, Packet::ID_FLASH_WRITE))
    {
      ROS_ERROR(
          "couldn't write drive %s addr 0x%08x",
          drive.uuid.to_string().c_str(),
          static_cast<unsigned>(addr));
      return false;
    }
  }
  ROS_INFO("burn complete");

  // todo: verify

  return true;
}

bool Bus::boot_all_drives()
{
  for (auto drive : drives)
  {
    printf("booting drive: %s\n", drive->uuid.to_string().c_str());
    send_packet(std::make_unique<Boot>(*drive));
    /*
    // todo: receive ACK from boot request, not sure... ?
    if (!wait_for_packet(1.0, Packet::ID_BOOT))
    {
      ROS_ERROR("couldn't boot drive %s", drive->uuid.to_string().c_str());
      return false;
    }
    */
  }
  return true;
}

bool Bus::use_serial_transport(const std::string& device_name)
{
  auto transport = make_unique<TransportSerial>();
  if (!transport->open_device(device_name))
  {
    ROS_FATAL("couldn't open serial device");
    return false;
  }
  ROS_INFO("opened %s", device_name.c_str());
  set_transport(std::move(transport));
  return true;
}

bool Bus::use_multicast_transport()
{
  ROS_INFO("opening multicast transport...");
  auto transport = make_unique<TransportMulticast>();
  if (!transport->init())
  {
    ROS_FATAL("couldn't init multicast transport");
    return false;
  }
  set_transport(std::move(transport));
  return true;
}
