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

#ifndef BUS_H
#define BUS_H

#include <memory>
#include <string>
#include <stdint.h>

#include <ros/time.h>

#include "packet/crc.h"
#include "drive.h"
#include "packet/packet.h"
#include "transport.h"


namespace betz {

class Bus
{
public:
  std::unique_ptr<Transport> transport;
  std::vector<std::shared_ptr<Drive>> drives;

  enum class ParserState
  {
    PREAMBLE,
    FLAGS,
    ADDRESS,
    LENGTH,
    PAYLOAD,
    CSUM_0,
    CSUM_1
  };

  ParserState parser_state = ParserState::PREAMBLE;
  CRC parser_crc;
  Packet parser_packet;

  ////////////////////////////////////////////////////////////////////

  Bus();
  ~Bus();

  bool spin_once(const uint8_t watch_packet_id = 0);

  void set_transport(std::unique_ptr<Transport> transport);

  bool send_packet(std::unique_ptr<Packet> packet);

  bool wait_for_packet(const double max_seconds, const uint8_t packet_id);

  bool rx_byte(const uint8_t b, Packet& rx_pkt);
  void rx_packet(Packet& packet);

  bool read_flash(
      const uint8_t drive_id,
      const uint32_t start_addr,
      const uint32_t len);

  bool set_led(const uint8_t drive_id, const bool on);

  int get_num_params(const uint8_t drive_id);

  // void add_drive_id(const uint8_t drive_id);
  //Drive *find_drive_by_id(const uint8_t drive_id);

  std::shared_ptr<Drive> drive_by_uuid(const std::vector<uint8_t>& uuid);

  enum class DiscoveryState
  {
    PROBING,
    COMPLETE
  };
  DiscoveryState discovery_state = DiscoveryState::PROBING;

  void discovery_begin();
  void discovery_tick();
  bool discovery_complete = false;
  int discovery_broadcast_count = 0;
  ros::Time discovery_time;

  void enumeration_tick();

  bool burn_firmware(const std::string& firmware_filename);

private:
  uint8_t rx_buf[4096] = {0};
  bool burn_firmware(Drive& drive, const std::string& firmware_filename);
};

}  // namespace betz

#endif
