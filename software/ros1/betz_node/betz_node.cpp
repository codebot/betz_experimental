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

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

#include "betz_node.h"
#include "bus.h"
#include "drive.h"
#include "transport_serial.h"
#include "transport_multicast.h"

using betz::BetzNode;
using std::string;
using std::make_unique;


BetzNode::BetzNode()
: nh(), nh_private("~")
{
  /*
  ros::ServiceServer int_param_srv =
      nh.advertiseService("set_int_param", set_int_param);
  */
}

BetzNode::~BetzNode()
{
}

/*
bool BetzNode::set_int_param(
    betz_node::SetIntParam::Request &request,
    betz_node::SetIntParam::Response &response)
{
  const int param_name_len = static_cast<int>(request.name.size());
  if (param_name_len > 250)
  {
    ROS_ERROR("param name len is too long: %s", request.name.c_str());
    return false;
  }
  ROS_INFO("set_int_param(%s, %d)",
      request.name.c_str(), request.value);
  uint8_t pkt[256] = {0};
  pkt[0] = 4;  // packet type 4 = set int param by name
  memcpy(&pkt[1], &request.value, 4);
  strcpy((char *)&pkt[5], request.name.c_str());
  return send_pkt(pkt, 6 + param_name_len);
}

bool get_int_param(betz_node::GetIntParam::Request &request,
    betz_node::GetIntParam::Response &response)
{
  int param_name_len = static_cast<int>(request.name.size());
  if (param_name_len > 250)
  {
    ROS_ERROR("param name len is too long: %s", request.name.c_str());
    return false;
  }
  uint8_t pkt[256] = {0};
  pkt[0] = 3;  // packet type 3 = get int param by name
  strcpy((char *)&pkt[1], request.name.c_str());
  if (!send_pkt(pkt, 2 + param_name_len))
    return false;
  if (!rs485_spin(0.5, 3))
  {
    ROS_ERROR("didn't hear back from MCU for request for parameter [%s]",
        request.name.c_str());
    return false;
  }
  // todo: actually copy the returned value into the response message
  ROS_INFO("get_int_param(%s) = %d", request.name.c_str(), response.value);
  return true;
}
*/

/*
static void rs485_rx_pkt(const uint32_t len, const uint8_t *data)
{
  if (g_betz)
    g_betz->rs485_rx_pkt(len, data);
}
*/

int BetzNode::init(int argc, char **argv)
{
  if (argc <= 1)
    return usage();

  string transport_name;
  nh_private.param<std::string>("transport", transport_name, "rs485");

  if (transport_name == "rs485")
  {
    string device_name;
    nh_private.param<std::string>("device", device_name, "/dev/ttyUSB0");
  
    auto transport = make_unique<TransportSerial>();
    if (!transport->open_device(device_name))
    {
      ROS_FATAL("couldn't open serial device");
      return 1;
    }
    ROS_INFO("opened %s", device_name.c_str());
    bus.set_transport(std::move(transport));
  }
  else if (transport_name == "multicast")
  {
    ROS_INFO("opening multicast transport...");
    auto transport = make_unique<TransportMulticast>();
    if (!transport->init())
    {
      ROS_FATAL("couldn't init multicast transport");
      return 1;
    }
    bus.set_transport(std::move(transport));
  }
  else
  {
    ROS_FATAL("unknown transport name: [%s]", transport_name.c_str());
    return 1;
  }

  bus.discovery_begin();

  ros::Rate rate(100);
  while (ros::ok() && !bus.discovery_complete)
  {
    bus.spin_once();
    rate.sleep();
  }

  ROS_INFO("discovery complete. found %zu drives.", bus.drives.size());
  
  const string verb(argv[1]);
  if (verb == "run")
    return run();
  else if (verb == "burn_firmware")
  {
    if (argc <= 2)
    {
      ROS_FATAL("syntax: betz_node burn_firmware FILENAME");
      return 1;
    }
    return burn_firmware(string(argv[2]));
  }
  else
    return usage();
}

int BetzNode::run()
{
  ROS_INFO("run()");
  // no harm in asking an already-running drive to boot; it just ignores it
  if (!bus.boot_all_drives())
  {
    ROS_FATAL("unable to boot all drives");
    return 1;
  }
  // todo: enumerate and stuff
  return 0;
}

int BetzNode::burn_firmware(const string& filename)
{
  ROS_INFO("burning firmware...");
  if (!bus.burn_firmware(filename))
  {
    ROS_FATAL("error burning firmware");
    return 1;
  }
  else
  {
    ROS_INFO("firmware burn completed");
    return 0;
  }
}

  /*
  const int num_params = betz.get_num_params();
  ROS_INFO("device has %d params", num_params);

  const bool read_ok = betz.read_flash(0x08000000, 128);
  ROS_INFO("read_flash(0x0, 128) = %d", read_ok ? 1 : 0);
  */

int BetzNode::usage()
{
  ROS_FATAL("no verb supplied!\n\nvalid verbs: run, reset, burn_firmware");
  return 1;
}
