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
#include "betz/bus.h"
#include "betz/drive.h"

using betz::BetzNode;
using std::string;
using std::make_unique;


BetzNode::BetzNode()
: nh(), nh_private("~")
{
}

BetzNode::~BetzNode()
{
}

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
    if (!bus.use_serial_transport(device_name))
      return 1;
  }
  else if (transport_name == "multicast")
  {
    if (!bus.use_multicast_transport())
      return 1;
  }
  else
  {
    ROS_FATAL("unknown transport name: [%s]", transport_name.c_str());
    return 1;
  }

  bus.discovery_begin(false);

  ros::Rate rate(100);
  while (ros::ok() && (bus.discovery_state < Bus::DiscoveryState::NUM_PARAMS))
  {
    bus.spin_once();
    rate.sleep();
  }

  ROS_INFO("discovery complete. found %zu drives.", bus.drives.size());

  const string verb(argv[1]);
  if (verb == "run")
    return run();
  else if (verb == "discover")
    return discover();
  else if (verb == "burn_firmware")
  {
    if (argc <= 2)
    {
      ROS_FATAL("syntax: betz_node burn_firmware FILENAME ID");
      return 1;
    }
    int id = -1;
    if (argc > 3)
      id = atoi(argv[3]);
    return burn_firmware(string(argv[2]), id);
  }
  else
    return usage();
}

int BetzNode::discover()
{
  return 0;  // nothing else to do
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

int BetzNode::burn_firmware(const string& filename, const int id)
{
  ROS_INFO("burning firmware...");
  if (!bus.burn_firmware(filename, id))
  {
    ROS_FATAL("error burning firmware");
    return 1;
  }
  else
    ROS_INFO("firmware burn completed");

  if (!bus.boot_all_drives())
  {
    ROS_FATAL("unable to boot all drives");
    return 1;
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
