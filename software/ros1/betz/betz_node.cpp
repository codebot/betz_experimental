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
#include "betz/set_position_target.h"

using betz::BetzNode;
using std::string;
using std::make_unique;
using std::shared_ptr;


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
  else if (verb == "calibrate")
    return calibrate();
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

  return 0;
}

void BetzNode::rx_packet(const Packet& packet)
{
  if (packet.payload.size() == 0)
    return;

  const uint8_t packet_id = packet.payload[0];
  switch (packet_id)
  {
    case Packet::ID_STATE_POLL: rx_state_poll(packet); break;
    default: break;
  }
}

void BetzNode::rx_state_poll(const Packet& packet)
{
  if (packet.payload.size() < 36)
  {
    ROS_ERROR("state packet too short: %d", (int)packet.payload.size());
    return;
  }

  int32_t t = 0;
  memcpy(&t, &packet.payload[4], 4);

  float enc = 0;
  memcpy(&enc, &packet.payload[8], 4);

  float joint_pos = 0;
  memcpy(&joint_pos, &packet.payload[12], 4);

  float phase_currents[3] = {0};
  memcpy(&phase_currents[0], &packet.payload[16], 4);
  memcpy(&phase_currents[1], &packet.payload[20], 4);
  memcpy(&phase_currents[2], &packet.payload[24], 4);

  float joint_vel = 0;
  memcpy(&joint_vel, &packet.payload[28], 4);

  float joint_effort = 0;  // quadrature voltage
  memcpy(&joint_effort, &packet.payload[32], 4);

  if (calibration_log)
  {
    fprintf(
        calibration_log,
        "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.8f,%.6f\n",
        t,
        enc,
        joint_pos,
        phase_currents[0],
        phase_currents[1],
        phase_currents[2],
        joint_vel,
        joint_effort);
  }
}

int BetzNode::calibrate()
{
  const int joint_id = 1;
  // make sure the joint enumerated
  shared_ptr<Drive> drive = bus.drive_by_id(static_cast<uint8_t>(joint_id));
  if (!drive)
  {
    ROS_FATAL("couldn't find joint %d", joint_id);
    return 1;
  }
  ROS_INFO("found drive %d", joint_id);

  const std::string calibration_filename("calibration_data.csv");
  calibration_log = fopen(calibration_filename.c_str(), "w");
  if (!calibration_log)
  {
    ROS_FATAL(
        "couldn't open calibration log: [%s]",
        calibration_filename.c_str());
    return 1;
  }
  ROS_INFO("opened %s", calibration_filename.c_str());

  bus.packet_listener =
      [this](const Packet& p) { this->rx_packet(p); };

  // first ramp up the position gain smoothly to avoid a freak-out
  const float gain_target = -10.0f;
  ros::Duration gain_ramp_time(2.0);
  ros::Rate serial_poll_rate(100);
  ros::Time last_poll_time(ros::Time::now());
  ros::Time t_start(ros::Time::now());

  ROS_INFO("engaging position controller");
  drive->set_param(bus, "position_kp", 0.0f);  // we'll ramp it gracefully
  drive->set_param(bus, "control_mode", 2);  // position control

  while (ros::ok())
  {
    const ros::Time t(ros::Time::now());
    if ((t - t_start) > gain_ramp_time)
      break;

    if (t > last_poll_time + ros::Duration(0.01))
    {
      last_poll_time = t;

      const float ramp_frac =
          (t - t_start).toSec() / gain_ramp_time.toSec();
      const float gain = ramp_frac * gain_target;
      ROS_INFO("setting gain to %.3f", gain);
      drive->set_param(bus, "position_kp", gain);
    }
    bus.spin_once();
    ros::spinOnce();
    serial_poll_rate.sleep();
  }

  drive->set_param(bus, "position_kp", gain_target);

  // now we'll cycle to each joint position and measure the holding torque
  // required to reach it
  for (double joint_pos = 0; joint_pos < 2.0 * M_PI; joint_pos += 0.01)
  {
    t_start = ros::Time::now();
    const ros::Time t(t_start);
    
    // send twice to make really sure it gets there
    bus.send_packet(
        make_unique<betz::SetPositionTarget>(*drive, joint_pos));
    usleep(10000);
    bus.send_packet(
        make_unique<betz::SetPositionTarget>(*drive, joint_pos));

    while (ros::ok() && (t - t_start).toSec() < 1.0)
    {
      bus.spin_once();
      ros::spinOnce();
      serial_poll_rate.sleep();
    }
  }

  ROS_INFO("return drive to idle mode");
  drive->set_param(bus, "position_kp", 0.0f);
  drive->set_param(bus, "control_mode", 0);
  return 0;
}

int BetzNode::usage()
{
  ROS_FATAL("no verb supplied!\n\nvalid verbs:\n"
      "  run\nreset\nburn_firmware\ncalibrate");
  return 1;
}