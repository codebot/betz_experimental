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
#include <stdint.h>
#include <string>
#include <vector>

#include "drive.h"
#include "bus.h"
#include "betz_node.h"
using betz::BetzNode;
using std::string;


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

void BetzNode::run()
{
  string rs485_device;
  nh_private.param<std::string>(
      "rs485_device",
      rs485_device,
      "/dev/ttyUSB0");
  if (!bus.open_serial_device(rs485_device))
  {
    ROS_FATAL("could not open device");
    return;
  }

  /*
  const int num_params = betz.get_num_params();
  ROS_INFO("device has %d params", num_params);

  const bool read_ok = betz.read_flash(0x08000000, 128);
  ROS_INFO("read_flash(0x0, 128) = %d", read_ok ? 1 : 0);
  */
  /*
  ROS_INFO("entering slow_bldc spin loop");
  betz.rs485_spin(-1);  // spin forever. wooooahhh that makes me feel dizzy
  */
}
