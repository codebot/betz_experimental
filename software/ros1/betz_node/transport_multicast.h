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

#ifndef TRANSPORT_MULTICAST_H
#define TRANSPORT_MULTICAST_H

#include <memory>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <netinet/in.h>

#include "transport.h"

namespace betz {

class TransportMulticast : public Transport
{
public:
  TransportMulticast();

  bool init();
  bool send(const uint8_t *data, const uint32_t len) override;
  int recv_nonblocking(uint8_t *data, const uint32_t max_len) override;

private:
  struct sockaddr_in tx_addr;
  int tx_sock = 0;
  int rx_sock = 0;
  int multicast_port = 0;
  in_addr_t multicast_group;
};

}  // namespace betz

#endif
