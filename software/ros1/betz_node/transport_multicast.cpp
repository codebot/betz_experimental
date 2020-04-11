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

#include <ifaddrs.h>

#include <cstring>

#include "transport_multicast.h"

using namespace betz;


TransportMulticast::TransportMulticast()
{
  multicast_group = 0xe0000042;
  multicast_port = 12345;
}

bool TransportMulticast::init()
{
  struct ifaddrs *ifaddr;
  if (getifaddrs(&ifaddr) == -1)
  {
    printf("couldn't call getifaddrs\n");
    return false;
  }

  char *tx_addr_str = NULL;

  for (struct ifaddrs *ifa = ifaddr; ifa; ifa = ifa->ifa_next)
  {
    if (!ifa->ifa_addr)
      continue;
    int family = ifa->ifa_addr->sa_family;
    if (family != AF_INET)
      continue;
    char host[NI_MAXHOST];
    if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                    host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST))
      continue;
    printf("found address %s on interface %s\n", host, ifa->ifa_name);
    if (0 == strcmp(host, "127.0.0.1"))
      continue; // boring
    tx_addr_str = host; // save this one for now
  }
  printf("using interface: %s\n", tx_addr_str);
  tx_addr.sin_addr.s_addr = inet_addr(tx_addr_str);
  freeifaddrs(ifaddr);

  tx_sock = socket(AF_INET, SOCK_DGRAM, 0);

  int result;
  result = setsockopt(
      tx_sock,
      IPPROTO_IP,
      IP_MULTICAST_IF,
      (char *)&tx_addr.sin_addr.s_addr,
      sizeof(tx_addr));
  if (result < 0)
  {
    printf("couldn't set tx socket for multicast\n");
    return false;
  }

  int loopback = 1;
  result = setsockopt(
      tx_sock,
      IPPROTO_IP,
      IP_MULTICAST_LOOP,
      &loopback,
      sizeof(loopback));
  if (result < 0)
  {
    printf("ERROR: couldn't set tx socket for tx multicast loopback\n");
    return false;
  }

  rx_sock = socket(AF_INET, SOCK_DGRAM, 0);

  int one = 1;
  result = setsockopt(
      rx_sock,
      SOL_SOCKET,
      SO_REUSEPORT,
      &one,
      sizeof(one));
  if (result < 0)
  {
    printf("ERROR: couldn't set SO_REUSEPORT on rx sock\n");
    return false;
  }

  result = setsockopt(
      rx_sock,
      SOL_SOCKET,
      SO_REUSEADDR,
      &one,
      sizeof(one));
  if (result < 0)
  {
    printf("ERROR: couldn't set SO_REUSEADDR on rx sock\n");
    return false;
  }

  struct sockaddr_in bind_addr;
  memset(&bind_addr, 0, sizeof(bind_addr));
  bind_addr.sin_family = AF_INET;
  bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  bind_addr.sin_port = htons(multicast_port);
  result = bind(
      rx_sock,
      (struct sockaddr *)&bind_addr,
      sizeof(bind_addr));
  if (result < 0)
  {
    printf("ERROR: couldn't bind to multicast group\n");
    return false;
  }

  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = multicast_group;
  mreq.imr_interface.s_addr = tx_addr.sin_addr.s_addr;
  result = setsockopt(
      rx_sock,
      IPPROTO_IP,
      IP_ADD_MEMBERSHIP,
      &mreq,
      sizeof(mreq));
  if (result < 0)
  {
    printf("ERROR: couldn't add rx sock to multicast group\n");
    return false;
  }

  return true;
}

bool TransportMulticast::send(const uint8_t *data, const uint32_t len)
{
  tx_addr.sin_addr.s_addr = multicast_group;
  tx_addr.sin_port = htons(multicast_port);

  int nsent = sendto(
      tx_sock,
      data,
      len,
      0,
      (const struct sockaddr *)&tx_addr,
      sizeof(tx_addr));
  if (nsent <= 0)
  {
    printf("ERROR: couldn't send: %s\n", strerror(errno));
    return false;
  }

  return true;
}

int TransportMulticast::recv_nonblocking(uint8_t *data, const uint32_t max_len)
{
  fd_set rdset;
  FD_ZERO(&rdset);
  int max_fd = rx_sock;

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  int rv = select(max_fd + 1, &rdset, NULL, NULL, &timeout);
  if (rv < 0)
    return rv;
  else if (rv > 0)
  {
    char buf[1024];
    int nbytes = recvfrom(
        rx_sock,
        buf,
        sizeof(buf),
        0,
        NULL,
        0);

    printf("received %d-byte packet, hooray\n", nbytes);
    return nbytes;
  }
  return 0;
}
