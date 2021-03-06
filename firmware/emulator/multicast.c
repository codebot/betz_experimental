#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <arpa/inet.h>
#include <errno.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <netinet/in.h>

#include "multicast.h"
#include "comms.h"

static struct sockaddr_in g_tx_addr;
static int g_tx_sock;
static int g_rx_sock;

static const int g_multicast_port = 12345;
static in_addr_t g_multicast_group;

//////////////////////////////////////////////////////////////////////

bool multicast_init()
{
  printf("multicast_init()\n");
  g_multicast_group = htonl(0xe0000042);

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
  if (!tx_addr_str)
  {
    printf("woah! no IP address found. cannot proceed :(\n");
    return false;
  }

  printf("using interface: %s\n", tx_addr_str);
  memset(&g_tx_addr, 0, sizeof(g_tx_addr));
  g_tx_addr.sin_addr.s_addr = inet_addr(tx_addr_str);
  freeifaddrs(ifaddr);

  g_tx_sock = socket(AF_INET, SOCK_DGRAM, 0);

  int result;
  result = setsockopt(
      g_tx_sock,
      IPPROTO_IP,
      IP_MULTICAST_IF,
      (char *)&g_tx_addr.sin_addr.s_addr,
      sizeof(g_tx_addr));
  if (result < 0)
  {
    printf("couldn't set tx socket for multicast\n");
    return false;
  }

  int loopback = 1;
  result = setsockopt(
      g_tx_sock,
      IPPROTO_IP,
      IP_MULTICAST_LOOP,
      &loopback,
      sizeof(loopback));
  if (result < 0)
  {
    printf("ERROR: couldn't set tx socket for tx multicast loopback\n");
    return false;
  }

  g_rx_sock = socket(AF_INET, SOCK_DGRAM, 0);

  int one = 1;
  result = setsockopt(
      g_rx_sock,
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
      g_rx_sock,
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
  bind_addr.sin_port = htons(g_multicast_port);
  result = bind(
      g_rx_sock,
      (struct sockaddr *)&bind_addr,
      sizeof(bind_addr));
  if (result < 0)
  {
    printf("ERROR: couldn't bind to multicast group\n");
    return false;
  }

  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = g_multicast_group;
  mreq.imr_interface.s_addr = g_tx_addr.sin_addr.s_addr;
  result = setsockopt(
      g_rx_sock,
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

void multicast_tx(const uint8_t *data, const uint32_t len)
{
  g_tx_addr.sin_addr.s_addr = g_multicast_group;
  g_tx_addr.sin_port = htons(g_multicast_port);

  int nsent = sendto(
      g_tx_sock,
      data,
      len,
      0,
      (const struct sockaddr *)&g_tx_addr,
      sizeof(g_tx_addr));
  if (nsent <= 0)
    printf("ERROR: couldn't send: %s\n", strerror(errno));
}

void multicast_listen(const uint32_t max_usec)
{
  fd_set rdset;
  FD_ZERO(&rdset);
  FD_SET(g_rx_sock, &rdset);
  int max_fd = g_rx_sock;

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = max_usec;

  while (true)
  {
    int rv = select(max_fd + 1, &rdset, NULL, NULL, &timeout);
    if (rv < 0)
      return;
    else if (rv > 0)
    {
      char buf[1024];
      int nbytes = recvfrom(
          g_rx_sock,
          buf,
          sizeof(buf),
          0,
          NULL,
          0);
  
      //printf("received %d-byte packet, hooray\n", nbytes);
      for (int i = 0; i < nbytes; i++)
        comms_rx_byte(buf[i]);
    }
    else // rv == 0
      break;
  }
}
