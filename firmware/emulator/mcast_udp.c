#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>

#include "mcast_udp.h"

static struct sockaddr_in g_tx_addr;
static int g_tx_sock;
static int g_rx_sock;

static const int g_mcast_udp_port = 12345;
static in_addr_t g_mcast_group;

//////////////////////////////////////////////////////////////////////

void mcast_udp_init()
{
  printf("mcast_udp_init()\n");
  g_mcast_group = htonl(0xe0000042);
  g_tx_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
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
    return;
  }

  int loopback = 1;
  result = setsockopt(
      g_tx_sock,
      IPPROTO_IP,
      IP_MULTICAST_LOOP,
      &loopback,
      sizeof(loopback));
  if (result < 0)
    printf("ERROR: couldn't set tx socket for tx multicast loopback\n");

  g_rx_sock = socket(AF_INET, SOCK_DGRAM, 0);

  int one = 1;
  result = setsockopt(
      g_rx_sock,
      SOL_SOCKET,
      SO_REUSEPORT,
      &one,
      sizeof(one));
  if (result < 0)
    printf("ERROR: couldn't set SO_REUSEPORT on rx sock\n");
  
  result = setsockopt(
      g_rx_sock,
      SOL_SOCKET,
      SO_REUSEADDR,
      &one,
      sizeof(one));
  if (result < 0)
    printf("ERROR: couldn't set SO_REUSEADDR on rx sock\n");

  struct sockaddr_in bind_addr;
  memset(&bind_addr, 0, sizeof(bind_addr));
  bind_addr.sin_family = AF_INET;
  bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  bind_addr.sin_port = htons(g_mcast_udp_port);
  result = bind(g_rx_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
  if (result < 0)
    printf("ERROR: couldn't bind to mcast port\n");

  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = g_mcast_group;
  mreq.imr_interface.s_addr = g_tx_addr.sin_addr.s_addr;
  result = setsockopt(
      g_rx_sock,
      IPPROTO_IP,
      IP_ADD_MEMBERSHIP,
      &mreq,
      sizeof(mreq));
  if (result < 0)
    printf("ERROR: couldn't add rx sock to multicast group\n");
}

void mcast_udp_tx(const uint8_t *data, const uint32_t len)
{
  printf("mcast_tx(%x)\n", len);
  int nsent = sendto(
      g_rx_sock,
      data,
      len,
      0,
      (struct sockaddr *)&g_tx_addr,
      sizeof(g_tx_addr));
  if (nsent <= 0)
    printf("ERROR: couldn't send: %s\n");
    perror("error");
}

void mcast_udp_listen(const uint32_t max_usec)
{
  fd_set rdset;
  FD_ZERO(&rdset);
  int max_fd = g_rx_sock;

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = max_usec;

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

    printf("received %d-byte packet, hooray\n", nbytes);
  }
}
