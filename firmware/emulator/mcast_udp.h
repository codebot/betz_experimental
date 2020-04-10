#ifndef MCAST_UDP_H
#define MCAST_UDP_H

#include <stdbool.h>

bool mcast_udp_init();
void mcast_udp_tx(const uint8_t *data, const uint32_t len);
void mcast_udp_listen(const uint32_t max_usec);

#endif
