#ifndef MULTICAST_H
#define MULTICAST_H

#include <stdbool.h>

bool multicast_init();
void multicast_tx(const uint8_t *data, const uint32_t len);
void multicast_listen(const uint32_t max_usec);

#endif
