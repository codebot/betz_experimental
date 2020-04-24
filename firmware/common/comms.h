#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>

void comms_init();
void comms_tick();
void comms_rx_byte(const uint8_t byte);
void comms_set_raw_tx_fptr(void (*fptr)(const uint8_t *, const uint32_t));

#endif
