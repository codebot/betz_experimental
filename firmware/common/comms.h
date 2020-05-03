#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>

void comms_init(void (*tx_fptr)(const uint8_t *, const uint32_t));

void comms_tick();
void comms_rx_byte(const uint8_t byte);

void comms_set_bootloader_mode();

#endif
