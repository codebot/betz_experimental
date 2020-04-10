#ifndef RS485_H
#define RS485_H

#include <stdint.h>
#include <stdbool.h>

void rs485_init();
void rs485_tick();
void rs485_enable_termination(const bool enable);
void rs485_tx(const uint8_t *data, const uint32_t len);

#endif
