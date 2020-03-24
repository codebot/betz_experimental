#ifndef RS485_H
#define RS485_H

#include <stdint.h>
#include <stdbool.h>

void rs485_init();
void rs485_tick();

// this function uses the minimal 8-bit "ID parameter" in its header
void rs485_tx(
    const uint8_t *data,
    const uint32_t len);

// this function uses the full 12-byte UUID of the device in its header
void rs485_tx_long(
    const uint8_t *data,
    const uint32_t len);

#define RS485_LONG_ADDR_LEN 12
extern uint8_t g_rs485_id;

#endif
