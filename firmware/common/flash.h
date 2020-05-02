#ifndef FLASH_H
#define FLASH_H

#include <stdbool.h>
#include <stdint.h>

#define FLASH_START 0x08000000

void flash_init();

void flash_read(
    const uint32_t read_addr,
    const uint32_t len,
    uint8_t *dest_addr);

bool flash_write(
    const uint32_t write_addr,
    const uint32_t write_len,
    const uint8_t *write_data);

#endif
