#ifndef FLASH_H
#define FLASH_H

#include <stdbool.h>
#include <stdint.h>

#define FLASH_START 0x08000000

void flash_init();

void flash_read(
    const uint32_t read_addr,
    const uint32_t len,
    void *dest_addr);

bool flash_write(
    const uint32_t write_addr,
    const uint32_t write_len,
    const uint8_t *write_data);

uint32_t flash_get_param_table_base_addr();
bool flash_erase_page_by_addr(const uint32_t addr);
bool flash_program_word(const uint32_t addr, const uint32_t data);
bool flash_program_byte(const uint32_t addr, const uint8_t byte);

uint32_t flash_read_word(const uint32_t addr);
uint8_t flash_read_byte(const uint32_t addr);

#endif
