#ifndef FLASH_H
#define FLASH_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { FLASH_SUCCESS, FLASH_FAIL } flash_result_t;

void flash_init();
uint32_t flash_size();  // returns flash size in bytes
uint32_t flash_page_size();  // return flash page size in bytes
flash_result_t flash_erase_sector(const uint_fast8_t sector);
flash_result_t flash_erase_block_by_addr(const uint32_t addr);
flash_result_t flash_program_word(const uint32_t addr, const uint32_t data);
flash_result_t flash_program_byte(const uint32_t addr, const uint8_t data);
flash_result_t flash_flush_d_cache();

void flash_read(
    const uint32_t read_addr,
    const uint32_t len,
    uint8_t *dest_addr);

bool flash_write(
    const uint32_t write_addr,
    const uint32_t write_len,
    const uint8_t *write_data);

bool flash_erase_page_by_addr(const uint32_t addr);

#endif
