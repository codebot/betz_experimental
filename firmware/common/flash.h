#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>

typedef enum { FLASH_SUCCESS, FLASH_FAIL } flash_result_t;

flash_result_t flash_erase_sector(const uint_fast8_t sector);
flash_result_t flash_erase_block_by_addr(const uint32_t *addr);
flash_result_t flash_program_word(const uint32_t *addr, const uint32_t data);
flash_result_t flash_program_byte(const uint8_t *addr, const uint8_t data);
flash_result_t flash_flush_d_cache();

#endif

