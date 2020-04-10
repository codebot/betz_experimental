#include <stdio.h>
#include <string.h>

#include "flash.h"


#define NUM_SECTORS 12

flash_result_t flash_erase_sector(const uint_fast8_t sector)
{
  printf("flash_erase_sector(%d)\n", (int)sector);
  return FLASH_SUCCESS;
}

flash_result_t flash_erase_block_by_addr(const uint32_t addr)
{
  printf("flash_erase_block_by_addr(0x%08x)\n", addr);
  if (addr <  0x08020000 ||
      addr >  0x080fffff ||
      (addr & 0x1ffff) != 0)
    return FLASH_FAIL; // bad address
  int offset = addr - 0x08020000;
  int block = 5 + offset / 0x20000;
  return flash_erase_sector(block);
}

flash_result_t flash_program_word(const uint32_t addr, const uint32_t data)
{
  printf("flash_program_word(0x%08x, %08x)\n", addr, data);
  return FLASH_SUCCESS;
}

flash_result_t flash_program_byte(const uint32_t addr, const uint8_t data)
{
  printf("flash_program_byte(0x%08x, %02x)\n", addr, data);
  return FLASH_SUCCESS;
}

void flash_read(
    const uint32_t read_addr,
    const uint32_t len,
    uint8_t *dest_addr)
{
  printf("flash_read(0x%08x, %x)\n", read_addr, len);
}
