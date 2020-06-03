/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <stdio.h>
#include <string.h>

#include "flash.h"


#define FLASH_LEN (512 * 1024)
#define FLASH_PAGE_SIZE 4096
#define FLASH_IMAGE_FILENAME "emulator_flash.bin"

static uint8_t flash_image[FLASH_LEN] = {0};
static bool flash_emulator_image_flush_to_disk();

#define FLASH_WRITE_BLOCK_SIZE 8
static uint8_t flash_write_buf[FLASH_WRITE_BLOCK_SIZE];
static uint32_t flash_write_buf_idx;
static uint32_t flash_write_addr;

void flash_init()
{
  printf("flash_init()\n");
  FILE *f = fopen(FLASH_IMAGE_FILENAME, "r");
  if (f)
  {
    if (FLASH_LEN != fread(flash_image, 1, FLASH_LEN, f))
      printf("AHHH couldn't read emulator flash image\n");
  }
  else
    printf("unable to open [%s]\n", FLASH_IMAGE_FILENAME);
}

void flash_read(
    const uint32_t read_addr,
    const uint32_t len,
    void *dest_addr)
{
  // printf("flash_read(0x%08x, %d)\n", read_addr, len);
  const int offset = read_addr - FLASH_START;
  if (offset > FLASH_LEN)
  {
    printf("AHHH invalid read address!!!\n");
    return;
  }
  memcpy(dest_addr, &flash_image[offset], len);
}

uint32_t flash_read_word(const uint32_t addr)
{
  uint32_t word = 0;
  flash_read(addr, 4, &word);
  return word;
}

uint8_t flash_read_byte(const uint32_t addr)
{
  uint8_t byte = 0;
  flash_read(addr, 1, &byte);
  return byte;
}

bool flash_write_block(
    const uint32_t write_addr,
    const uint32_t write_len,
    const uint8_t *write_data)
{
  printf("flash_write(0x%08x, %d)\n", write_addr, write_len);
  const int offset = write_addr - FLASH_START;
  if (offset + write_len > FLASH_LEN)
  {
    printf("AHHHH invalid write address!!!\n");
    return false;
  }
  for (int i = 0; i < write_len; i++)
    printf("%02d: 0x%02x\n", i, (unsigned)write_data[i]);

  memcpy(&flash_image[offset], write_data, write_len);

  return flash_emulator_image_flush_to_disk();
}

bool flash_emulator_image_flush_to_disk()
{
  // flush to disk
  FILE *f = fopen(FLASH_IMAGE_FILENAME, "w");
  if (!f)
  {
    printf("AHHHH couldn't open emulator flash image\n");
    return false;
  }
  if (FLASH_LEN != fwrite(flash_image, 1, FLASH_LEN, f))
  {
    printf("AHHHH error writing flash image to disk\n");
    return false;
  }
  fclose(f);
  return true;
}

bool flash_erase_page_by_addr(const uint32_t addr)
{
  printf("flash_erase_page_by_addr(0x%08x)\n", (unsigned)addr);

  // const int page_size = 0x20000;  // 128 KB flash page (stm32f4xx)
  const int page_size = 0x1000;  // 4 KB flash page (stm32g4)

  printf("erase page at 0x%08x\r\n", (unsigned)addr);
  const int offset = addr - FLASH_START;
  if (offset + page_size > FLASH_LEN)
  {
    printf("AHHHH invalid write address!!!\n");
    return false;
  }

  // todo: verify this addr is a page boundary

  memset(&flash_image[offset], 0xff, page_size);

  return flash_emulator_image_flush_to_disk();
}

uint32_t flash_get_param_table_base_addr()
{
  return FLASH_START + 3 * 128 * 1024;
}

uint32_t flash_get_param_table_size()
{
  return 128 * 1024;
}

#if 0
bool flash_program_word(const uint32_t addr, const uint32_t data)
{
  const int offset = addr - FLASH_START;
  if (offset + 4 > FLASH_LEN)
  {
    printf("AHHHH invalid write address!!!\n");
    return false;
  }
  memcpy(&flash_image[offset], &data, 4);
  return flash_emulator_image_flush_to_disk();
}

bool flash_program_byte(const uint32_t addr, const uint8_t data)
{
  const int offset = addr - FLASH_START;
  if (offset >= FLASH_LEN)
  {
    printf("AHHHH invalid write address!!!\n");
    return false;
  }
  flash_image[offset] = data;
  return flash_emulator_image_flush_to_disk();
}
#endif

bool flash_program_dword(
    const uint32_t addr,
    const uint32_t word_0,
    const uint32_t word_1)
{
}

bool flash_write_begin(const uint32_t addr)
{
  flash_write_buf_idx = 0;
  flash_write_addr = addr;
  return true;
}

static bool flash_write_flush()
{
  const bool result = flash_write_block(
      flash_write_addr,
      FLASH_WRITE_BLOCK_SIZE,
      flash_write_buf);

  flash_write_buf_idx = 0;
  flash_write_addr += FLASH_WRITE_BLOCK_SIZE;

  return result;
}

bool flash_write_byte(const uint8_t byte)
{
  if (flash_write_buf_idx >= FLASH_WRITE_BLOCK_SIZE)
  {
    printf("ahhhhh flash write buf overrun attempted!\r\n");
    return false;
  }
  flash_write_buf[flash_write_buf_idx++] = byte;
  if (flash_write_buf_idx >= FLASH_WRITE_BLOCK_SIZE)
    flash_write_flush();
  return true;
}

bool flash_write_word(const uint32_t word)
{
  const uint8_t * const p_bytes = (const uint8_t * const)&word;
  for (int byte_idx = 0; byte_idx < 4; byte_idx++)
  {
    if (!flash_write_byte(p_bytes[byte_idx]))
      return false;
  }
  return true;
}

bool flash_write_end()
{
  for (; flash_write_buf_idx < FLASH_WRITE_BLOCK_SIZE; flash_write_buf_idx++)
    flash_write_buf[flash_write_buf_idx] = 0x42;
  return flash_write_flush();
}

bool flash_erase_range(const uint32_t start, const uint32_t len)
{
  printf("flash_erase_range(0x%08x, %d)\r\n", (unsigned)start, (int)len);
  for (uint32_t addr = start; addr < start + len; addr += FLASH_PAGE_SIZE)
  {
    if (!flash_erase_page_by_addr(addr))
      return false;
  }
  return true;
}
