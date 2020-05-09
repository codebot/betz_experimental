#include <stdio.h>
#include <string.h>

#include "flash.h"


#define FLASH_LEN (512 * 1024)
#define FLASH_IMAGE_FILENAME "emulator_flash.bin"

static uint8_t flash_image[FLASH_LEN] = {0};

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
    uint8_t *dest_addr)
{
  printf("flash_read(0x%08x, %d)\n", read_addr, len);
  const int offset = read_addr - FLASH_START;
  if (offset > FLASH_LEN)
  {
    printf("AHHH invalid read address!!!\n");
    return;
  }
  memcpy(dest_addr, &flash_image[offset], len);
}

bool flash_write(
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
  printf("erase page at 0x%08x\r\n", (unsigned)addr);
  return true;
}

uint32_t flash_get_param_table_base_addr()
{
  return 0x000e0000;
}
