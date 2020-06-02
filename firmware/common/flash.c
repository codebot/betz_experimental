#include <stdio.h>
#include <string.h>

#include "flash.h"
#include "soc.h"
#include "systime.h"
//#include "watchdog.h"


#if defined(BOARD_blue)

// stm32f405 has 12 flash pages
// flash organization:
//   bootloader: 128 KB spanning sectors 0, 1, 2, 3, 4
//   application: 768 KB spanning sectors 5..10
//   parameters: 128 KB in sector 11

#elif defined(BOARD_mini)

// stm32g474 has 128 pages of 4K bytes each (512K total)
// flash organization:
//   bootloader: first 128 KB
//   application: next 256 KB
//   parameters: last 128 KB

#endif

#define FLASH_WRITE_BLOCK_SIZE 8
static uint8_t flash_write_buf[FLASH_WRITE_BLOCK_SIZE];
static uint32_t flash_write_buf_idx;
static uint32_t flash_write_addr;

// temporary
static void flash_unlock();
static void flash_lock() __attribute__((unused));
static bool flash_wait_for_idle();

static uint32_t g_flash_size = 0;
static uint32_t g_flash_page_size = 0;

void flash_init()
{
  // figure out the parameters of the device we are running on
  uint16_t size_kb = 0;
  memcpy(&size_kb, (uint16_t *)FLASHSIZE_BASE, 2);
  g_flash_size = (uint32_t)size_kb * 1024;

  uint32_t idcode = 0;
  memcpy(&idcode, (uint32_t *)DBGMCU_BASE, 4);

  const uint32_t dev_id = idcode & 0xfff;
  if (dev_id == 0x413)
    g_flash_page_size = 0x20000;  // 128 KB flash sectors
  else if (dev_id == 0x469)
    g_flash_page_size = 0x1000;  // 4096 byte flash pages
  else
    printf("AAAAAAHHHH! unknown dev_id!\r\n");

  printf("  flash size: %d\r\n", (int)g_flash_size);
  printf("  idcode: 0x%08x\r\n", (int)idcode);
  printf("  dev_id = 0x%03x\r\n", (int)dev_id);
  printf("  flash page size = %d\r\n", (int)g_flash_page_size);
}

void flash_unlock()
{
  // magic sequence to unlock flash...
  if (FLASH->CR & FLASH_CR_LOCK)
  {
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xcdef89ab;
  }
  if (FLASH->CR & FLASH_CR_LOCK)
    printf("couldn't unlock flash\r\n");
}

void flash_lock() 
{
  FLASH->CR |= FLASH_CR_LOCK;
}

bool flash_wait_for_idle()
{
  // uint32_t t_start = SYSTIME;
  while (FLASH->SR & FLASH_SR_BSY)
  {
    // keep resetting the watchdog, since this may be a long-running erase op
    // give it 10 seconds of watchdog auto-resetting
    // TODO: actually enable watchdog someday...
    // if (SYSTIME - t_start < 10000000)
    //   watchdog_reset_counter();
  }
  // todo: check all the error bits, etc.
  if (FLASH->SR & 0x1f3)
  {
    printf("unexpected FLASH_SR error bit: FLASH_SR = 0x%08lx\r\n",
           FLASH->SR);
    FLASH->SR |= (FLASH->SR & 0x1f3); // clear the error bit(s)
    return false;
  }
  return true;
}

#if defined(BOARD_blue)
bool flash_stm32f4_program_word(const uint32_t addr, const uint32_t data)
{
  flash_unlock();
  if (!flash_wait_for_idle())
    return false;
  FLASH->CR |= FLASH_CR_PG; // set the programming bit
  FLASH->CR &= ~FLASH_CR_PSIZE; // wipe out PSIZE to get ready to set it
  FLASH->CR |=  FLASH_CR_PSIZE_1; // we'll do 32-bit erases at a time
  *((volatile uint32_t *)addr) = data;
  const bool result = flash_wait_for_idle();
  FLASH->CR &= ~FLASH_CR_PG; // disable the programming bit
  //flash_lock();
  return result;
}
#endif

bool flash_program_dword(
    const uint32_t addr,
    const uint32_t word_0,
    const uint32_t word_1)
{
#if defined(BOARD_blue)
  if (!flash_stm32f4_program_word(addr, word_0))
    return false;
  if (!flash_stm32f4_program_word(addr+4, word_1))
    return false;
  return true;
#else

  flash_unlock();
  if (!flash_wait_for_idle())
    return false;
  FLASH->CR |= FLASH_CR_PG; // set the programming bit
  *((volatile uint32_t *)addr) = word_0;
  *((volatile uint32_t *)addr+4) = word_1;
  const bool result = flash_wait_for_idle();
  FLASH->CR &= ~FLASH_CR_PG; // disable the programming bit
  //flash_lock();
  return result;

#endif
}

void flash_read(
    const uint32_t read_addr,
    const uint32_t len,
    void *dest_addr)
{
  if (len >= 65536)
    return;  // sanity-check to avoid anything super bonkers
  memcpy(dest_addr, (const void *)read_addr, len);
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
  if (write_len % 8 != 0)
  {
    printf(
        "flash_write len must be a multiple of 8. saw %d\r\n",
        (int)write_len);
    return false;
  }

  if (write_addr % g_flash_page_size == 0)
  {
    if (!flash_erase_page_by_addr(write_addr))
      return false;
  }

  // now we can program the payload
  for (int i = 0; i < write_len; i += 8)
  {
    uint32_t words[2] = {0};
    memcpy(&words, &write_data[i], 8);
    if (!flash_program_dword(write_addr + i, words[0], words[1]))
      return false;
  }

  return true;
}

bool flash_erase_page_by_addr(const uint32_t addr)
{
  printf("erase page at 0x%08x\r\n", (unsigned)addr);
#if defined(BOARD_blue)
  // stm32f405 and friends
  if (addr <  0x08020000 ||
      addr >  0x080fffff ||
      (addr & 0x1ffff) != 0)
    return false;  // bad address
  const int offset = addr - 0x08020000;
  const int sector = 5 + offset / g_flash_page_size;

  flash_unlock();
  flash_wait_for_idle();
  FLASH->CR &= ~FLASH_CR_PSIZE;
  FLASH->CR |=  FLASH_CR_PSIZE_1; // we'll do 32-bit erases at a time
  FLASH->CR &= ~0xf8; // wipe out the sector address bits
  // todo: revisit this calculation for sectors beyond 12 (?)
  FLASH->CR |= sector << 3;  // fill in the sector address bits
  FLASH->CR |= FLASH_CR_SER; // sector erase operation
  FLASH->CR |= FLASH_CR_STRT; // start the sector-erase operation
  const bool result = flash_wait_for_idle();
  FLASH->CR &= ~FLASH_CR_SER; // reset the sector-erase operation bit
  FLASH->CR &= ~0xf8; // and wipe out the sector address bits
  //flash_lock(); // lock the flash again
  return result; // we're done
#elif defined(BOARD_mini)
  // stm32g474 and friends
  if (addr <  0x08020000 ||
      addr >  0x0807ffff ||
      (addr & 0xfff) != 0)
    return false;  // bad address
  const int offset = addr - 0x08020000;
  const int page = 32 + offset / g_flash_page_size;

  flash_unlock();
  flash_wait_for_idle();
  FLASH->CR &= ~0x7f8; // wipe out the page number bits
  FLASH->CR |= page << 3;  // fill in the page number bits
  FLASH->CR |= FLASH_CR_PER; // sector erase operation
  FLASH->CR |= FLASH_CR_STRT; // start the sector-erase operation
  const bool result = flash_wait_for_idle();
  FLASH->CR &= ~FLASH_CR_PER; // reset the sector-erase operation bit
  FLASH->CR &= ~0x7f8; // and wipe out the sector address bits
  //flash_lock(); // lock the flash again
  return result; // we're done
#else
  printf(
      "OH NO! unrecognized g_flash_page_size: 0x%08x\n",
      (unsigned)g_flash_page_size);
  return false;
#endif
}

uint32_t flash_get_param_table_base_addr()
{
  // todo: smarter branching based on MCU type and flash size
  if (g_flash_page_size == 0x1000)
  {
    // STM32G4, hard-code top 128 KB for now
    return 0x08000000 + 3 * 128 * 1024;
  }
  else
  {
    // STM32F405, hard-code top 128 KB for now
    return 0x080e0000;
  }
}

uint32_t flash_get_param_table_size()
{
  // todo: branch based on MCU type and flash size
  return 128 * 1024;
}

bool flash_erase_range(const uint32_t start, const uint32_t len)
{
  printf("flash_erase_range(0x%08x, %d)\r\n", (unsigned)start, (int)len);
  for (uint32_t addr = start; addr < start + len; addr += g_flash_page_size)
  {
    if (!flash_erase_page_by_addr(addr))
      return false;
  }  
  return true;
}

bool flash_write_begin(const uint32_t addr)
{
  flash_write_buf_idx = 0;
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
