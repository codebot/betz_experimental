#include <stdio.h>
#include <string.h>

#include "flash.h"
#include "stm32f405xx.h"
#include "systime.h"
//#include "watchdog.h"


// stm32f405 has 12 flash pages
// flash organization:
//   bootloader: 128 KB spanning sectors 0, 1, 2, 3, 4
//   application: 768 KB spanning sectors 5..10
//   parameters: 128 KB in sector 11

// stm32g474 has 128 pages of 4K bytes each (512K total)
// flash organization:
//   bootloader: first 128 KB
//   application: next 256 KB
//   parameters: last 128 KB

#define NUM_SECTORS 12

static void flash_unlock();
static void flash_lock() __attribute__((unused));
static bool flash_wait_for_idle();
static uint32_t g_flash_size = 0;
static uint32_t g_flash_page_size = 0;
static bool flash_erase_page_by_addr(const uint32_t addr);
static bool flash_program_word(const uint32_t addr, const uint32_t data);

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

  /*
  printf("  flash size: %d\r\n", (int)g_flash_size);
  printf("  idcode: 0x%08x\r\n", (int)idcode);
  printf("  dev_id = 0x%03x\r\n", (int)dev_id);
  printf("  flash page size = %d\r\n", (int)g_flash_page_size);
  */
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

bool flash_program_word(const uint32_t addr, const uint32_t data)
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

void flash_read(
    const uint32_t read_addr,
    const uint32_t len,
    uint8_t *dest_addr)
{
  if (len >= 65536)
    return;  // sanity-check to avoid anything super bonkers
  memcpy(dest_addr, (const void *)read_addr, len);
}

bool flash_write(
    const uint32_t write_addr,
    const uint32_t write_len,
    const uint8_t *write_data)
{
  if (write_addr % g_flash_page_size == 0)
  {
    if (!flash_erase_page_by_addr(write_addr))
      return false;
  }

  // now we can program the payload
  for (int i = 0; i < write_len; i += 4)
  {
    uint32_t word = 0;
    memcpy(&word, &write_data[i], sizeof(word));
    flash_program_word(write_addr + i, word);
  }

  return true;
}

bool flash_erase_page_by_addr(const uint32_t addr)
{
  printf("erase page at 0x%08x\r\n", (unsigned)addr);
  if (g_flash_page_size == 0x20000)
  {
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
  }
  else if (g_flash_page_size == 0x1000)
  {
    // todo: stm32g474 and friends
    return false;
  }
  else
  {
    printf(
        "OH NO! unrecognized g_flash_page_size: 0x%08x\n",
        (unsigned)g_flash_page_size);
    return false;
  }
}
