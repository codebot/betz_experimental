#include <stdint.h>
#include "console.h"
#include "comms.h"
#include "rs485.h"
#include "stm32f405xx.h"
#include "startup.h"
#include "stack.h"
#include "status_led.h"
#include "systime.h"
#include "uuid.h"

extern uint32_t _srelocate_flash, _srelocate, _erelocate, _ebss, _sbss;
extern int main();

void startup_clock_init_fail() { while (1) { } }
void __libc_init_array(void);

void reset_vector()
{
  // need to put a reference to the stack array
  // to make sure the linker brings it in. I'm sure there
  // is a more elegant way to do this, but this seems to work
  g_stack[0] = 0;

  //watchdog_reset_counter();

  // set up data segment
  uint32_t *pSrc = &_srelocate_flash;
  uint32_t *pDest = &_srelocate;
  if (pSrc != pDest)
    for (; pDest < &_erelocate; )
      *pDest++ = *pSrc++;

  // set up bss segment
  for (pDest = &_sbss; pDest < &_ebss; )
    *pDest++ = 0;

  __libc_init_array() ;
  SCB->CPACR |= ((3UL << (10*2)) | (3UL << (11*2))); // activate the FPU

  // set up the clocking scheme
  RCC->CR |= 0x1; // ensure the HSI (internal) oscillator is on
  RCC->CFGR = 0; // ensure the HSI oscillator is the clock source
  RCC->CR &= 0xfef6ffff; // turn off the main PLL and HSE oscillator
  RCC->PLLCFGR = 0x24003010; // ensure PLLCFGR is at reset state
  RCC->CR &= 0xfffbffff; // reset HSEBYP (i.e., HSE is *not* bypassed)
  RCC->CIR = 0x0; // disable all RCC interrupts
  RCC->CR |= RCC_CR_HSEON; // enable HSE oscillator (off-chip crystal)
  for (volatile uint32_t i = 0; 
       i < 0x5000 && !(RCC->CR & RCC_CR_HSERDY); i++)
  {
    // no op... wait for either timeout or HSE to spin up
  }
  if (!(RCC->CR & RCC_CR_HSERDY))
    startup_clock_init_fail(); // go there and spin forever. BUH BYE

  // we need to set 5 wait states on the flash controller to go 168 MHz
  FLASH->ACR = 0; // ensure the caches are turned off, so we can reset them
  FLASH->ACR = FLASH_ACR_DCRST | FLASH_ACR_ICRST; // flush the cache
  FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | 
               FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS; // re-enable the caches

  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // clock up the power controller
  PWR->CR |= PWR_CR_VOS; // ensure the voltage regulator is at max beef
                         // this will let us run at 168 MHz without overdrive
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // set HCLK (AHB clock) to sysclock
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // set APB high-speed clock to sysclock/2
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // set APB  low-speed clock to sysclock/4

  #define CRYSTAL_MHZ 8000000
  #define PLL_M (CRYSTAL_MHZ/2000000)
  // with HSE_VALUE = 8 MHz, PLL_M will be 4
  #define PLL_N 168
  // PLL_VCO = crystal mhz / PLL_M * PLL_N = 336 MHz
  #define PLL_P   2
  // SYSCLK = PLL_VCO / PLL_P = 168 MHz
  #define PLL_Q   7
  // USB clock = PLL_VCO / PLL_Q = 48 MHz 
  RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1)-1) << 16) |
                 (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);
  RCC->CR |= RCC_CR_PLLON; // start spinning up the PLL
  while (!(RCC->CR & RCC_CR_PLLRDY)) { } // wait until it's spun up

  RCC->CFGR &= ~((uint32_t)RCC_CFGR_SW); // select internal oscillator
  RCC->CFGR |= RCC_CFGR_SW_PLL; // select PLL as clock source
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { } // wait for it...
  // hooray we're done! we're now running at 168 MHz.

  status_led_init();
  console_init();
  systime_init();
  param_init();
  uuid_init();
  rs485_init();
  control_init();

  comms_init();
  comms_set_raw_tx_fptr(rs485_tx);

  __enable_irq();
  main(); // jump to application main()
  while (1) { } // hopefully we never get here...
}

