#include <stdint.h>

#include "adc.h"
#include "console.h"
#include "control.h"
#include "comms.h"
#include "enc.h"
#include "flash.h"
#include "param.h"
#include "pwm.h"
#include "rng.h"
#include "rs485.h"
#include "soc.h"
#include "startup.h"
#include "stack.h"
#include "state.h"
#include "status_led.h"
#include "systime.h"
#include "uuid.h"

extern uint32_t _srelocate_flash, _srelocate, _erelocate, _ebss, _sbss;
extern int main();

void startup_clock_init_fail() { while (1) { } }
void __libc_init_array(void);

volatile int ahhhh = 42;

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

  __libc_init_array();

  SCB->CPACR |= ((3UL << (10*2)) | (3UL << (11*2))); // activate the FPU

#if defined(BOARD_blue)
  // we need to set 5 wait states on the flash controller to go 168 MHz
  FLASH->ACR = 0; // ensure the caches are turned off, so we can reset them
  FLASH->ACR = FLASH_ACR_DCRST | FLASH_ACR_ICRST; // flush the cache
  FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | 
               FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS; // re-enable the caches
#elif defined(BOARD_mini)
  // we need to set 4 wait states on the flash controller to go 168 MHz
  FLASH->ACR |=
      FLASH_ACR_DCEN   |
      FLASH_ACR_ICEN   |
      FLASH_ACR_PRFTEN |
      FLASH_ACR_LATENCY_4WS ;
#endif

  RCC->CR |= RCC_CR_HSION; // ensure the HSI (internal) oscillator is on
  RCC->CFGR = RCC_CFGR_SW_HSI; // ensure the HSI oscillator is the clock source
  // turn off main PLL and HSE oscillators
  RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_CSSON | RCC_CR_HSEBYP | RCC_CR_HSEON);

  // set up the clocking scheme
#if defined(BOARD_blue)
  RCC->PLLCFGR = 0x24003010; // ensure PLLCFGR is at reset state
  RCC->CIR = 0x0; // disable all RCC interrupts
#elif defined(BOARD_mini)
  RCC->PLLCFGR = 0x00001000; // ensure PLLCFGR is at reset state
#endif


  RCC->CR |= RCC_CR_HSEON; // enable HSE oscillator (off-chip crystal)

  for (volatile uint32_t i = 0; 
       i < 0x5000 && !(RCC->CR & RCC_CR_HSERDY); i++)
  {
    // no op... wait for either timeout or HSE to spin up
  }
  if (!(RCC->CR & RCC_CR_HSERDY))
    startup_clock_init_fail(); // go there and spin forever. BUH BYE


  // both boards have an 8 MHz crystal
  #define CRYSTAL_MHZ 8000000

#if defined(BOARD_blue)
  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // clock up the power controller
  PWR->CR |= PWR_CR_VOS; // ensure the voltage regulator is at max beef

  RCC->CFGR |=
      RCC_CFGR_HPRE_DIV1  | // set HCLK (AHB clock) to sysclock
      RCC_CFGR_PPRE2_DIV2 | // set APB high-speed clock to sysclock/2
      RCC_CFGR_PPRE1_DIV4 ; // set APB  low-speed clock to sysclock/4

  // we want PLL input clock of 2 MHz
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

#elif defined(BOARD_mini)
  RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // clock up the power controller
  PWR->CR5 &= ~PWR_CR5_R1MODE;  // boost mode for extra beef
  PWR->CR1 |= PWR_CR1_VOS_0; // ensure the voltage regulator is at Range 1
  // on STM32G4 (very different from STM32F4 in this respect), we can take
  // the peripheral buses all up to 170 MHz

  // we want PLL input clock of 8 MHz
  #define PLL_M (CRYSTAL_MHZ/8000000)
  // with an 8 MHz crystal, PLL_M will be 1, so we have fPLL_IN = 8 MHz

  #define PLL_N 42
  // PLL_VCO = crystal / PLL_M * PLL_N = 8 / 1 * 21 = 336 MHz

  // PLL_P will be 7, to end up with ADC clock of 336/7 = 48 MHz
  // PLL_Q will be 8, but we do not need it (no USB). Don't enable output
  // PLL_R will be 2, to end up with main clock of 336/2 = 168 MHz

  // we will temporarily set AHB to divide by 2 during spin-up
  RCC->CFGR |= RCC_CFGR_HPRE_DIV2;  // set HCLK (AHB clock) to sysclock/2

  RCC->PLLCFGR =
      RCC_PLLCFGR_PLLSRC_HSE | // PLL input is crystal oscillator
      0                      | // leave PLL_M as zero to use div1 for input
      (PLL_N << RCC_PLLCFGR_PLLN_Pos) |
      RCC_PLLCFGR_PLLPEN     | // enable PLL P output (for ADC) with div7
      RCC_PLLCFGR_PLLQ_0     | // set PLL Q divider to 8 (unused)
      RCC_PLLCFGR_PLLQ_1     | // set PLL Q divider to 8 (unused)
      RCC_PLLCFGR_PLLREN     ; // enable main (R) output with div2

#endif

  RCC->CR |= RCC_CR_PLLON; // start spinning up the PLL
  while (!(RCC->CR & RCC_CR_PLLRDY)) { } // wait until it's spun up

  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
  RCC->CFGR &= ~((uint32_t)RCC_CFGR_SW); // select internal oscillator
  RCC->CFGR |= RCC_CFGR_SW_PLL; // select PLL as clock source
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { } // wait for it...
  // hooray we're done! PLL output is now 168 MHz

#if defined(BOARD_mini)
  // on the STM32G4, must wait 1 usec, then set AHB divider to div1
  for (volatile int i = 0; i < 1000; i++) { }
  RCC->CFGR &= ~RCC_CFGR_HPRE;  // now we're set for div1 on HCLK
#endif

  status_led_init();
  console_init();
  systime_init();
  state_init();
  rs485_init();
  param_init();
  uuid_init();
  flash_init();
  rng_init();
  pwm_init();
  control_init();
  enc_init();

  comms_init(rs485_tx);

  __enable_irq();
  main();

  // TODO: move stuff up as it's verified...

  adc_init();

  while (1) { } // hopefully we never get here...
}
