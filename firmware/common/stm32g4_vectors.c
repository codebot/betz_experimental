#include <stdio.h>
#include "stack.h"
#include "startup.h"

void unhandled_vector(void)
{
  while (1) { }
}

// declare weak symbols for all interrupts so they can be overridden easily
#define WEAK_VECTOR __attribute__((weak, alias("unhandled_vector")))
void nmi_vector(void) WEAK_VECTOR;
void hardfault_vector(void) WEAK_VECTOR;
void memmanage_vector(void) WEAK_VECTOR;
void busfault_vector(void) WEAK_VECTOR;
void usagefault_vector(void) WEAK_VECTOR;
void svcall_vector(void) WEAK_VECTOR;
void debugmonitor_vector(void) WEAK_VECTOR;
void pendsv_vector(void) WEAK_VECTOR;
void systick_vector(void) WEAK_VECTOR;

// interrupt 0
void wwdg_vector(void) WEAK_VECTOR;
void pvd_vector(void) WEAK_VECTOR;
void rtc_tamp_css_lse_vector(void) WEAK_VECTOR;
void rtc_wkup_vector(void) WEAK_VECTOR;
void flash_vector(void) WEAK_VECTOR;
void rcc_vector(void) WEAK_VECTOR;
void exti0_vector(void) WEAK_VECTOR;
void exti1_vector(void) WEAK_VECTOR;
void exti2_vector(void) WEAK_VECTOR;
void exti3_vector(void) WEAK_VECTOR;

// interrupt 10
void exti4_vector(void) WEAK_VECTOR;
void dma1_ch1_vector(void) WEAK_VECTOR;
void dma1_ch2_vector(void) WEAK_VECTOR;
void dma1_ch3_vector(void) WEAK_VECTOR;
void dma1_ch4_vector(void) WEAK_VECTOR;
void dma1_ch5_vector(void) WEAK_VECTOR;
void dma1_ch6_vector(void) WEAK_VECTOR;
void dma1_ch7_vector(void) WEAK_VECTOR;
void adc1_2_vector(void) WEAK_VECTOR;
void usb_hp_vector(void) WEAK_VECTOR;

// interrupt 20
void usb_lp_vector(void) WEAK_VECTOR;
void fdcan1_intr1_vector(void) WEAK_VECTOR;
void fdcan1_intr0_vector(void) WEAK_VECTOR;
void exti9_5_vector(void) WEAK_VECTOR;
void tim1_brk_tim9_vector(void) WEAK_VECTOR;
void tim1_up_tim10_vector(void) WEAK_VECTOR;
void tim1_trg_com_tim11_vector(void) WEAK_VECTOR;
void tim1_cc_vector(void) WEAK_VECTOR;
void tim2_vector(void) WEAK_VECTOR;
void tim3_vector(void) WEAK_VECTOR;

// interrupt 30
void tim4_vector(void) WEAK_VECTOR;
void i2c1_ev_vector(void) WEAK_VECTOR;
void i2c1_er_vector(void) WEAK_VECTOR;
void i2c2_ev_vector(void) WEAK_VECTOR;
void i2c2_er_vector(void) WEAK_VECTOR;
void spi1_vector(void) WEAK_VECTOR;
void spi2_vector(void) WEAK_VECTOR;
void usart1_vector(void) WEAK_VECTOR;
void usart2_vector(void) WEAK_VECTOR;
void usart3_vector(void) WEAK_VECTOR;

// interrupt 40
void exti15_10_vector(void) WEAK_VECTOR;
void rtc_alarm_vector(void) WEAK_VECTOR;
void otg_fs_wkup_vector(void) WEAK_VECTOR;
void tim8_brk_tim12_vector(void) WEAK_VECTOR;
void tim8_up_tim13_vector(void) WEAK_VECTOR;
void tim8_trg_com_tim14_vector(void) WEAK_VECTOR;
void tim8_cc_vector(void) WEAK_VECTOR;
void dma1_stream7_vector(void) WEAK_VECTOR;
void fsmc_vector(void) WEAK_VECTOR;
void sdio_vector(void) WEAK_VECTOR;

// interrupt 50
void tim5_vector(void) WEAK_VECTOR;
void spi3_vector(void) WEAK_VECTOR;
void uart4_vector(void) WEAK_VECTOR;
void uart5_vector(void) WEAK_VECTOR;
void tim6_dac_vector(void) WEAK_VECTOR;
void tim7_vector(void) WEAK_VECTOR;
void dma2_stream0_vector(void) WEAK_VECTOR;
void dma2_stream1_vector(void) WEAK_VECTOR;
void dma2_stream2_vector(void) WEAK_VECTOR;
void dma2_stream3_vector(void) WEAK_VECTOR;

// interrupt 60
void dma2_stream4_vector(void) WEAK_VECTOR;
void eth_vector(void) WEAK_VECTOR;
void eth_wkup_vector(void) WEAK_VECTOR;
void can2_tx_vector(void) WEAK_VECTOR;
void can2_rx0_vector(void) WEAK_VECTOR;
void can2_rx1_vector(void) WEAK_VECTOR;
void can2_sce_vector(void) WEAK_VECTOR;
void otg_fs_vector(void) WEAK_VECTOR;
void dma2_stream5_vector(void) WEAK_VECTOR;
void dma2_stream6_vector(void) WEAK_VECTOR;

// interrupt 70
void dma2_stream7_vector(void) WEAK_VECTOR;
void usart6_vector(void) WEAK_VECTOR;
void i2c3_ev_vector(void) WEAK_VECTOR;
void i2c3_er_vector(void) WEAK_VECTOR;
void otg_hs_ep1_out_vector(void) WEAK_VECTOR;
void otg_hs_ep1_in_vector(void) WEAK_VECTOR;
void otg_hs_wkup_vector(void) WEAK_VECTOR;
void otg_hs_vector(void) WEAK_VECTOR;
void dcmi_vector(void) WEAK_VECTOR;
void cryp_vector(void) WEAK_VECTOR;
void hash_rng_vector(void) WEAK_VECTOR;
void fpu_vector(void) WEAK_VECTOR;

void dummy_reset_vector(void) { }

typedef void (*vector_func_t)(void);
__attribute__((section(".vectors"))) vector_func_t g_vectors[] =
{
  (vector_func_t)(&g_stack[STACK_SIZE-8]), // initial stack pointer
  reset_vector,
  nmi_vector,
  hardfault_vector,
  memmanage_vector,
  busfault_vector,
  usagefault_vector,
  0, 0, 0, 0,
  svcall_vector,
  debugmonitor_vector,
  0,
  pendsv_vector,
  systick_vector,
  wwdg_vector,       // 0
  pvd_vector,        
  rtc_tamp_css_lse_vector,
  rtc_wkup_vector,
  flash_vector,
  rcc_vector,  // 5
  exti0_vector,
  exti1_vector,
  exti2_vector,
  exti3_vector,
  exti4_vector, // 10
  dma1_ch1_vector,
  dma1_ch2_vector,
  dma1_ch3_vector,
  dma1_ch4_vector,
  dma1_ch5_vector,
  dma1_ch6_vector,
  dma1_ch7_vector,
  adc1_2_vector,
  usb_hp_vector,

  usb_lp_vector,  // 20
  fdcan1_intr1_vector,
  fdcan1_intr0_vector,
  exti9_5_vector,
  tim1_brk_tim9_vector,
  tim1_up_tim10_vector,
  tim1_trg_com_tim11_vector,
  tim1_cc_vector,
  tim2_vector,
  tim3_vector,

  tim4_vector,  // 30
  i2c1_ev_vector,
  i2c1_er_vector,
  i2c2_ev_vector,
  i2c2_er_vector,
  spi1_vector,
  spi2_vector,
  usart1_vector,
  usart2_vector,
  usart3_vector,
  exti15_10_vector,  // 40
  rtc_alarm_vector,
  otg_fs_wkup_vector,
  tim8_brk_tim12_vector,
  tim8_up_tim13_vector,
  tim8_trg_com_tim14_vector,
  tim8_cc_vector,
  dma1_stream7_vector,
  fsmc_vector,
  sdio_vector,
  tim5_vector,  // 50
  spi3_vector,
  uart4_vector,
  uart5_vector,
  tim6_dac_vector,
  tim7_vector,
  dma2_stream0_vector,
  dma2_stream1_vector,
  dma2_stream2_vector,
  dma2_stream3_vector,
  dma2_stream4_vector,  // 60
  eth_vector,
  eth_wkup_vector,
  can2_tx_vector,
  can2_rx0_vector,
  can2_rx1_vector,
  can2_sce_vector,
  otg_fs_vector,
  dma2_stream5_vector,
  dma2_stream6_vector,
  dma2_stream7_vector,  // 70
  usart6_vector,
  i2c3_ev_vector,
  i2c3_er_vector,
  otg_hs_ep1_out_vector,
  otg_hs_ep1_in_vector,
  otg_hs_wkup_vector,
  otg_hs_vector,
  dcmi_vector,
  cryp_vector,
  hash_rng_vector,
  fpu_vector
};

