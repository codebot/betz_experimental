#include <stdio.h>
#include "comms.h"
#include "rs485.h"
#include "status_led.h"
#include "systime.h"

int main(int argc, char **argv)
{
  printf("===\r\nbootloader\r\n===\r\n\r\n");
  comms_set_bootloader_mode(true);

  // since we're not doing any motor control in the bootloader, let's
  // turn off the PWM interrupt that was set up in pwm_init() to have
  // faster response to comms traffic
#if defined(BOARD_blue)
  NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
#elif defined(BOARD_mini)
  NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
#endif

  status_led_off();
  uint32_t t_blink = systime_read();
  while (1)
  {
    rs485_tick();
    comms_tick();
    if (systime_read() - t_blink > 100000)
    {
      t_blink = systime_read();
      status_led_toggle();
    }
  }
  return 0;
}
