#include <stdio.h>

#include "adc.h"
#include "comms.h"
#include "rs485.h"
#include "status_led.h"
#include "state.h"
#include "systime.h"

int main(int argc, char **argv)
{
  const uint32_t print_us = 100000;
  uint32_t t_next_read = systime_read() + print_us;

  while (1)
  {
    rs485_tick();
    comms_tick();
    if (systime_read() > t_next_read)
    {
      t_next_read += print_us;

      adc_blocking_read();
      printf(
          "adc: %04d  %04d  %04d\r\n",
          g_state.raw_adc[0],
          g_state.raw_adc[1],
          g_state.raw_adc[2]);
    }
  }
  return 0;
}
