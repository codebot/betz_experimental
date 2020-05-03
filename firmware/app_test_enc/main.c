#include <stdio.h>

#include "comms.h"
#include "enc.h"
#include "rs485.h"
#include "status_led.h"
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
      const unsigned pos = (unsigned)enc_read_pos_blocking();
      status_led_toggle();
      printf("pos: %04x\r\n", pos);
    }
  }
  return 0;
}
