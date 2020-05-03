#include <stdio.h>

#include "comms.h"
#include "pwm.h"
#include "rs485.h"
#include "status_led.h"
#include "systime.h"

int main(int argc, char **argv)
{
  const uint32_t print_us = 100000;
  uint32_t t_next = systime_read() + print_us;

  const int mag = 50;
  pwm_set(
    pwm_max() / 2 + mag/2,
    pwm_max() / 2 + mag/2,
    pwm_max() / 2 - mag);

  while (1)
  {
    rs485_tick();
    comms_tick();
    if (systime_read() > t_next)
    {
      t_next += print_us;
      status_led_toggle();
    }
  }
  return 0;
}
