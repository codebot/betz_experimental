#include "comms.h"
#include "rs485.h"
#include "status_led.h"
#include "systime.h"

int main(int argc, char **argv)
{
  const uint32_t half_blink_us = 50000;
  uint32_t t_next_toggle = systime_read() + half_blink_us;

  while (1)
  {
    rs485_tick();
    comms_tick();
    if (systime_read() > t_next_toggle)
    {
      status_led_toggle();
      t_next_toggle += half_blink_us;
    }
  }
  return 0;
}
