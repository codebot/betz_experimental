#include <stdio.h>
#include "comms.h"
#include "rs485.h"
#include "status_led.h"
#include "systime.h"

int main(int argc, char **argv)
{
  comms_set_bootloader_mode();
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
