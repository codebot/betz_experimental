#include <stdio.h>
#include "rs485.h"
#include "status_led.h"

int main(int argc, char **argv)
{
  status_led_off();
  while (1)
  {
    rs485_tick();
  }
  return 0;
}
