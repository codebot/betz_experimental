#include <stdio.h>
#include "delay.h"
#include "systime.h"
#include "status_led.h"

int main(int argc, char **argv)
{
  while (1)
  {
    for (int i = 0; i < 10; i++)
      delay_ms(100);

    status_led_toggle();
    const volatile uint32_t systime = SYSTIME;
    printf("systime = %d\r\n", (int)systime);
  }
  return 0;
}
