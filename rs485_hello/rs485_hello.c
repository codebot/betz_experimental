#include <stdio.h>
#include "rs485.h"
#include "status_led.h"

int main(int argc, char **argv)
{
  int hello_count = 0;
  char msg[50] = {0};
  while (1)
  {
    for (volatile int i = 0; i < 5000000; i++) { }
    status_led_toggle();
    const int n = snprintf(msg, sizeof(msg), "hello %d\r\n", hello_count);
    hello_count++;
    rs485_tx((uint8_t *)msg, n);
  }
  return 0;
}
