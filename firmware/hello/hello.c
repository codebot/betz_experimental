#include <stdio.h>
#include "status_led.h"

int main(int argc, char **argv)
{
  int hello_count = 0;
  printf("hello, world!\r\n");
  while (1)
  {
    for (volatile int i = 0; i < 5000000; i++) { }
    status_led_toggle();
    printf("hello %d\r\n", hello_count);
    hello_count++;
  }
  return 0;
}
