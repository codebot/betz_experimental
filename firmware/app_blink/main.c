#include "status_led.h"

int main(int argc, char **argv)
{
  while (1)
  {
    for (volatile int i = 0; i < 500000; i++) { }
    status_led_toggle();
  }
  return 0;
}
