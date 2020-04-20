#include <stdio.h>
#include "delay.h"
#include "rng.h"
#include "stm32f405xx.h"

int main(int argc, char **argv)
{
  while (1)
  {
    for (int i = 0; i < 10; i++)
      delay_ms(100);

    const uint32_t r = rng_read();
    printf("rng = 0x%08x\r\n", (unsigned)r);
  }
  return 0;
}
