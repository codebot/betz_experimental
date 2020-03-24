#include <stdio.h>
#include "delay.h"
#include "uuid.h"

int main(int argc, char **argv)
{
  while (1)
  {
    for (int i = 0; i < 10; i++)
      delay_ms(100);

    printf("uuid = 0x%08x 0x%08x 0x%08x\r\n",
        (unsigned)g_uuid[0],
        (unsigned)g_uuid[1],
        (unsigned)g_uuid[2]);
  }
  return 0;
}
