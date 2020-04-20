#include <stdio.h>
#include "delay.h"
#include "uuid.h"

int main(int argc, char **argv)
{
  while (1)
  {
    for (int i = 0; i < 10; i++)
      delay_ms(100);

    printf("uuid = ");
    for (int i = 0; i < UUID_LEN; i++)
    {
      printf("%02x", (unsigned)g_uuid[i]);
      if (i % 4 == 3 && i != 11)
        printf(":");
    }
    printf("\r\n");
  }
  return 0;
}
