#include <stdio.h>
#include "uuid.h"

uint8_t g_uuid[UUID_LEN];

void uuid_init()
{
}

void emulator_uuid_set_serial_number(const uint64_t serial_number)
{
  printf("serial number: 0x%llx\n", (long long unsigned)serial_number);

  for (int i = 0; i < 4; i++)
    g_uuid[i] = 0;
  for (int i = 4; i < 12; i++)
    g_uuid[i] = (uint8_t)((serial_number >> (8*i)) & 0xff);
}
