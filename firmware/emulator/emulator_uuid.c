#include "uuid.h"

uint8_t g_uuid[UUID_LEN];

void uuid_init()
{
  for (int i = 0; i < UUID_LEN; i++)
    g_uuid[i] = i;
}
