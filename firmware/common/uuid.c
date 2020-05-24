#include <string.h>

#include "uuid.h"
#include "soc.h"

uint8_t g_uuid[UUID_LEN];

void uuid_init()
{
  memcpy(&g_uuid[0], (uint8_t *)UID_BASE, UUID_LEN);
}
