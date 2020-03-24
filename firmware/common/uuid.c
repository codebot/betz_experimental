#include "uuid.h"
#include "stm32f405xx.h"

uint32_t g_uuid[UUID_LEN];

void uuid_init()
{
  g_uuid[0] = *((uint32_t *)UID_BASE);
  g_uuid[1] = *((uint32_t *)(UID_BASE+4));
  g_uuid[2] = *((uint32_t *)(UID_BASE+8));
}
