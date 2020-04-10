#include "uuid.h"

uint32_t g_uuid[UUID_LEN];

void uuid_init()
{
  g_uuid[0] = 0x00112233;
  g_uuid[1] = 0x44556677;
  g_uuid[2] = 0x89abcdef;
}
