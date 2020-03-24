#ifndef UUID_H
#define UUID_H

#include <stdint.h>

void uuid_init();

#define UUID_LEN 3
extern uint32_t g_uuid[UUID_LEN];

#endif
