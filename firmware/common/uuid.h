#ifndef UUID_H
#define UUID_H

#include <stdint.h>

void uuid_init();

#define UUID_LEN 12
extern uint8_t g_uuid[UUID_LEN];

#endif
