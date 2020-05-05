#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>

typedef enum
{
  PARAM_TYPE_INVALID = 0,
  PARAM_TYPE_INT = 1,
  PARAM_TYPE_FLOAT = 2
} param_type_t;

typedef enum
{
  PARAM_PERSISTENT = 0,
  PARAM_TRANSIENT = 1
} param_storage_t;

void param_init();
uint32_t param_count();

void param_int(
    const char *name,
    volatile int *ptr,
    const int default_value,
    const param_storage_t param_storage);

const char *param_get_name(const uint32_t param_idx);
param_type_t param_get_type(const uint32_t param_idx);
param_storage_t param_get_storage(const uint32_t param_idx);
volatile void *param_get_ptr(const uint32_t param_idx);
void param_set_float(const char *param_name, const float value);
void param_set_int(const char *param_name, const uint32_t value);

#endif
