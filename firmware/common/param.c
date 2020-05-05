#include <stdio.h>
#include <string.h>
#include "param.h"

struct param
{
  const char *n;  // name
  int t;  // type
  volatile void *p;  // pointer to value
  param_storage_t storage;
};

#define MAX_PARAMS 100
static struct param params[MAX_PARAMS];
static uint32_t num_params = 0;

void param_init()
{
  num_params = 0;
  for (uint32_t i = 0; i < MAX_PARAMS; i++)
  {
    params[i].t = PARAM_TYPE_INVALID;
    params[i].n = NULL;
    params[i].p = NULL;
  }
}

uint32_t param_count()
{
  return num_params;
}

void param_int(
    const char *name,
    volatile int *ptr,
    const int default_value,
    const param_storage_t storage)
{
  if (num_params >= MAX_PARAMS) {
    printf("out of param storage; can't add param [%s]\n", name);
    return;  // no more room. sorry
  }
  params[num_params].n = name;
  params[num_params].t = PARAM_TYPE_INT;
  params[num_params].p = ptr;
  params[num_params].storage = storage;
  num_params++;

  if (storage == PARAM_PERSISTENT)
  {
    // todo: load from flash if it's there
    // otherwise, set to default value
    *ptr = default_value;
  }
}

const char *param_get_name(const uint32_t param_idx)
{
  if (param_idx >= num_params)
    return NULL;
  return params[param_idx].n;
}

param_type_t param_get_type(const uint32_t param_idx)
{
  if (param_idx >= num_params)
    return PARAM_TYPE_INVALID;
  return params[param_idx].t;
}

volatile void *param_get_ptr(const uint32_t param_idx)
{
  if (param_idx >= num_params)
    return NULL;
  return params[param_idx].p;
}

param_storage_t param_get_storage(const uint32_t param_idx)
{
  if (param_idx >= num_params)
    return PARAM_TRANSIENT;
  return params[param_idx].storage;
}

//void param_set_int(const char *param_name, const uint32_t value)

void param_set_float(const char *param_name, const float value)
{
  for (uint32_t i = 0; i < num_params; i++)
  {
    struct param *p = &params[i];
    if (p->t == PARAM_TYPE_FLOAT && !strcmp(p->n, param_name))
    {
      *((float *)p->p) = value;
      break;
    }
  }
}
