#include <stdio.h>
#include <string.h>

#include "flash.h"
#include "param.h"

struct param
{
  const char *n;  // name
  int t;  // type
  volatile void *p;  // pointer to value
  param_storage_t storage;
};

#define MAX_PARAMS 100
static struct param g_param_params[MAX_PARAMS];
static uint32_t g_param_num_params = 0;

typedef union
{
  uint32_t u32;
  int32_t i;
  float f;
} param_value_t;

static bool param_find_flash_value(
    const char *name,
    const int type,
    param_value_t *value);

/////////////////////////////////////////////////////////////

void param_init()
{
  g_param_num_params = 0;
  for (uint32_t i = 0; i < MAX_PARAMS; i++)
  {
    g_param_params[i].t = PARAM_TYPE_INVALID;
    g_param_params[i].n = NULL;
    g_param_params[i].p = NULL;
  }
}

uint32_t param_count()
{
  return g_param_num_params;
}

void param_int(
    const char *name,
    volatile int *ptr,
    const int default_value,
    const param_storage_t storage)
{
  if (g_param_num_params >= MAX_PARAMS) {
    printf("out of param storage; can't add param [%s]\n", name);
    return;  // no more room. sorry
  }
  g_param_params[g_param_num_params].n = name;
  g_param_params[g_param_num_params].t = PARAM_TYPE_INT;
  g_param_params[g_param_num_params].p = ptr;
  g_param_params[g_param_num_params].storage = storage;
  g_param_num_params++;

  *ptr = default_value;

  if (storage == PARAM_PERSISTENT)
  {
    param_value_t value;
    if (param_find_flash_value(name, PARAM_TYPE_INT, &value))
    {
      printf("found param [%s] in flash: %d\r\n", name, (int)value.i);
      *ptr = value.i;
    }
    else
    {
      printf("couldn't find param [%s] in flash\r\n", name);
    }
  }
}

void param_float(
    const char *name,
    volatile float *ptr,
    const float default_value,
    const param_storage_t storage)
{
  if (g_param_num_params >= MAX_PARAMS) {
    printf("out of param storage; can't add param [%s]\n", name);
    return;  // no more room. sorry
  }
  g_param_params[g_param_num_params].n = name;
  g_param_params[g_param_num_params].t = PARAM_TYPE_FLOAT;
  g_param_params[g_param_num_params].p = ptr;
  g_param_params[g_param_num_params].storage = storage;
  g_param_num_params++;

  *ptr = default_value;

  if (storage == PARAM_PERSISTENT)
  {
    param_value_t value;
    if (param_find_flash_value(name, PARAM_TYPE_FLOAT, &value))
    {
      printf("found param [%s] in flash: %f\r\n", name, value.f);
      *ptr = value.f;
    }
    else
    {
      printf("couldn't find param [%s] in flash\r\n", name);
    }
  }
}

const char *param_get_name(const uint32_t param_idx)
{
  if (param_idx >= g_param_num_params)
    return NULL;
  return g_param_params[param_idx].n;
}

param_type_t param_get_type(const uint32_t param_idx)
{
  if (param_idx >= g_param_num_params)
    return PARAM_TYPE_INVALID;
  return g_param_params[param_idx].t;
}

volatile void *param_get_ptr(const uint32_t param_idx)
{
  if (param_idx >= g_param_num_params)
    return NULL;
  return g_param_params[param_idx].p;
}

param_storage_t param_get_storage(const uint32_t param_idx)
{
  if (param_idx >= g_param_num_params)
    return PARAM_TRANSIENT;
  return g_param_params[param_idx].storage;
}

void param_set_int(const char *param_name, const int value)
{
  for (uint32_t i = 0; i < g_param_num_params; i++)
  {
    struct param *p = &g_param_params[i];
    if (p->t == PARAM_TYPE_INT && !strcmp(p->n, param_name))
    {
      *((int *)p->p) = value;
      break;
    }
  }
}

void param_set_float(const char *param_name, const float value)
{
  for (uint32_t i = 0; i < g_param_num_params; i++)
  {
    struct param *p = &g_param_params[i];
    if (p->t == PARAM_TYPE_FLOAT && !strcmp(p->n, param_name))
    {
      *((float *)p->p) = value;
      break;
    }
  }
}

void param_load_from_flash()
{
  printf("param_load_from_flash()\r\n");
  printf("done\r\n");
}

void param_save_to_flash()
{
  printf("param_save_to_flash()\r\n");
  // todo: be smart, pre-calculate the flash table size, and erase
  // enough pages to get that all done before writing starts.
  // for now, it's all hard-coded in the flash driver
  flash_erase_range(
      flash_get_param_table_base_addr(),
      flash_get_param_table_size());

  uint32_t flash_write_addr = flash_get_param_table_base_addr();

  // first, write the number of params that the reader (in the future)
  // can expect to find
  uint32_t num_persistent_params = 0;
  for (int i = 0; i < g_param_num_params; i++)
    if (g_param_params[i].storage == PARAM_PERSISTENT)
      num_persistent_params++;

  if (!flash_program_dword(flash_write_addr, num_persistent_params, 0x42))
  {
    printf("woah! error programming table length\n");
    return;
  }
  flash_write_addr += 8;

  // now walk through the table, writing the persistent entries
  for (uint32_t i = 0; i < g_param_num_params; i++)
  {
    if (g_param_params[i].storage != PARAM_PERSISTENT)
      continue;
    printf(
        "writing param %d at 0x%08x\r\n",
        (int)i,
        (unsigned)flash_write_addr);
    const uint8_t type_byte = (uint8_t)g_param_params[i].t;
    if (!flash_program_byte(flash_write_addr, type_byte))
    {
      printf("woah! error programming param %d type byte\n", (int)i);
      return;
    }
    flash_write_addr++;
    // now write the null-terminated name string
    for (const char *name_rptr = g_param_params[i].n;
        *name_rptr; name_rptr++)
    {
      if (!flash_program_byte(flash_write_addr, *name_rptr))
      {
        printf("woah! error programming param %d name\n", (int)i);
        return;
      }
      flash_write_addr++;
    }
    // null-terminate the name string
    if (!flash_program_byte(flash_write_addr, 0))
    {
      printf("woah! error null-terminating param %d name\n", (int)i);
      return;
    }
    flash_write_addr++;
    // write the 4 byte value
    volatile uint8_t *pval = g_param_params[i].p;
    for (int b_idx = 0; b_idx < 4; b_idx++)
    {
      flash_program_byte(flash_write_addr, pval[b_idx]);
      flash_write_addr++;
    }
  }

  printf("done\r\n");
}

bool param_find_flash_value(
    const char *name,
    const int type,
    param_value_t *value)
{
  if (!value || !name)  // come on, man
    return false;

  const size_t name_len = strlen(name);
  if (name_len == 0)  // come on, man
    return false;

  // walk through the param table and see if we can find this param
  uint32_t flash_read_addr = flash_get_param_table_base_addr();

  uint32_t num_persistent_params = flash_read_word(flash_read_addr);
  if (num_persistent_params > 0xffff)  // empty param table... it's nonsense
    return false;

  flash_read_addr += 4;

  for (int param_idx = 0; param_idx < num_persistent_params; param_idx++)
  {
    const uint8_t type_byte = flash_read_byte(flash_read_addr);
    flash_read_addr++;
    
    bool found = true;
    for (int name_char_idx = 0; name_char_idx < name_len; name_char_idx++)
    {
      const char flash_name_char =
          flash_read_byte(flash_read_addr + name_char_idx);
      if (flash_name_char != name[name_char_idx])
      {
        found = false;
        break;
      }
    }

    // ensure the flash has a NULL at the right place (same length string)
    if (flash_read_byte(flash_read_addr + name_len) != 0)
      found = false;

    // advance the read pointer up to the NULL char in flash
    while (true)
    {
      const uint8_t name_char = flash_read_byte(flash_read_addr);
      flash_read_addr++;
      if (name_char == 0)
        break;
    }

    if (type_byte != type)
      found = false;

    if (!found)
    {
      flash_read_addr += 4;
      continue;  // this wasn't it. move on to the next stored param
    }

    value->u32 =
        (flash_read_byte(flash_read_addr + 0) <<  0) |
        (flash_read_byte(flash_read_addr + 1) <<  8) |
        (flash_read_byte(flash_read_addr + 2) << 16) |
        (flash_read_byte(flash_read_addr + 3) << 24) ;

    return true;
  }
  return false;
}
