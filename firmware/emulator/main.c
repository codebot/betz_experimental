#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "comms.h"
#include "control.h"
#include "flash.h"
#include "multicast.h"
#include "param.h"
#include "rng.h"
#include "state.h"
#include "uuid.h"
#include "systime.h"

void emulator_physics_init()
{
  g_state.enc = 0;
  srandom(0);
}

void emulator_physics_tick()
{
  float denc = ((random() % 1001 - 500) * 0.001) * 0.01;
  g_state.enc += denc;
  if (g_state.enc < 0)
    g_state.enc += 2 * M_PI;
  else if (g_state.enc >= 2 * M_PI)
    g_state.enc -= 2 * M_PI;
}

int main(int argc, char **argv)
{
  flash_init();
  systime_init();
  rng_init();

  if (!multicast_init())
    return 1;

  uuid_init();
  state_init();
  param_init();
  control_init();
  comms_init(multicast_tx);
  comms_set_bootloader_mode(true);

  while (true)
  {
    multicast_listen(0);
    emulator_physics_tick();
    comms_tick();
    control_tick();
    usleep(10000);
  }
  return 0;
}
