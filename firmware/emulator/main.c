#include <unistd.h>
#include <stdbool.h>

#include "comms.h"
#include "control.h"
#include "flash.h"
#include "multicast.h"
#include "param.h"
#include "rng.h"
#include "state.h"
#include "uuid.h"

int main(int argc, char **argv)
{
  flash_init();
  rng_init();
  multicast_init();
  uuid_init();
  state_init();
  param_init();
  control_init();
  comms_init(multicast_tx);
  comms_set_bootloader_mode(true);

  while (true)
  {
    multicast_listen(0);
    comms_tick();
    usleep(10000);
  }
  return 0;
}
