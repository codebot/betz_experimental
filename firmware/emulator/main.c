#include <unistd.h>
#include <stdbool.h>

#include "comms.h"
#include "control.h"
#include "multicast.h"
#include "param.h"
#include "rng.h"
#include "uuid.h"

int main(int argc, char **argv)
{
  rng_init();
  multicast_init();
  uuid_init();
  param_init();
  control_init();
  comms_set_raw_tx_fptr(multicast_tx);

  while (true)
  {
    multicast_listen(0);
    comms_tick();
    usleep(10000);
  }
  return 0;
}
