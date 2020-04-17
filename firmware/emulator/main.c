#include <unistd.h>
#include <stdbool.h>
#include "comms.h"
#include "uuid.h"
#include "param.h"
#include "control.h"
#include "multicast.h"

int main(int argc, char **argv)
{
  multicast_init();
  uuid_init();
  param_init();
  control_init();
  comms_set_raw_tx_fptr(multicast_tx);

  while (true)
  {
    multicast_listen(0);
    usleep(10000);
  }
  return 0;
}
