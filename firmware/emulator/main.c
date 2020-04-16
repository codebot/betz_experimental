#include <unistd.h>
#include <stdbool.h>
#include "comms.h"
#include "multicast.h"

int main(int argc, char **argv)
{
  multicast_init();
  comms_set_raw_tx_fptr(multicast_tx);
  while (true)
  {
    multicast_listen(0);
    usleep(100000);
  }
  return 0;
}
