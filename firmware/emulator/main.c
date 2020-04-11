#include <stdbool.h>
#include "comms.h"
#include "multicast.h"

int main(int argc, char **argv)
{
  multicast_init();
  comms_set_raw_tx_fptr(multicast_tx);
  const char *pkt = "hello";
  while (true)
  {
    multicast_listen(100000);
    multicast_tx(pkt, 5);
  }
  return 0;
}
