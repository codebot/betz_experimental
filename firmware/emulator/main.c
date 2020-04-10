#include <stdbool.h>
#include "comms.h"
#include "mcast_udp.h"

int main(int argc, char **argv)
{
  mcast_udp_init();
  comms_set_raw_tx_fptr(mcast_udp_tx);
  const char *pkt = "hello";
  while (true)
  {
    mcast_udp_listen(100000);
    mcast_udp_tx(pkt, 5);
  }
  return 0;
}
