#include "rng.h"
#include <stdlib.h>
#include <time.h>

void rnd_init()
{
  srandom(time(NULL));
}

uint32_t rnd_read()
{
  // make sure we get a full 32-bit random, to be like the STM32 RNG
  // otherwise we might just get a 15-bit random number and be sad.
  uint32_t rands[4] = {0};
  for (int i = 0; i < 4; i++)
    rands[i] = random() & 0xff;
  return rands[0] | (rands[1] << 8) | (rands[2] << 16) | (rands[3] << 24);
}
