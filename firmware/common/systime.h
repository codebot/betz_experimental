#ifndef SYSTIME_H
#define SYSTIME_H

#include "stm32f405xx.h"

void systime_init();
#define SYSTIME (TIM2->CNT)

#endif

