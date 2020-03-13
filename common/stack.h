#ifndef STACK_H
#define STACK_H

#include <stdint.h>

// this processor only has 128K of RAM, so we'd better go easy on the stack.
#ifndef STACK_SIZE
#  define STACK_SIZE 16384
#endif

extern volatile uint8_t g_stack[STACK_SIZE];

#endif
