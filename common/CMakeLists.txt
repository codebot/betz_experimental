add_library(common
  console.c
  pin.c
  rs485.c
  stack.c
  startup.c
  status_led.c
  stm32f4_vectors.c
  stubs.c)

target_link_libraries(common m)
