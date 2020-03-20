add_library(common
  pin.c
  stack.c
  startup.c
  status_led.c
  stm32f4_vectors.c)

target_link_libraries(common m)
