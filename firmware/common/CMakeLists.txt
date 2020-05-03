add_library(common
  comms.c
  console.c
  control.c
  delay.c
  flash.c
  param.c
  pin.c
  rng.c
  rs485.c
  stack.c
  startup.c
  status_led.c
  stm32f4_vectors.c
  stubs.c
  sys.c
  systime.c
  uuid.c
)

target_link_libraries(common m)
