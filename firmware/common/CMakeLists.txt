add_library(common
  console.c
  delay.c
  flash.c
  param.c
  parser.c
  pin.c
  rs485.c
  stack.c
  startup.c
  status_led.c
  stm32f4_vectors.c
  stubs.c
  systime.c
  uuid.c
)

target_link_libraries(common m)
