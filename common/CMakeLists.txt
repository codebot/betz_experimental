add_library(common
  stack.c
  startup.c
  stm32f4_vectors.c)

target_link_libraries(common m)
