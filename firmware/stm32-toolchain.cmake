set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_VERSION   1)
set(CMAKE_SYSTEM_PROCESSOR arm-eabi)

set(CMAKE_C_COMPILER       arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER     arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER     arm-none-eabi-as)
set(CMAKE_OBJCOPY          arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP          arm-none-eabi-objdump)

set(ARCH_FLAGS "-mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16")

SET(CMAKE_C_FLAGS
  "-g ${ARCH_FLAGS} -fno-builtin -Wall -Werror -I. -std=gnu99 -fdata-sections -ffunction-sections -MD -DSTM32F405"
  CACHE INTERNAL "c compiler flags")

# cxx flags only used for cmake configure step; c++ stuff isn't set up in
# linker script to actually work, I don't think. We're just using C.
SET(CMAKE_CXX_FLAGS
  "-g ${ARCH_FLAGS} -fno-builtin -Wall -Werror -I. -fdata-sections -ffunction-sections -MD -DSTM32F405"
  CACHE INTERNAL "c compiler flags")


SET(CMAKE_ASM_FLAGS
  "${ARCH_FLAGS}"
  CACHE INTERNAL "asm compiler flags")

SET(CMAKE_EXE_LINKER_FLAGS
  "-g ${ARCH_FLAGS} --specs=nosys.specs"
  CACHE INTERNAL "exe link flags")
