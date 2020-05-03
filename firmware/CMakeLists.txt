set(CMAKE_TOOLCHAIN_FILE "stm32-toolchain.cmake")

cmake_minimum_required(VERSION 3.0.0)
project(betzdrive)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting CMAKE_BUILD_TYPE explicitly")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(openocd_preamble openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/olimex-arm-jtag-swd.cfg -f target/stm32f4x.cfg)

function(betzdrive_raw_executable stem)
  add_executable(${stem}.elf ${ARGN})
  set_target_properties(${stem}.elf PROPERTIES LINK_FLAGS "\
    -lc \
    -lgcc \
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/map,--cref \
    -T ${PROJECT_SOURCE_DIR}/common/stm32f405_bl.ld \
")
  target_compile_definitions(${stem}.elf PUBLIC IS_BOOTLOADER)
  target_link_libraries(${stem}.elf common)
  add_custom_command(TARGET ${stem}.elf POST_BUILD
      COMMAND arm-none-eabi-objcopy -O binary ${stem}.elf ${stem}.bin
      COMMENT "creating ${stem}.bin")
  add_custom_command(TARGET ${stem}.elf POST_BUILD
      COMMAND arm-none-eabi-objdump -S -d ${stem}.elf > ${stem}.objdump
      COMMENT "disassembling ${stem}.elf")
  add_custom_target(${stem}.flash_swd
      DEPENDS ${stem}.elf
      COMMAND ${openocd_preamble} -c "init; sleep 100; halt; sleep 100; flash write_image erase ${stem}.bin 0x08000000; verify_image ${stem}.bin 0x08000000; sleep 100; reset run; sleep 100; shutdown"
      VERBATIM)
  add_custom_target(${stem}.gdb
      DEPENDS ${stem}.elf
      COMMAND arm-none-eabi-gdb ${stem}.elf -x ${PROJECT_SOURCE_DIR}/common/gdb_init_commands
      VERBATIM)
endfunction()

function(betzdrive_application stem)
  add_executable(${stem}.elf ${ARGN})
  set_target_properties(${stem}.elf PROPERTIES LINK_FLAGS "\
    -lc \
    -lgcc \
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/map,--cref \
    -T ${PROJECT_SOURCE_DIR}/common/stm32f405_app.ld \
")
  target_link_libraries(${stem}.elf common)
  add_custom_command(TARGET ${stem}.elf POST_BUILD
      COMMAND arm-none-eabi-objcopy -O binary ${stem}.elf ${stem}.bin
      COMMENT "creating ${stem}.bin")
  add_custom_command(TARGET ${stem}.elf POST_BUILD
      COMMAND arm-none-eabi-objdump -S -d ${stem}.elf > ${stem}.objdump
      COMMENT "disassembling ${stem}.elf")
  add_custom_target(${stem}.gdb
      DEPENDS ${stem}.elf
      COMMAND arm-none-eabi-gdb ${stem}.elf -x ${PROJECT_SOURCE_DIR}/common/gdb_init_commands
      VERBATIM)
endfunction()

add_custom_target(reset_mcu
    DEPENDS test_blink.elf
    COMMAND ${openocd_preamble} -c "init; halt; reset run; shutdown"
    VERBATIM)

add_custom_target(gdb_server
    DEPENDS test_blink.elf
    COMMAND ${openocd_preamble} -c "init; halt"
    VERBATIM)

include_directories(common)
include_directories(common/stm32)

add_subdirectory(common)
add_subdirectory(test_blink)
add_subdirectory(test_console)
add_subdirectory(test_rs485)
add_subdirectory(bootloader)
add_subdirectory(test_systime)
add_subdirectory(test_uuid)
add_subdirectory(test_rng)

add_subdirectory(app_blink)
