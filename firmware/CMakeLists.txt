set(CMAKE_TOOLCHAIN_FILE "stm32-toolchain.cmake")

cmake_minimum_required(VERSION 3.0.0)
project(betzdrive)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting CMAKE_BUILD_TYPE explicitly")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(blue_openocd_preamble openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/olimex-arm-jtag-swd.cfg -f target/stm32f4x.cfg)
set(mini_openocd_preamble openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/olimex-arm-jtag-swd.cfg -f target/stm32g4x.cfg)

function(betzdrive_board_raw_executable board stem)
  message("betzdrive_board_raw_executable(${board}, ${stem})")
  add_executable(${stem}.${board}.elf ${ARGN})
  set_target_properties(${stem}.${board}.elf PROPERTIES LINK_FLAGS "\
    -lc \
    -lgcc \
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/map,--cref \
    -T ${PROJECT_SOURCE_DIR}/common/${board}_bl.ld \
")
  target_compile_definitions(${stem}.${board}.elf PUBLIC IS_BOOTLOADER "BOARD_${board}")
  target_link_libraries(${stem}.${board}.elf common.${board})
  add_custom_command(TARGET ${stem}.${board}.elf POST_BUILD
      COMMAND arm-none-eabi-objcopy -O binary ${stem}.${board}.elf ${stem}.${board}.bin
      COMMENT "creating ${stem}.${board}.bin")
  add_custom_command(TARGET ${stem}.${board}.elf POST_BUILD
      COMMAND arm-none-eabi-objdump -S -d ${stem}.${board}.elf > ${stem}.${board}.objdump
      COMMENT "disassembling ${stem}.${board}.elf")
  add_custom_target(${stem}.${board}.flash_swd
      DEPENDS ${stem}.${board}.elf
      COMMAND ${openocd_preamble} -c "init; sleep 100; halt; sleep 100; flash write_image erase ${stem}.${board}.bin 0x08000000; verify_image ${stem}.${board}.bin 0x08000000; sleep 100; reset run; sleep 100; shutdown"
      VERBATIM)
  add_custom_target(${stem}.${board}.gdb
      DEPENDS ${stem}.${board}.elf
      COMMAND gdb-multiarch ${stem}.${board}.elf -x ${PROJECT_SOURCE_DIR}/common/gdb_init_commands
      VERBATIM)
endfunction()

function(betzdrive_raw_executable stem)
  betzdrive_board_raw_executable(blue ${stem} ${ARGN})
  betzdrive_board_raw_executable(mini ${stem} ${ARGN})
endfunction()


function(betzdrive_board_application board stem)
  add_executable(${stem}.${board}.elf ${ARGN})
  set_target_properties(${stem}.${board}.elf PROPERTIES LINK_FLAGS "\
    -lc \
    -lgcc \
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/map,--cref \
    -T ${PROJECT_SOURCE_DIR}/common/${board}_app.ld \
")
  target_link_libraries(${stem}.${board}.elf common.${board})
  target_compile_definitions(${stem}.${board}.elf PUBLIC "BOARD_${board}")
  add_custom_command(TARGET ${stem}.${board}.elf POST_BUILD
      COMMAND arm-none-eabi-objcopy -O binary ${stem}.${board}.elf ${stem}.${board}.bin
      COMMENT "creating ${stem}.bin")
  add_custom_command(TARGET ${stem}.${board}.elf POST_BUILD
      COMMAND arm-none-eabi-objdump -S -d ${stem}.${board}.elf > ${stem}.${board}.objdump
      COMMENT "disassembling ${stem}.${board}.elf")
  add_custom_target(${stem}.${board}.gdb
      DEPENDS ${stem}.${board}.elf
      COMMAND gdb-multiarch ${stem}.${board}.elf -x ${PROJECT_SOURCE_DIR}/common/gdb_init_commands
      VERBATIM)
endfunction()

function(betzdrive_application stem)
  betzdrive_board_application(blue ${stem} ${ARGN})
  betzdrive_board_application(mini ${stem} ${ARGN})
endfunction()

function(betz_board_add_library board stem)
  add_library(${stem}.${board} ${ARGN})
  target_link_libraries(${stem}.${board} m)
  target_compile_definitions(${stem}.${board} PUBLIC "BOARD_${board}")
endfunction()

function(betz_add_library stem)
  betz_board_add_library(blue ${stem} ${ARGN})
  betz_board_add_library(mini ${stem} ${ARGN})
endfunction()

add_custom_target(reset_mcu.mini
    DEPENDS test_blink.mini.elf
    COMMAND ${mini_openocd_preamble} -c "debug_level 3; init; halt; reset run; shutdown"
    VERBATIM)

add_custom_target(reset_mcu.blue
    DEPENDS test_blink.blue.elf
    COMMAND ${blue_openocd_preamble} -c "debug_level 3; init; halt; reset run; shutdown"
    VERBATIM)

add_custom_target(gdb_server.mini
    DEPENDS test_blink.mini.elf
    COMMAND ${mini_openocd_preamble} -c "init; halt"
    VERBATIM)

add_custom_target(gdb_server.blue
    DEPENDS test_blink.blue.elf
    COMMAND ${blue_openocd_preamble} -c "init; halt"
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
add_subdirectory(app_test_adc)
add_subdirectory(app_test_enc)
add_subdirectory(app_test_pwm)
add_subdirectory(app_bldc)
