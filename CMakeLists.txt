# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Default board for the interactive probe: an original Pi Pico. Board choice
# is per build directory -- the standalone programmer boards are Waveshare
# RP2040-Zeros (no plain LED; WS2812 on GP16):
#   cmake -B build-zero -DPICO_BOARD=waveshare_rp2040_zero
# A custom yd_rp2040 board definition (Pico plus WS2812/USR key) also lives
# in src/boards for anyone building on that clone.
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico)
endif()

list(APPEND PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/src/boards)

include(pico_sdk_import.cmake)
include(scripts/functions.cmake)
include(FreeRTOS_Kernel_import.cmake)

project(pico_rvd C CXX ASM)

pico_sdk_init()

add_subdirectory(src/shell)

# SAOv3 host library (saoh_core from SAOv3-lib); override with -DSAOH_CORE_DIR=...
set(SAOH_CORE_DIR "$ENV{HOME}/Projects/Embedded/RareCircuits/saov3/lib/host/saoh_core"
    CACHE PATH "Path to SAOv3-lib's host/saoh_core directory")

file(
        GLOB SOURCES_SHELL_COMMANDS
        src/shell_commands/*.cpp
)

set(
        SOURCES_PICORVD
        src/main.cpp
        src/application/Application.cpp

        src/usb/usb_descriptors.c

        src/usb/usb_task.cpp
        src/usb/msc_disk.cpp
        src/gdb/gdb_task.cpp
        src/shell_task.cpp

        ${SOURCES_SHELL_COMMANDS}

        src/chip_tools/PicoSWIO.cpp
        src/chip_tools/RVDebug.cpp
        src/chip_tools/SoftBreak.cpp
        src/chip_tools/WCHFlash.cpp
        src/chip_tools/GDBServer.cpp
        src/chip_tools/utils.cpp
        src/chip_tools/Packet2.cpp

        # SAOv3 host-side library, so discovery uses the reference
        # implementation rather than a hand-rolled approximation.
        src/shell_commands/saoh_pico_hal.c
        ${SAOH_CORE_DIR}/src/discovery.c
        ${SAOH_CORE_DIR}/src/err.c
        ${SAOH_CORE_DIR}/src/itf.c
        ${SAOH_CORE_DIR}/src/log.c
        ${SAOH_CORE_DIR}/src/smbus_i2c_shim.c
)

# Two builds of the same firmware. pico_rvd is the interactive probe;
# pico_rvd_factory is the standalone pass-around programmer, which starts the
# factory flashing loop at boot (powered by the target it is plugged into) and
# keeps its production log in its own flash.
foreach(TARGET IN ITEMS pico_rvd pico_rvd_factory)
    add_executable(${TARGET} ${SOURCES_PICORVD})

    pico_generate_pio_header(${TARGET} ${CMAKE_CURRENT_LIST_DIR}/src/chip_tools/singlewire.pio)
    pico_generate_pio_header(${TARGET} ${CMAKE_CURRENT_LIST_DIR}/src/shell_commands/la_capture.pio)
endforeach()

target_compile_definitions(pico_rvd_factory PRIVATE FACTORY_STANDALONE=1)

include_directories(
        ${SAOH_CORE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/shell_commands
        ${CMAKE_CURRENT_SOURCE_DIR}/src/config
        ${CMAKE_CURRENT_SOURCE_DIR}/src/application
        ${CMAKE_CURRENT_SOURCE_DIR}/src/usb
        ${CMAKE_CURRENT_SOURCE_DIR}/src/chip_tools
)

foreach(TARGET IN ITEMS pico_rvd pico_rvd_factory)
    pico_enable_stdio_usb(${TARGET} 1)

    pico_enable_stdio_uart(${TARGET} 1)

    target_link_libraries(
            ${TARGET}
            shell
            pico_stdlib
            pico_stdio_usb
            pico_bootsel_via_double_reset
            pico_flash
            hardware_flash
            hardware_pio
            hardware_dma
            hardware_i2c
            hardware_clocks
            tinyusb_device
            FreeRTOS-Kernel
            FreeRTOS-Kernel-Heap4
    )
endforeach()

set_source_files_properties(
        ${SAOH_CORE_DIR}/src/discovery.c
        ${SAOH_CORE_DIR}/src/err.c
        ${SAOH_CORE_DIR}/src/itf.c
        ${SAOH_CORE_DIR}/src/log.c
        ${SAOH_CORE_DIR}/src/smbus_i2c_shim.c
        PROPERTIES COMPILE_OPTIONS
        "-include;${CMAKE_CURRENT_SOURCE_DIR}/src/shell_commands/saoh_log_capture.h"
)

foreach(TARGET IN ITEMS pico_rvd pico_rvd_factory)
    pico_add_extra_outputs(${TARGET})
    add_deploy_target(${TARGET} "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.uf2")
endforeach()

add_subdirectory(examples)