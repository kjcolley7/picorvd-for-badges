# Locate the FreeRTOS kernel and add its RP2040 SMP port. include() this
# before project().
#
# The kernel normally comes from the FreeRTOS-Kernel submodule, which is
# fetched on first use if the checkout is empty. To build against a kernel
# elsewhere, pass -DFREERTOS_KERNEL_PATH=<dir> to cmake or set the
# FREERTOS_KERNEL_PATH environment variable before the first configure (the
# value is cached per build directory).

if (NOT FREERTOS_KERNEL_PATH AND DEFINED ENV{FREERTOS_KERNEL_PATH})
    set(FREERTOS_KERNEL_PATH "$ENV{FREERTOS_KERNEL_PATH}")
endif()
set(_FREERTOS_KERNEL_SUBMODULE "${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel")
if (NOT FREERTOS_KERNEL_PATH)
    set(FREERTOS_KERNEL_PATH "${_FREERTOS_KERNEL_SUBMODULE}")
endif()
if (FREERTOS_KERNEL_PATH STREQUAL _FREERTOS_KERNEL_SUBMODULE)
    init_submodule_if_needed("${CMAKE_CURRENT_LIST_DIR}" FreeRTOS-Kernel portable/ThirdParty/GCC/RP2040/CMakeLists.txt)
endif()
get_filename_component(FREERTOS_KERNEL_PATH "${FREERTOS_KERNEL_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
set(FREERTOS_KERNEL_PATH "${FREERTOS_KERNEL_PATH}" CACHE PATH "Path to the FreeRTOS kernel" FORCE)

set(FREERTOS_KERNEL_RP2040_PORT "${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040")
if (NOT EXISTS "${FREERTOS_KERNEL_RP2040_PORT}/CMakeLists.txt")
    message(FATAL_ERROR
        "No FreeRTOS RP2040 port at ${FREERTOS_KERNEL_RP2040_PORT}. "
        "Run 'git submodule update --init' or set FREERTOS_KERNEL_PATH.")
endif()
message(STATUS "FreeRTOS kernel: ${FREERTOS_KERNEL_PATH}")

add_subdirectory("${FREERTOS_KERNEL_RP2040_PORT}" FREERTOS_KERNEL)
