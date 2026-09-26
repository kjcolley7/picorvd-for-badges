#pragma once

#include <stdint.h>

// Startup progress markers, stashed in a watchdog scratch register so that a
// boot which hangs can still say where it got to -- the next boot reads it back.
// See failsafe_arm() in main.cpp.
enum {
    BOOT_CP_NONE       = 0,
    BOOT_CP_MAIN       = 1,   // entered main()
    BOOT_CP_SERIAL_ID  = 2,   // read the flash unique id
    BOOT_CP_SYSCLK     = 3,   // switched to 125MHz
    BOOT_CP_STDIO      = 4,   // stdio_init_all() returned
    BOOT_CP_BOARD_INIT = 5,   // board_init() returned
    BOOT_CP_SWIO       = 10,  // PicoSWIO up
    BOOT_CP_SWIO_RESET = 11,  // PicoSWIO::reset() returned
    BOOT_CP_RVDEBUG    = 12,  // RVDebug::init() returned
    BOOT_CP_FLASH      = 13,  // WCHFlash::reset() returned
    BOOT_CP_SOFTBREAK  = 14,  // SoftBreak::init() returned
    BOOT_CP_GDBSERVER  = 15,  // GDBServer::reset() returned
    BOOT_CP_APP_INIT   = 6,   // application_init() returned
    BOOT_CP_TASKS      = 7,   // all tasks created
    BOOT_CP_SCHEDULER  = 8,   // vTaskStartScheduler() called
    BOOT_CP_USB_TASK   = 20,  // vTaskUsb entered
    BOOT_CP_TUSB_INIT  = 21,  // tusb_init() returned
};

#ifdef __cplusplus
extern "C" {
#endif

void boot_checkpoint(uint32_t n);
void failsafe_disarm(void);
extern uint32_t g_prev_boot_checkpoint;
const char *boot_checkpoint_name(uint32_t n);


// Stamped reboots. A reboot the firmware chooses to perform leaves its reason
// in watchdog scratch[2] as 0xDD | kind | detail, which survives the reset;
// failsafe_arm() captures it and the shell reports it on the next console
// connect. Kinds 1 and 2 are raised by the FreeRTOS fatal hooks, 3-6 by
// stamped_reboot(), and 7 is synthesised at boot when the hardware watchdog
// itself fired -- meaning nothing, not even the watchdog task, was running.
// Kind 8 is also stamped_reboot(), after a factory image update.
enum {
    CRASH_KIND_STACK_OVERFLOW = 1,
    CRASH_KIND_OUT_OF_MEMORY  = 2,
    CRASH_KIND_SHELL_WEDGED   = 3,
    CRASH_KIND_USB_WEDGED     = 4,
    CRASH_KIND_TIER1_STARVED  = 5,
    CRASH_KIND_RESTART_REQ    = 6,
    CRASH_KIND_HW_WATCHDOG    = 7,
    CRASH_KIND_IMAGE_UPDATED  = 8,   // not a crash: new factory image stored
};
void stamped_reboot(uint32_t kind);

#ifdef __cplusplus
}
#endif
