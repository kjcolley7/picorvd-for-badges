#include <FreeRTOS.h>
#include <task.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/usb_reset_config.h"
#include "hardware/watchdog.h"
#include "tusb.h"
#include "usb_itf.h"
#include "boot_checkpoints.h"
#include "msc_disk.h"

// Opening the console port at this baud rate restarts the firmware, as opposed
// to PICO_USB_RESET_MAGIC_BAUD_RATE (1200) which drops to the BOOTSEL loader.
//
// This is the escape hatch for a wedged shell. It works precisely because this
// task runs at a higher priority than the shell (3 vs 2), so a shell stuck in a
// command still leaves USB being serviced -- the console stops echoing, but the
// line-coding change is still seen here. A restart re-runs Application::init(),
// which calls PicoSWIO::reset() and re-establishes the target's debug
// interface, i.e. exactly what physically replugging the Pico achieves.
#define PICORVD_RESTART_MAGIC_BAUD_RATE 2400

// Liveness beacon for vTaskWatchdog: incremented every tud_task pass. If the
// USB stack ever deadlocks or the task starves, the probe is unreachable in
// every way that matters (console, gdb, even the baud-rate reset tricks), so
// the watchdog turns that state into a quick reboot instead of a brick.
volatile uint32_t g_usb_heartbeat = 0;

void vTaskUsb(__unused void *pvParams) {
    boot_checkpoint(BOOT_CP_USB_TASK);
    msc_disk_init();
    tusb_init();
    boot_checkpoint(BOOT_CP_TUSB_INIT);

    bool failsafe_live = true;

    for (;;) {
        tud_task();
        g_usb_heartbeat++;

        // The boot failsafe resets the probe if startup never gets this far.
        // Once the host has enumerated us the console is reachable and the
        // 1200-baud BOOTSEL trigger works, so it has nothing left to protect.
        if (failsafe_live) {
            if (tud_mounted()) {
                failsafe_disarm();
                failsafe_live = false;
            }
            else {
                watchdog_update();
            }
        }

        cdc_line_coding_t lc;

        tud_cdc_n_get_line_coding(ITF_CONSOLE, &lc);
        if (lc.bit_rate == PICO_USB_RESET_MAGIC_BAUD_RATE) {

            printf("\n\nPerform reboot into the BOOTSEL\n\n");
            tud_cdc_n_write_flush(ITF_CONSOLE);

            reset_usb_boot(0, PICO_USB_RESET_BOOTSEL_INTERFACE_DISABLE_MASK);
        }

        if (lc.bit_rate == PICORVD_RESTART_MAGIC_BAUD_RATE) {

            printf("\n\nRestarting firmware\n\n");
            tud_cdc_n_write_flush(ITF_CONSOLE);

            // Give the flush a moment to reach the host before the reset.
            vTaskDelay(pdMS_TO_TICKS(50));

            stamped_reboot(CRASH_KIND_RESTART_REQ);
        }

        vTaskDelay(1);
    }
}
