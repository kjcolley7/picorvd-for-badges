#include "gdb_task.h"
#include "Application.h"
#include <FreeRTOS.h>
#include <task.h>
#include "tusb.h"
#include "usb_itf.h"


void vTaskGdb(__unused void *pvParams) {
    vTaskDelay(pdMS_TO_TICKS(500));

    for (;;) {
        bool connected = tud_cdc_n_connected(ITF_GDB);
        char rx = 0;
        bool rx_enable = tud_cdc_n_available(ITF_GDB); // this "available" check is required for some reason
        char tx = 0;
        bool tx_enable = 0;

        if (rx_enable) {
            tud_cdc_n_read(ITF_GDB, &rx, 1);
        }

        gApp->gdb->update(connected, rx_enable, rx, tx_enable, tx);

        if (tx_enable) {
            tud_cdc_n_write(ITF_GDB, &tx, 1);
            tud_cdc_n_write_flush(ITF_GDB);
        }

        // Idle pass: nothing came in, nothing went out. Yield, because this
        // loop has no blocking call anywhere in it, and at priority 2 on a
        // two-core SMP scheduler that means it permanently owns a core.
        // That was survivable until the factory loop started writing its
        // flash log: pico_flash's FreeRTOS SMP helper parks a max-priority
        // task on the *other* core with interrupts off, which evicts this
        // spinner onto the writer's core, where it starves the priority-1
        // writer (and the priority-1 status LED) indefinitely -- the writer
        // only got the CPU back when some unrelated console activity
        // reshuffled the run queues. Symptom on the bench: after each
        // flashed board the loop went silent and the LED froze completely
        // solid until the host typed anything. One tick of sleep when idle
        // costs a GDB session nothing measurable.
        if (!rx_enable && !tx_enable) {
            vTaskDelay(1);
        }
    }
}
