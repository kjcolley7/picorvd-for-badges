#include "shell_task.h"
#include "shell_commands/commands.h"
#include "Application.h"
#include <FreeRTOS.h>
#include <task.h>
#include "tusb.h"
#include "usb_itf.h"
#include "boot_checkpoints.h"
#include <unistd.h>

volatile uint32_t g_shell_heartbeat = 0;

static void help(Console &console);

static const Console::Handler handlers[] = {
        {"help",          help},
        {"pico_clocks",   command_clocks},
        {"init_swio",     command_init_swio},
        {"reset",         command_reset},
        {"halt",          command_halt},
        {"resume",        command_resume},
        {"step",          command_step},
        {"dump",          command_dump},
        {"dump2",         command_dump2},
        {"status",        command_status},
        {"part_id",       command_part_id},
        {"swio_test",     command_swio_test},
        {"why",           command_why},
        {"chip_id",       command_chip_id},
        {"wipe_chip",     command_wipe},
        {"halt_on_reset", command_halt_on_reset},
        {"i2c_scan",      command_i2c_scan},
        {"i2c_w",         command_i2c_write},
        {"i2c_r",         command_i2c_read},
        {"sao_cmd",       command_sao_cmd},
        {"sao_wb",        command_sao_wb},
        {"sao_rb",        command_sao_rb},
        {"arp_udid",      command_arp_udid},
        {"arp_test",      command_arp_test},
        {"arp_enum",      command_arp_enum},
        {"sao_discover",  command_sao_discover},
        {"sao_log",       command_sao_log},
        {"sao_loop",      command_sao_loop},
        {"la_arm",        command_la_arm},
        {"la_stop",       command_la_stop},
        {"la_dump",       command_la_dump},
        {"la_raw",        command_la_raw},
        {"factory",       command_factory},
        {NULL, NULL},
};

void help(Console &console) {
    printf("Commands:\n");
    for (int i = 0;; i++) {
        if (!handlers[i].name || !handlers[i].handler) {
            break;
        }

        printf("  %s\n", handlers[i].name);
    }
}

static bool is_connected() {
    return tud_cdc_n_connected(ITF_CONSOLE);
}

static void wait_usb() {
    vTaskDelay(pdMS_TO_TICKS(500));
    while (!is_connected()) {
        // Beat here too, otherwise sitting unplugged looks like a wedge.
        g_shell_heartbeat++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

void vTaskShell(__unused void *pvParams) {
    printf(COLOR_GREEN("// Starting shell") "\n");
    auto *console = new Console(handlers);

    vTaskDelay(pdMS_TO_TICKS(500));

    for (;;) {
        wait_usb();

        // A fatal-error reboot could not say so at the time, so say it now.
        {
            extern uint32_t g_prev_crash_stamp;
            if ((g_prev_crash_stamp & 0xFF000000u) == 0xDD000000u) {
                switch ((g_prev_crash_stamp >> 16) & 0xFFu) {
                case CRASH_KIND_STACK_OVERFLOW:
                    printf(COLOR_RED("// previous run died: STACK OVERFLOW in task '%c%c...'") "\n",
                           (g_prev_crash_stamp & 0xFF) ? (char)(g_prev_crash_stamp & 0xFF) : '?',
                           ((g_prev_crash_stamp >> 8) & 0xFF) ? (char)((g_prev_crash_stamp >> 8) & 0xFF) : '?');
                    break;
                case CRASH_KIND_OUT_OF_MEMORY:
                    printf(COLOR_RED("// previous run died: FreeRTOS heap exhausted") "\n");
                    break;
                case CRASH_KIND_SHELL_WEDGED:
                    printf(COLOR_RED("// previous run died: shell task wedged (software watchdog)") "\n");
                    break;
                case CRASH_KIND_USB_WEDGED:
                    printf(COLOR_RED("// previous run died: USB task wedged (software watchdog)") "\n");
                    break;
                case CRASH_KIND_TIER1_STARVED:
                    printf(COLOR_RED("// previous run died: priority-1 tasks (factory loop, LED) starved (software watchdog)") "\n");
                    break;
                case CRASH_KIND_RESTART_REQ:
                    printf("// previous run: restart requested over the console (2400 baud)\n");
                    break;
                case CRASH_KIND_IMAGE_UPDATED:
                    printf("// previous run: factory image updated over the PICORVD drive\n");
                    break;
                case CRASH_KIND_HW_WATCHDOG:
                    printf(COLOR_RED("// previous run died: HARDWARE watchdog -- no task ran at all") "\n");
                    break;
                default:
                    printf(COLOR_RED("// previous run died: unknown stamp %08lX") "\n",
                           (unsigned long)g_prev_crash_stamp);
                    break;
                }
            }
        }

        // A boot that hung could not say so at the time, so say it now.
        if (g_prev_boot_checkpoint) {
            printf(COLOR_RED("// previous boot died at checkpoint %lu (%s)") "\n",
                   (unsigned long)g_prev_boot_checkpoint,
                   boot_checkpoint_name(g_prev_boot_checkpoint));
        }

        // Both heaps, because the C one runs much closer to the edge than it
        // looks: Console and GDBServer take 48KB of packet buffers out of it.
        {
            extern char end, __HeapLimit;
            char *brk = (char *)sbrk(0);
            printf("// heap: C %u used / %u free, FreeRTOS %u free\n",
                   (unsigned)(brk - &end), (unsigned)(&__HeapLimit - brk),
                   (unsigned)xPortGetFreeHeapSize());
        }

        console->reset();
        console->start();

        while (is_connected()) {
            char rx;
            if (tud_cdc_n_read(ITF_CONSOLE, &rx, sizeof(rx)) > 0) {
                console->update(rx);
            }

            g_shell_heartbeat++;
            vTaskDelay(1);
        }
    }
}
