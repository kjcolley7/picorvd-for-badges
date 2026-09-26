#include <FreeRTOS.h>
#include <task.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "tusb.h"

#include "usb/usb_task.h"
#include "usb/tusb_config.h"
#include "usb/usb_itf.h"
#include "shell_task.h"
#include "gdb/gdb_task.h"

#include "Application.h"
#include "boot_checkpoints.h"
#include "pico/bootrom.h"
#include "factory.h"
#include "factory_log.h"
#include "usb/msc_disk.h"


// Loop counter for the status LED task, read by `factory where`: a frozen
// LED is the documented wedge signal, so this says whether the task behind
// it was actually starved or merely showing a mostly-on pattern.
volatile uint32_t g_led_beat = 0;

#if defined(PICO_DEFAULT_LED_PIN) || defined(PICO_DEFAULT_WS2812_PIN)

#ifndef PICO_DEFAULT_LED_PIN
// Boards like the Waveshare RP2040-Zero (the standalone programmer) carry no
// plain LED, only a WS2812. Bit-banged: 24 bits at ~1.25us each is ~30us with
// IRQs masked per 10ms refresh, which is cheaper than claiming a PIO state
// machine next to the SWIO and logic-analyzer programs. The color latches
// on the >50us of idle line before the next refresh. Runs from RAM: an XIP
// cache miss mid-frame (likelier while the other core reads flash) would
// stretch a bit and corrupt the color.
static void __no_inline_not_in_flash_func(ws2812_put)(uint32_t grb) {
    uint32_t save = save_and_disable_interrupts();
    for (int i = 23; i >= 0; i--) {
        bool one = (grb >> i) & 1;
        // Cycle counts assume the 125MHz configCPU_CLOCK_HZ, shaved a little
        // for loop overhead: T1H ~0.8us / T0H ~0.4us, ~1.25us per bit.
        gpio_put(PICO_DEFAULT_WS2812_PIN, 1);
        busy_wait_at_least_cycles(one ? 85 : 35);
        gpio_put(PICO_DEFAULT_WS2812_PIN, 0);
        busy_wait_at_least_cycles(one ? 45 : 95);
    }
    restore_interrupts(save);
}
#endif

// WS2812 words go out G, R, B. Levels stay low (the RP2040-Zero's LED is
// eye-watering near full scale, and the probe runs off the badge's supply).
// If red and green ever show up swapped, this packing is the thing to flip.
#define LED_RGB(r, g, b) (((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b))

void vTaskStatusLed(__unused void *pvParams) {
#ifdef PICO_DEFAULT_LED_PIN
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_LED_PIN, "On-board LED"));
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
#else
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_WS2812_PIN, "WS2812 status LED"));
    gpio_init(PICO_DEFAULT_WS2812_PIN);
    gpio_set_dir(PICO_DEFAULT_WS2812_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_WS2812_PIN, 0);
#endif

    for (;;) {
        g_led_beat++;
        // Factory patterns. On a standalone programmer this LED is the whole
        // user interface, so each state reads differently at a glance.
        //
        // A plain LED (the tethered Pico) carries the state in its blink
        // pattern alone; the WS2812 boards are color-coded instead. Either
        // way every pattern keeps moving: a frozen LED is the tell for a
        // wedged probe. The steady WS2812 states (waiting, booting, passed)
        // are therefore a 0.5 Hz, 80 percent duty heartbeat rather than solid.
        uint32_t ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        bool heartbeat = (ms % 2000) < 1600;
        bool blink;         // plain LED pattern
        bool lit;           // WS2812: on/off this tick
        uint32_t grb;       // WS2812 color
        switch (factory_led_state()) {
            case FACTORY_LED_WAITING:       // short blip / blue heartbeat
                blink = (ms % 500) < 100;
                lit = heartbeat;
                grb = LED_RGB(0, 0, 24);
                break;
            case FACTORY_LED_BUSY:          // fast blink / fast blinking red
                blink = (ms % 100) < 50;
                lit = blink;
                grb = LED_RGB(24, 0, 0);
                break;
            case FACTORY_LED_BOOTING:       // fast blink / orange heartbeat
                blink = (ms % 100) < 50;
                lit = heartbeat;
                grb = LED_RGB(24, 8, 0);
                break;
            case FACTORY_LED_OK:            // on w/ brief blip / green heartbeat
                blink = (ms % 1000) < 900;
                lit = heartbeat;
                grb = LED_RGB(0, 24, 0);
                break;
            case FACTORY_LED_FAIL:          // double blip / double blip, red
                blink = (ms % 1000) < 100 || ((ms % 1000) >= 250 && (ms % 1000) < 350);
                lit = blink;
                grb = LED_RGB(24, 0, 0);
                break;
            case FACTORY_LED_NO_IMAGE:      // slow blink / magenta heartbeat
                blink = (ms % 2000) < 1000;
                lit = heartbeat;
                grb = LED_RGB(16, 0, 16);
                break;
            case FACTORY_LED_OFF:
            default:
                blink = tud_cdc_n_connected(ITF_GDB);
                lit = blink;
                grb = LED_RGB(6, 6, 6);     // dim white
                break;
        }
#ifdef PICO_DEFAULT_LED_PIN
        (void)lit;
        (void)grb;
        gpio_put(PICO_DEFAULT_LED_PIN, blink);
#else
        (void)blink;
        ws2812_put(lit ? grb : 0);
#endif
        vTaskDelay(10);
    }
}

#endif

// Software watchdog: three heartbeats and a hardware backstop.
//
// The hardware watchdog alone will not do as a *shell* watchdog: its ceiling
// is ~8.4s, and halt_on_reset legitimately spins for 10s waiting for you to
// toggle the target's VCC. So the shell's heartbeat is watched from a task
// that runs at the same priority as USB -- high enough to keep being
// scheduled while the shell spins, and it yields with vTaskDelay so it costs
// nothing. Only a command that never returns can hold the counter still for
// the full timeout; the longest legitimate operation is halt_on_reset at
// 10s, so 30s distinguishes the two comfortably.
#define SHELL_WEDGE_TIMEOUT_MS 30000
// The USB task services interrupts-off-fast work only; if it makes no pass
// for this long, the stack is deadlocked and the probe is unreachable by any
// software means -- reboot fast so a factory operator just sees a blip.
#define USB_WEDGE_TIMEOUT_MS    5000
// The lowest application priority -- the factory loop and the status LED --
// has no heartbeat the two checks above would notice missing. On 2026-09-21
// that whole tier was starved for minutes at a stretch (one core parked
// with interrupts off by pico_flash's SMP lockout helper, the other owned
// by a priority-2 task that never yielded) while the shell answered
// commands and this watchdog saw nothing wrong. A priority-1 canary task
// now counts for it; see vTaskCanary. 30s, like the shell: no legitimate
// priority-2 activity can hold both cores for that long.
#define TIER1_STARVE_TIMEOUT_MS 30000
#define WATCHDOG_POLL_MS         500
// Once the boot failsafe hands over (failsafe_disarm), the hardware watchdog
// keeps running for the life of the firmware and vTaskWatchdog feeds it.
// That task running at all proves the scheduler is alive on some core; if it
// stops -- both cores with interrupts off, a fault handler spinning, XIP
// gone -- nothing software-side can help, and this is the way back. The
// RP2040's ceiling is ~8.4s.
#define HW_WATCHDOG_TIMEOUT_MS   8000

static volatile bool g_hw_watchdog_live = false;

// Fatal-error hooks. A stack overflow or heap exhaustion has already
// corrupted or starved something by the time it is detected, so the only
// sane move is: stamp what died into a watchdog scratch register (it
// survives the reboot; the shell reports it on the next console connect)
// and restart the probe.
uint32_t g_prev_crash_stamp = 0;

#define CRASH_MAGIC 0xDD000000u

// Layout: 0xDD | kind | name[1] | name[0]  (kind 1 = stack overflow, 2 = OOM)
extern "C" void vApplicationStackOverflowHook(TaskHandle_t task, char *name) {
    (void)task;
    // Two chars of task name is enough to tell shell/gdb/factory/usb apart.
    watchdog_hw->scratch[2] = CRASH_MAGIC | 0x00010000u
                            | ((uint32_t)(uint8_t)name[1] << 8)
                            | (uint32_t)(uint8_t)name[0];
    watchdog_reboot(0, 0, 0);
    for (;;) { }
}

extern "C" void vApplicationMallocFailedHook(void) {
    watchdog_hw->scratch[2] = CRASH_MAGIC | 0x00020000u;
    watchdog_reboot(0, 0, 0);
    for (;;) { }
}

// Canary for the lowest application priority; see TIER1_STARVE_TIMEOUT_MS.
// It does nothing but count. If it stops counting, no priority-1 task can be
// running either, and the probe reboots with a stamp saying so.
volatile uint32_t g_canary_beat = 0;

static void vTaskCanary(__unused void *pvParams) {
    for (;;) {
        g_canary_beat++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Stamp the reason into scratch[2] and reboot; see boot_checkpoints.h.
extern "C" void stamped_reboot(uint32_t kind) {
    watchdog_hw->scratch[2] = CRASH_MAGIC | ((kind & 0xFFu) << 16);
    watchdog_reboot(0, 0, 0);
    for (;;) { }
}

void vTaskWatchdog(__unused void *pvParams) {
    uint32_t last_beat = g_shell_heartbeat;
    uint32_t last_usb_beat = g_usb_heartbeat;
    uint32_t last_canary = g_canary_beat;
    uint32_t stalled_ms = 0;
    uint32_t usb_stalled_ms = 0;
    uint32_t canary_stalled_ms = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_POLL_MS));

        // Getting here proves the scheduler is alive, which is all the
        // hardware watchdog is guarding. Not before the failsafe hands over:
        // during boot the USB task feeds it, so that a boot which never
        // enumerates is still caught.
        if (g_hw_watchdog_live) {
            watchdog_update();
        }

        uint32_t beat = g_shell_heartbeat;
        uint32_t usb_beat = g_usb_heartbeat;
        uint32_t canary = g_canary_beat;

        if (beat != last_beat) {
            last_beat = beat;
            stalled_ms = 0;
        }
        else {
            stalled_ms += WATCHDOG_POLL_MS;
        }

        if (usb_beat != last_usb_beat) {
            last_usb_beat = usb_beat;
            usb_stalled_ms = 0;
        }
        else {
            usb_stalled_ms += WATCHDOG_POLL_MS;
        }

        if (canary != last_canary) {
            last_canary = canary;
            canary_stalled_ms = 0;
        }
        else {
            canary_stalled_ms += WATCHDOG_POLL_MS;
        }

        // Restarting re-runs Application::init(), which re-establishes the
        // target's debug interface -- the same recovery as replugging. The
        // stamp survives the reboot and the shell names the cause on the
        // next console connect.
        if (usb_stalled_ms >= USB_WEDGE_TIMEOUT_MS) {
            stamped_reboot(CRASH_KIND_USB_WEDGED);
        }
        if (stalled_ms >= SHELL_WEDGE_TIMEOUT_MS) {
            stamped_reboot(CRASH_KIND_SHELL_WEDGED);
        }
        if (canary_stalled_ms >= TIER1_STARVE_TIMEOUT_MS) {
            stamped_reboot(CRASH_KIND_TIER1_STARVED);
        }
    }
}

// Boot failsafe.
//
// Anything that hangs before USB enumerates takes the probe with it: no console,
// no 1200-baud BOOTSEL trigger, and the only way back is the physical button.
// So the hardware watchdog is armed for the whole of startup, with a magic value
// left in a scratch register. If we come back around having never reached USB,
// that magic is still there, and we drop straight into the bootloader instead of
// trying again -- the probe reflashes itself out of a bad image.
//
// Scratch register choice matters: the SDK owns [4..7] for its reboot vectors,
// and the bootrom takes [0] and [1] as the arguments to reset_usb_boot() -- so
// the act of escaping to the bootloader would destroy the very breadcrumb we
// went there to preserve. [2] and [3] are the only ones nobody else touches.
#define FAILSAFE_MAGIC   0xB007FA11u
#define FAILSAFE_TIMEOUT_MS 8000

// How far the *previous* boot got before it died, or 0 if it got all the way.
// Reported by the shell, since by definition the failed boot could not report it
// itself.
uint32_t g_prev_boot_checkpoint;

void boot_checkpoint(uint32_t n) {
    watchdog_hw->scratch[3] = n;
}

static void failsafe_arm() {
    if (watchdog_caused_reboot() && watchdog_hw->scratch[2] == FAILSAFE_MAGIC) {
        watchdog_hw->scratch[2] = 0;
        reset_usb_boot(0, 0);
    }

    g_prev_boot_checkpoint = watchdog_hw->scratch[3];
    g_prev_crash_stamp = watchdog_hw->scratch[2];
    // A timeout of the runtime hardware watchdog leaves no stamp -- nothing
    // was running to write one -- but the SDK marks an enable()-timeout
    // reboot distinctly from watchdog_reboot(), so it can be named anyway.
    // (picotool's reboot-to-application does not set that marker.)
    if (watchdog_enable_caused_reboot()
        && (g_prev_crash_stamp & 0xFF000000u) != CRASH_MAGIC) {
        g_prev_crash_stamp = CRASH_MAGIC | ((uint32_t)CRASH_KIND_HW_WATCHDOG << 16);
    }
    watchdog_hw->scratch[2] = 0;
    watchdog_hw->scratch[2] = FAILSAFE_MAGIC;
    boot_checkpoint(BOOT_CP_MAIN);
    watchdog_enable(FAILSAFE_TIMEOUT_MS, 1);
}

// Called once USB is enumerated: the escape hatch is live, so the failsafe has
// done its job and must stop resetting us.
void failsafe_disarm() {
    watchdog_hw->scratch[2] = 0;
    watchdog_hw->scratch[3] = 0;
    // Hand over rather than switch off: from here the hardware watchdog is
    // the backstop for a dead scheduler, fed by vTaskWatchdog. Re-arming
    // with the new timeout replaces the failsafe's 8s one in place.
    watchdog_enable(HW_WATCHDOG_TIMEOUT_MS, 1);
    g_hw_watchdog_live = true;
}

void board_init() {
    usbd_serial_init();
    boot_checkpoint(BOOT_CP_SERIAL_ID);
    set_sys_clock_khz(configCPU_CLOCK_HZ / 1000, false);
    boot_checkpoint(BOOT_CP_SYSCLK);
    stdio_init_all();
    boot_checkpoint(BOOT_CP_STDIO);
}

void application_init() {
    gApp = new Application();
    gApp->init();
}

int main() {
    failsafe_arm();

    bi_decl(bi_program_description("ch32v003 debugger binary."));

    board_init();
    boot_checkpoint(BOOT_CP_BOARD_INIT);
    application_init();
    boot_checkpoint(BOOT_CP_APP_INIT);
    factory_log_init();

    // 4x minimal: the task also builds the PICORVD drive at startup and
    // formats LOG.CSV rows inside mass-storage read callbacks.
    xTaskCreate(
            vTaskUsb,
            "usb",
            configMINIMAL_STACK_SIZE * 4,
            NULL,
            3,
            NULL
    );

    xTaskCreate(
            vTaskShell,
            "shell",
            configMINIMAL_STACK_SIZE * 8,
            NULL,
            2,
            NULL
    );

    xTaskCreate(
            vTaskGdb,
            "gdb",
            configMINIMAL_STACK_SIZE * 8,
            NULL,
            2,
            NULL
    );

    // Same priority as usb, so it still runs when the shell is spinning.
    xTaskCreate(
            vTaskWatchdog,
            "watchdog",
            configMINIMAL_STACK_SIZE,
            NULL,
            3,
            NULL
    );

    // Stores factory images uploaded through the PICORVD drive.
    xTaskCreate(
            vTaskMscDisk,
            "msc",
            configMINIMAL_STACK_SIZE * 4,
            NULL,
            1,
            NULL
    );

    // Priority-1 canary for the watchdog above; see vTaskCanary.
    xTaskCreate(
            vTaskCanary,
            "canary",
            configMINIMAL_STACK_SIZE,
            NULL,
            1,
            NULL
    );

#if defined(PICO_DEFAULT_LED_PIN) || defined(PICO_DEFAULT_WS2812_PIN)
    xTaskCreate(
            vTaskStatusLed,
            "status_led",
            configMINIMAL_STACK_SIZE,
            NULL,
            1,
            NULL
    );
#endif

#ifdef FACTORY_STANDALONE
    // The pass-around programmer: powered by the badge it is plugged into,
    // no console, no operator command. Flashing starts the moment we boot.
    factory_start();
#endif

    boot_checkpoint(BOOT_CP_TASKS);

    vTaskStartScheduler();

    return 0;
}

// Names for the markers above, so a failed boot reads as English rather than a
// number the reader has to go and look up.
const char *boot_checkpoint_name(uint32_t n) {
    switch (n) {
        case BOOT_CP_NONE:       return "clean";
        case BOOT_CP_MAIN:       return "entering main()";
        case BOOT_CP_SERIAL_ID:  return "reading flash unique id";
        case BOOT_CP_SYSCLK:     return "switching system clock";
        case BOOT_CP_STDIO:      return "stdio_init_all()";
        case BOOT_CP_BOARD_INIT: return "board_init() done";
        case BOOT_CP_SWIO:       return "PicoSWIO::init()";
        case BOOT_CP_SWIO_RESET: return "PicoSWIO::reset()";
        case BOOT_CP_RVDEBUG:    return "RVDebug::init()";
        case BOOT_CP_FLASH:      return "WCHFlash::reset()";
        case BOOT_CP_SOFTBREAK:  return "SoftBreak::init()";
        case BOOT_CP_GDBSERVER:  return "GDBServer::reset()";
        case BOOT_CP_APP_INIT:   return "application_init() done";
        case BOOT_CP_TASKS:      return "tasks created, starting scheduler";
        case BOOT_CP_USB_TASK:   return "usb task entered";
        case BOOT_CP_TUSB_INIT:  return "tusb_init() done";
        default:                 return "unknown";
    }
}
