/**
 * @file factory.cpp
 * @brief Production flashing loop for CH32V003-based boards.
 *
 * `factory` starts the loop in its own FreeRTOS task and returns; the shell
 * keeps its normal prompt, and `factory stop` ends the session. The task
 * polls for a target on SWIO, flashes the factory image stored in the
 * probe's flash (factory_image.cpp; uploaded over the PICORVD USB drive,
 * see usb/msc_disk.cpp), verifies it with automatic retry, and boots it.
 *
 * Two builds share this file. The tethered probe starts the loop with the
 * `factory` shell command and a host PC captures the CSV lines. After
 * booting the target it finds it with an SAOv3 bus scan (products choose
 * differing default addresses, so none is assumed), checks the identity and
 * sends the self-test command configured in CONFIG.TXT, and paints the LEDs
 * if the device exposes the standard SAOv3 LED class. The standalone build
 * (FACTORY_STANDALONE, target pico_rvd_factory) is for pass-around
 * programmer boards powered by the board under test, and assumes nothing
 * about the target beyond a CH32V003 to program. Success there is the
 * verified read-back of the flash, optionally followed by a UART self-test
 * byte over the SWIO pin; the outcome shows on the probe's LED (see
 * factory.h), a failing board is retried for as long as it stays plugged in,
 * and the flash log (factory_log.cpp) is the only record. Both builds append
 * every outcome to that log; `factory log` dumps it (so does LOG.CSV on the
 * drive) and `factory clear` erases it when the boards come back to a bench.
 *
 * The task deliberately touches no USB: SWIO, I2C and printf only. Earlier
 * revisions polled the console for a stop key from inside the (shell-task)
 * command handler, which put tinyusb's CDC data path under two-task load for
 * hours at a stretch; the field failure mode was the console going mute
 * mid-session. The shell's own loop is the only CDC reader again, exactly
 * the concurrency profile the rest of this firmware has always run.
 */

#include "pico.h"
#include "shell/Console.h"
#include "shell/console_colors.h"
#include "commands.h"
#include "factory.h"
#include "factory_image.h"
#include "factory_log.h"
#include "boot_checkpoints.h"

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#include "saoh/consts/sao.h"    // SAOv3 common interface commands + constants

#define FLASH_ATTEMPTS  3
#define BOOT_WAIT_POLLS 25      /* x200ms = 5s for the target to boot and answer I2C */
#define REMOVAL_POLLS   5       /* consecutive misses that count as unplugged */

// Post-flash UART self-test trigger (CONFIG.TXT uart_selftest): how long the
// freshly booted firmware gets to bring its UART up before the first byte,
// then how often and for how long the byte is repeated. A single byte turned
// out to be missed now and then by targets that otherwise booted fine; a
// self-test trigger should be idempotent, so a burst costs nothing.
#define UART_SELFTEST_DELAY_MS  100
#define UART_SELFTEST_REPEAT_MS 100
#define UART_SELFTEST_BURST_MS  1500

// Level for the post-flash LED check, per 8-bit channel. Bright enough to
// judge color, low enough that a string of SK6812s off the SAO 3V3 rail
// (which the probe supplies on a tethered bench) stays well under 100mA.
#define FACTORY_LED_TEST_LEVEL 64

// saoh_pico_hal.c: paint every LED via the SAOv3 LED class. 1 = painted,
// 0 = device has no LED class, <0 = bus error.
extern "C" int saoh_pico_leds_paint(uint8_t addr, uint8_t level);

static volatile bool factory_run = false;
static volatile bool factory_live = false;

// Bench instrumentation for the post-flash stall (2026-09-21): the loop
// goes quiet after a board until the host types a console command, and any
// printf we could add would be subject to the very stall we are chasing.
// So stamp the tick and the LED task's loop counter at each step and let
// `factory where` read them back afterwards -- the stamps written before
// the release tell us which step ate the time, and whether the LED task was
// alive while it did.
extern volatile uint32_t g_led_beat;
struct FactoryMark { uint32_t tick; uint32_t led; };
static volatile FactoryMark factory_marks[6];
static const char *const factory_mark_names[6] = {
    "verdict decided",
    "CSV printed",
    "log appended",
    "removal loop done",
    "removed printed",
    "top of loop",
};
static void factory_mark(int i) {
    factory_marks[i].tick = xTaskGetTickCount();
    factory_marks[i].led = g_led_beat;
}

// For the status LED task: the loop's state, since on a standalone programmer
// the LED is the only feedback an operator gets.
static volatile FactoryLed factory_led = FACTORY_LED_OFF;

FactoryLed factory_led_state(void) {
    return factory_led;
}

// Read the 96-bit factory UID (ESIG_UNIID1..3) from a halted target.
//
// The naive three reads proved unreliable in production: the fast debug read
// does not wait for the abstract command to finish, and on the signature
// area of a freshly halted (especially blank) chip DATA0 still holds the
// previous word, so the block came back shifted by one slot -- the log's
// [junk][id][id] rows. An earlier revision tried to filter that with a
// priming read plus a two-pass agreement check, which mostly just rejected
// the shifted data: 44 of 47 production records logged a zero UID. Now the
// reads are synchronous (RVDebug::get_mem_u32_sync waits on BUSY), and the
// agreement check stays as a guard rather than a workaround. The third word
// reads 0xFFFFFFFF on every CH32V003 seen so far; the id is effectively 64
// bits and is logged as read. Zeros on failure -- an honest "unknown" in the
// log beats confidently wrong.
static bool read_uid(uint32_t uid[3]) {
    for (int attempt = 0; attempt < 3; attempt++) {
        uint32_t a[3], b[3];
        for (int i = 0; i < 3; i++) a[i] = gApp->rvd->get_mem_u32_sync(0x1FFFF7E8 + 4u * i);
        for (int i = 0; i < 3; i++) b[i] = gApp->rvd->get_mem_u32_sync(0x1FFFF7E8 + 4u * i);

        bool stable = a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
        bool residue = a[0] == a[1] && a[1] == a[2];
        if (stable && !residue) {
            for (int i = 0; i < 3; i++) uid[i] = a[i];
            return true;
        }
    }
    uid[0] = uid[1] = uid[2] = 0;
    return false;
}

#ifndef FACTORY_STANDALONE
// Find an SAOv3 device on the bus: the first address answering the
// common-interface magic word. Products deliberately pick differing default
// addresses to dodge collisions on static-address badges, so the confirm
// can't assume one; a full 7-bit sweep costs a few tens of ms. Returns the
// address, or 0 for none found.
static uint8_t sao_scan(void) {
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint16_t magic;
        if (sao_read_word_quiet(addr, SAO_CMNITF_CMD_MAGIC, &magic)
            && magic == SAO_CMNITF_MAGIC_WORD) {
            return addr;
        }
    }
    return 0;
}
#endif

// A part id that reads as all-ones or all-zeroes is the SWIO line floating or
// stuck, i.e. no target.
static bool target_present(void) {
    gApp->swio->reset();
    uint32_t id = gApp->swio->get_partid();
    return id != 0 && id != 0xFFFFFFFFu;
}

static bool flash_once(const FactoryImage *img) {
    // Fresh attach and halt every attempt: a torn previous attempt may have
    // left the target in any state.
    gApp->swio->reset();
    if (!gApp->rvd->halt()) {
        printf(COLOR_RED("  halt failed") "\n");
        return false;
    }

    // Erase the image extent, but never the target's last flash page:
    // CH32V003 firmware commonly keeps its settings there, and re-flashing a
    // board must not wipe them. Whole sectors where the sector ends short of
    // that page; page by page in the last sector, only as far as the image
    // reaches. The image is written in whole pages: the stored copy is
    // followed by erased (0xFF) flash, so rounding up reads padding.
    uint32_t sector = (uint32_t)gApp->flash->get_sector_size();
    uint32_t page = (uint32_t)gApp->flash->get_page_size();
    uint32_t keep_from = (uint32_t)gApp->flash->get_flash_size() - page;
    uint32_t size = (img->size + page - 1) / page * page;
    if (size > keep_from) {
        printf(COLOR_RED("  image (%lu bytes) would overwrite the settings page at 0x%04lX") "\n",
               (unsigned long)img->size, (unsigned long)keep_from);
        return false;
    }
    for (uint32_t addr = 0; addr < size; ) {
        bool whole = (addr % sector) == 0 && addr + sector <= keep_from;
        if (!(whole ? gApp->flash->wipe_sector(addr) : gApp->flash->wipe_page(addr))) {
            printf(COLOR_RED("  erase failed at 0x%04lX") "\n", (unsigned long)addr);
            return false;
        }
        addr += whole ? sector : page;
    }

    if (!gApp->flash->write_flash(0, (void *)img->data, (int)size)) {
        printf(COLOR_RED("  write failed") "\n");
        return false;
    }

    return gApp->flash->verify_flash(0, (void *)img->data, (int)size);
}

static void factory_task(void *arg) {
    (void)arg;
    int okay = 0, failed = 0;

#ifdef FACTORY_STANDALONE
    // With no PC on the other end of the USB cable the failsafe's normal
    // disarm point -- enumeration -- never comes, and any watchdog reboot
    // during a session would bounce through failsafe_arm() into BOOTSEL: a
    // dark, dead programmer until someone replugs it. Reaching this task
    // proves the image boots, which is all the failsafe was guarding.
    failsafe_disarm();
#endif

    while (factory_run) {
        // Idle polling checks only the image header: CRCing the whole image
        // every pass kept the flash busy with uncached reads four times a
        // second, which is what glitched the WS2812 colors.
        if (!factory_image_stored()) {
            factory_led = FACTORY_LED_NO_IMAGE;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (factory_led == FACTORY_LED_NO_IMAGE) {
            factory_led = FACTORY_LED_WAITING;
        }

        if (!target_present()) {
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        // Debounce: a connector mid-insertion can answer one probe and drop
        // the next. Only flash once the target has been there twice in a row.
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!target_present()) {
            continue;
        }

        // The image can be replaced over USB between boards (msc_disk.cpp
        // waits for the loop to be idle), so pick it up fresh, fully
        // checked, for each board.
        FactoryImage img;
        if (!factory_image_get(&img)) {
            factory_led = FACTORY_LED_NO_IMAGE;
            continue;
        }

        factory_led = FACTORY_LED_BUSY;
        factory_mark(5);
        TickType_t t0 = xTaskGetTickCount();
        printf("[%d] target detected, flashing...\n", okay + failed + 1);

        // Birth registry: the chip's 96-bit factory UID (ESIG_UNIID1..3),
        // read while halted. Logged as a CSV line the host-side tail appends
        // to factory_log.csv, so every flashed unit is on record.
        uint32_t uid[3] = {0, 0, 0};
        gApp->swio->reset();
        if (gApp->rvd->halt()) {
            read_uid(uid);
        }

        bool flashed = false;
        int attempts = 0;
        while (!flashed && factory_run) {
            attempts++;
            if (attempts > 1) {
                printf("  verify failed, attempt %d\n", attempts);
            }
            flashed = flash_once(&img);
            if (!flashed) {
#ifdef FACTORY_STANDALONE
                // Nobody is watching a console: keep retrying for as long as
                // the badge stays plugged in. Record the struggle once at the
                // point the tethered build would have given up, so a badge
                // that gets unplugged mid-failure is still on the books; a
                // later success appends its own record over this one's story.
                if (attempts == FLASH_ATTEMPTS) {
                    uint32_t so_far = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
                    factory_log_append(uid, img.cfg.note, false, attempts, so_far);
                    factory_led = FACTORY_LED_FAIL;   // keeps retrying, but flag it
                }
                vTaskDelay(pdMS_TO_TICKS(250));
                if (!target_present()) {
                    break;
                }
#else
                if (attempts >= FLASH_ATTEMPTS) {
                    break;
                }
#endif
            }
        }

        // The UID read on first contact fails systematically on blank chips:
        // halted out of erased-flash execution, the ESIG reads come back as
        // all-FF residue, which read_uid rightly rejects. After a verified
        // flash the target sits halted in a known-good state, so give the
        // read a second chance before booting -- production badges are
        // always blank at first plug, so this path is the common one.
        if (flashed && uid[0] == 0 && uid[1] == 0 && uid[2] == 0) {
            if (gApp->rvd->halt()) {
                read_uid(uid);
            }
        }

#ifdef FACTORY_STANDALONE
        // No assumptions about the target board beyond the CH32V003 itself:
        // no speaker, no I2C, nothing to interrogate. The verified read-back
        // in flash_once() is the success criterion. The target is booted
        // further down, only after the log record is committed: a badge
        // whose new firmware lights everything up at boot can brown out a
        // marginal power path and reboot the probe, and if that happens the
        // record must already be on the books.
        bool confirmed = flashed;
        bool wrong_id = false;      // no identity to check without I2C
        bool led_fail = false;      // nor LEDs to paint
#else
        bool confirmed = false;
        bool wrong_id = false;
        bool led_fail = false;
        bool leds_painted = false;
        uint8_t sao_addr = 0;
        if (flashed) {
            // Boot the fresh firmware. The debug link is expected to drop
            // once it starts (it tristates its SWIO pin), so from here the
            // target is only reachable over I2C.
            factory_led = FACTORY_LED_BOOTING;
            gApp->rvd->reset();
            gApp->rvd->resume();

            for (int i = 0; i < BOOT_WAIT_POLLS && !confirmed && !wrong_id && factory_run; i++) {
                vTaskDelay(pdMS_TO_TICKS(200));

                // Standard SAOv3 liveness probe: scan for a device answering
                // the common-interface magic word, wherever it chose to sit.
                // Nothing vendor-specific is sent until the identity has
                // checked out.
                sao_addr = sao_scan();
                if (!sao_addr) {
                    if (i == BOOT_WAIT_POLLS / 2) {
                        // Bench-established quirk: a freshly flashed target
                        // is sometimes left halted despite the resume; one
                        // more resume has always recovered it.
                        gApp->rvd->resume();
                    }
                    continue;
                }

                // A failed word read after a good scan is treated as
                // still-booting and polled again; a clean read of the wrong
                // identity is decisive.
                uint16_t proto = 0, vid = 0, pid = 0;
                if (!sao_read_word_quiet(sao_addr, SAO_CMNITF_CMD_READ_PROTO_VERSION, &proto)
                    || !sao_read_word_quiet(sao_addr, SAO_CMNITF_CMD_READ_VID, &vid)
                    || !sao_read_word_quiet(sao_addr, SAO_CMNITF_CMD_READ_PID, &pid)) {
                    continue;
                }

                if ((proto & SAO_CMNITF_PROTO_VERSION_COMPAT_MASK)
                        != (SAO_CMNITF_PROTO_VERSION_CURRENT & SAO_CMNITF_PROTO_VERSION_COMPAT_MASK)) {
                    wrong_id = true;
                    printf(COLOR_RED("  0x%02X: incompatible SAOv3 proto %04X") "\n",
                           sao_addr, proto);
                }
                else if ((img.cfg.vid || img.cfg.pid)
                         && (vid != img.cfg.vid || pid != img.cfg.pid)) {
                    wrong_id = true;
                    printf(COLOR_RED("  0x%02X: answers as %04X:%04X, expected %04X:%04X") "\n",
                           sao_addr, vid, pid, img.cfg.vid, img.cfg.pid);
                }
                else {
                    // The product's configured self-test command, if any:
                    // a vendor command whose effect the operator can see or
                    // hear doubles as a quick production check.
                    confirmed = img.cfg.selftest_cmd < 0
                             || sao_send_byte_quiet(sao_addr, (uint8_t)img.cfg.selftest_cmd);
                    // Then the visual check: if the device exposes the
                    // standard SAOv3 LED class, take it over and paint
                    // every LED white, so the operator sees the whole chain
                    // lit -- a marginal joint or a dead color die stands
                    // out against its neighbours. A device without the
                    // class passes on the self-test alone; a device that
                    // has the class but will not take the write has failed.
                    if (confirmed) {
                        int rc = saoh_pico_leds_paint(sao_addr, FACTORY_LED_TEST_LEVEL);
                        if (rc < 0) {
                            confirmed = false;
                            led_fail = true;
                            printf(COLOR_RED("  0x%02X: LED class write failed (%d)") "\n",
                                   sao_addr, rc);
                        }
                        leds_painted = rc > 0;
                    }
                }
            }
        }
#endif

        uint32_t ms = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
#ifdef FACTORY_STANDALONE
        // A good flash still has the boot and self-test trigger ahead of it
        // (after the log record, below); OK only once those are done.
        factory_led = confirmed ? FACTORY_LED_BOOTING : FACTORY_LED_FAIL;
#else
        factory_led = confirmed ? FACTORY_LED_OK : FACTORY_LED_FAIL;
#endif
        factory_mark(0);
        if (confirmed) {
            okay++;
#ifdef FACTORY_STANDALONE
            printf(COLOR_GREEN("[%d] OK in %lums -- flashed and booted, unplug when done") "\n",
                   okay + failed, (unsigned long)ms);
#else
            printf(COLOR_GREEN("[%d] OK in %lums -- %s, unplug when done") "\n",
                   okay + failed, (unsigned long)ms,
                   leds_painted ? "LEDs white" : "self-test triggered");
#endif
        }
        else {
            failed++;
            printf(COLOR_RED("[%d] FAILED (%s) -- unplug this board and set it aside") "\n",
                   okay + failed,
                   !flashed ? "flash/verify failed"
                            : wrong_id ? "wrong device identity"
                            : led_fail ? "LED class write failed"
                                       : "flashed but no SAO answered");
        }

        printf("CSV,%08lX%08lX%08lX,%s,%s,%lu\n",
               (unsigned long)uid[0], (unsigned long)uid[1], (unsigned long)uid[2],
               img.cfg.note, confirmed ? "ok" : "fail", (unsigned long)ms);
        factory_mark(1);

        // The same line, into the probe's own flash: a standalone programmer
        // has no host tailing the console, so this is its only record.
        factory_log_append(uid, img.cfg.note, confirmed,
                           (uint8_t)(attempts > 255 ? 255 : attempts), ms);
        factory_mark(2);

#ifdef FACTORY_STANDALONE
        // Record safely down; now boot the fresh firmware.
        if (flashed) {
            gApp->rvd->reset();
            gApp->rvd->resume();
            // Bench-established quirk: a freshly flashed target is
            // sometimes left halted despite the resume;
            // one more resume has always recovered it. Harmless when the
            // first one took.
            vTaskDelay(pdMS_TO_TICKS(100));
            gApp->rvd->resume();

            // Some targets listen for a self-test trigger on their SWIO pin,
            // which can double as a UART RX (CONFIG.TXT uart_selftest). Give
            // the firmware time to boot, then send the byte every
            // UART_SELFTEST_REPEAT_MS for UART_SELFTEST_BURST_MS, handing the
            // pin back to SWIO after each one. No halt and no reflash follow:
            // the removal watch below only reads the part ID, and a target
            // that keeps its debug link alive keeps reading as present -- so
            // as long as the board stays plugged in it is neither re-flashed
            // nor stopped.
            if (img.cfg.uart_selftest >= 0) {
                vTaskDelay(pdMS_TO_TICKS(UART_SELFTEST_DELAY_MS));
            }
            for (int t = 0; img.cfg.uart_selftest >= 0 && t < UART_SELFTEST_BURST_MS && factory_run;
                 t += UART_SELFTEST_REPEAT_MS) {
                if (!gApp->swio->send_uart_byte((uint8_t)img.cfg.uart_selftest, img.cfg.uart_baud)) {
                    printf(COLOR_RED("  SWIO pin has no UART TX; self-test byte not sent") "\n");
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(UART_SELFTEST_REPEAT_MS));
            }
            factory_led = FACTORY_LED_OK;
        }
#endif

#ifdef FACTORY_STANDALONE
        // Wait for removal over SWIO alone -- there is nothing else to ask.
        // A target whose new firmware repurposes the SWIO pin reads as absent
        // immediately; that is fine, because a target that no longer answers
        // can't be re-detected and re-flashed either, so falling through to
        // "waiting" just arms the probe for the next board. The LED only
        // goes back to "waiting" if the board was seen and then went away
        // (below); otherwise the absence may just be the firmware taking the
        // pin, and the verdict holds until a new board is detected.
        int gone = 0;
        bool seen = false;
        while (gone < REMOVAL_POLLS && factory_run) {
            vTaskDelay(pdMS_TO_TICKS(200));
            if (target_present()) {
                gone = 0;
                seen = true;
            }
            else {
                gone++;
            }
        }
#else
        // Wait for removal over SWIO alone, as the standalone build does. A
        // booted board tristates its SWIO pin and so reads as absent almost
        // at once; that is fine, because a target that no longer answers
        // cannot be re-detected or re-flashed anyway, so falling through
        // just arms the probe for the next board. A board that failed is
        // watched the same way, so reseating a bad SWIO contact retries it.
        // The LED keeps the last verdict unless the board was seen and then
        // went away (below), so the operator keeps the "done, unplug it"
        // signal while the board may still be plugged in.
        //
        // (An earlier revision watched a passed board over I2C. That was
        // replaced while chasing the post-flash stall; the stall turned out
        // not to be I2C-related -- see factory_marks -- but SWIO is the
        // simpler watch and there is no reason to go back.)
        int gone = 0;
        bool seen = false;
        while (gone < REMOVAL_POLLS && factory_run) {
            vTaskDelay(pdMS_TO_TICKS(200));
            if (target_present()) {
                gone = 0;
                seen = true;
            }
            else {
                gone++;
            }
        }
#endif
        factory_mark(3);
        // A board that kept answering and then stopped was really unplugged:
        // its verdict has been seen, so show "waiting" again. (A probe powered
        // by the board goes dark on unplug anyway; this is for one on USB.)
        if (factory_run && seen) {
            factory_led = FACTORY_LED_WAITING;
        }
        if (factory_run) {
            printf("    removed; waiting for next board\n");
        }
        factory_mark(4);
    }

    printf("factory done: %d ok, %d failed\n", okay, failed);
    factory_led = FACTORY_LED_OFF;
    factory_live = false;
    vTaskDelete(NULL);
}

// Start the flashing loop. The tethered build calls this from the shell
// command below; the standalone build calls it from main() at boot.
bool factory_start(void) {
    if (factory_live) {
        return false;
    }
    factory_run = true;
    factory_live = true;
    factory_led = FACTORY_LED_WAITING;
    xTaskCreate(factory_task, "factory", configMINIMAL_STACK_SIZE * 8, NULL, 1, NULL);

    FactoryImage img;
    if (factory_image_get(&img)) {
        printf("factory mode: %s (%lu bytes), log %d/%d used\n",
               img.cfg.note, (unsigned long)img.size,
               factory_log_count(), factory_log_capacity());
    }
    else {
        printf("factory mode: no image yet -- copy a .bin onto the PICORVD drive\n");
    }
#ifdef FACTORY_STANDALONE
#ifdef PICO_DEFAULT_LED_PIN
    printf("plug a board to flash it; LED mostly on = passed, double blip = failed.\n"
           "'factory stop' to end.\n");
#else
    printf("plug a board to flash it; green = passed, red = failed. 'factory stop' to end.\n");
#endif
#else
    printf("plug a board to flash it; LEDs white = done. 'factory stop' to end.\n");
#endif
    return true;
}

void command_factory(Console &c) {
    while (c.packet.match(' ')) {
        // match_word starts at the cursor, which dispatch left on the space
        // after "factory"; eat it (and any extras) before looking for "stop".
    }
    if (c.packet.match_word("stop")) {
        if (factory_live) {
            factory_run = false;
            printf("stopping after the current board...\n");
        }
        else {
            printf("factory is not running\n");
        }
        return;
    }

    if (c.packet.match_word("image")) {
        factory_image_print();
        return;
    }

    if (c.packet.match_word("log")) {
        factory_log_dump();
        return;
    }

    if (c.packet.match_word("where")) {
        uint32_t now = xTaskGetTickCount();
        printf("now: tick %lu, led beat %lu, factory %s\n",
               (unsigned long)now, (unsigned long)g_led_beat,
               factory_live ? "running" : "stopped");
        for (int i = 0; i < 6; i++) {
            uint32_t t = factory_marks[i].tick;
            printf("  %-18s tick %10lu (%+8ld ms ago)  led beat %lu\n",
                   factory_mark_names[i], (unsigned long)t,
                   (long)(now - t) * (long)portTICK_PERIOD_MS,
                   (unsigned long)factory_marks[i].led);
        }
        return;
    }

    if (c.packet.match_word("clear")) {
        if (factory_live) {
            // The loop could append mid-erase; make clearing a bench-only,
            // loop-stopped operation. (The standalone build starts the loop
            // at boot, so collecting its log ends with a stop/clear/replug.)
            printf("factory is running; 'factory stop' first\n");
            return;
        }
        printf("erasing %d-record log...\n", factory_log_count());
        printf(factory_log_clear() ? "factory log cleared\n"
                                   : COLOR_RED("factory log erase FAILED") "\n");
        return;
    }

    if (!factory_start()) {
        printf("factory already running; 'factory stop' to end it\n");
    }
}
