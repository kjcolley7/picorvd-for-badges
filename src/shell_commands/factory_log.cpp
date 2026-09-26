/**
 * @file factory_log.cpp
 * @brief Append-only production log in the probe's flash (2048 records).
 *
 * The standalone factory build runs with no PC attached, so the CSV lines the
 * tethered station relies on a host to capture have to live on the probe
 * itself. Layout: one record per 256-byte flash page, appended in order
 * through the log's two segments (flash_layout.h), CRC
 * over each record. The append point is simply the first still-erased page,
 * so there is no header or index to corrupt: a record is either fully written
 * (CRC checks out), torn by a power loss (CRC fails, reported and skipped at
 * dump time), or the page is erased. Flash order is chronological order --
 * the probe has no RTC, so sequence plus per-boot uptime is all the
 * timekeeping there is; wall-clock time gets attached when the logs are
 * collected and merged.
 *
 * Writes go through flash_safe_execute(), which on this FreeRTOS SMP build
 * parks the other core in an interrupts-off task for the duration -- the
 * RP2040 executes from the same flash it is programming, so both cores must
 * be out of XIP. Page programs are sub-millisecond; the clear erases one
 * 4KB sector per call so USB gets serviced between the ~45ms erases.
 */

#include "factory_log.h"
#include "flash_layout.h"

#include <hardware/flash.h>
#include <hardware/regs/addressmap.h>
#include <pico/flash.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <stdio.h>
#include <string.h>

#include "usb/tusb_config.h"
#include "shell/console_colors.h"

#define FLOG_MAGIC          0x474F4C46u                             // "FLOG"

struct flog_record {
    uint32_t magic;
    uint32_t seq;           // 1-based append index, for cross-checking merges
    uint32_t uid[3];        // target's 96-bit factory UID (zeros = read failed)
    uint32_t elapsed_ms;    // detection to outcome, this board
    uint32_t uptime_s;      // probe uptime at the time of the record
    uint8_t  ok;
    uint8_t  attempts;
    uint8_t  reserved[2];
    char     fw_note[64];   // the factory image's note (CONFIG.TXT)
    uint32_t crc;           // over everything above
};
static_assert(sizeof(flog_record) <= FLASH_PAGE_SIZE, "record must fit a page");

// Read the log through the uncached XIP alias: the cache may still hold the
// erased 0xFF contents of a page we have since programmed.
static uint32_t flog_offset(int i) {
    return i < FLOG_SEG1_RECORDS
         ? FLOG_SEG1_OFFSET + (uint32_t)i * FLASH_PAGE_SIZE
         : FLOG_SEG2_OFFSET + (uint32_t)(i - FLOG_SEG1_RECORDS) * FLASH_PAGE_SIZE;
}

static const uint8_t *flog_page(int i) {
    return (const uint8_t *)(XIP_NOCACHE_NOALLOC_BASE + flog_offset(i));
}

static bool sector_erased(uint32_t offset) {
    const uint32_t *p = (const uint32_t *)(XIP_NOCACHE_NOALLOC_BASE + offset);
    for (uint32_t i = 0; i < FLASH_SECTOR_SIZE / 4; i++) {
        if (p[i] != 0xFFFFFFFFu) {
            return false;
        }
    }
    return true;
}

static SemaphoreHandle_t flog_lock;
static int flog_next = -1;      // first erased page, or FLOG_MAX_RECORDS; -1 = unscanned

static uint32_t crc32_of(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

static bool record_valid(const flog_record *r) {
    return r->magic == FLOG_MAGIC
        && r->crc == crc32_of(r, offsetof(flog_record, crc));
}

// A page is free iff its first word is still erased. A torn write may have
// programmed only part of the page, but the magic goes down with everything
// else in the one page-program, so a page that died mid-write reads as used
// (and its CRC won't check out).
static int scan_append_point(void) {
    for (int i = 0; i < FLOG_MAX_RECORDS; i++) {
        if (*(const uint32_t *)flog_page(i) == 0xFFFFFFFFu) {
            return i;
        }
    }
    return FLOG_MAX_RECORDS;
}

void factory_log_init(void) {
    flog_lock = xSemaphoreCreateMutex();
}

int factory_log_capacity(void) {
    return FLOG_MAX_RECORDS;
}

int factory_log_count(void) {
    xSemaphoreTake(flog_lock, portMAX_DELAY);
    if (flog_next < 0) {
        flog_next = scan_append_point();
    }
    int n = flog_next;
    xSemaphoreGive(flog_lock);
    return n;
}

struct prog_args {
    uint32_t offset;
    const uint8_t *data;
};

static void do_program(void *arg) {
    const prog_args *a = (const prog_args *)arg;
    flash_range_program(a->offset, a->data, FLASH_PAGE_SIZE);
}

static void do_erase(void *arg) {
    flash_range_erase((uint32_t)(uintptr_t)arg, FLASH_SECTOR_SIZE);
}

bool factory_log_append(const uint32_t uid[3], const char *fw_note,
                        bool ok, uint8_t attempts, uint32_t elapsed_ms) {
    xSemaphoreTake(flog_lock, portMAX_DELAY);
    if (flog_next < 0) {
        flog_next = scan_append_point();
    }

    // Dedupe repeated failures only: a badge sitting in the standalone
    // retry loop would otherwise write a fail record per cycle. Successes
    // always append -- every completed flash, including a deliberate
    // reflash of the same badge, is a production event worth its own row.
    if (!ok && flog_next > 0) {
        const flog_record *last = (const flog_record *)flog_page(flog_next - 1);
        if (record_valid(last) && last->ok == 0
            && memcmp(last->uid, uid, sizeof(last->uid)) == 0) {
            xSemaphoreGive(flog_lock);
            return true;
        }
    }

    if (flog_next >= FLOG_MAX_RECORDS) {
        xSemaphoreGive(flog_lock);
        printf(COLOR_RED("factory log FULL (%d records) -- dump and clear it") "\n",
               FLOG_MAX_RECORDS);
        return false;
    }

    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));

    flog_record *r = (flog_record *)page;
    memset(r, 0, sizeof(*r));
    r->magic = FLOG_MAGIC;
    r->seq = (uint32_t)flog_next + 1;
    memcpy(r->uid, uid, sizeof(r->uid));
    r->elapsed_ms = elapsed_ms;
    r->uptime_s = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ);
    r->ok = ok ? 1 : 0;
    r->attempts = attempts;
    strncpy(r->fw_note, fw_note, sizeof(r->fw_note) - 1);
    r->crc = crc32_of(r, offsetof(flog_record, crc));

    prog_args args = {
        .offset = flog_offset(flog_next),
        .data = page,
    };
    // Starting a fresh sector: make sure it really is blank. Segment 2 was
    // never part of the log on older firmware, so don't trust it blindly.
    int rc = PICO_OK;
    if (args.offset % FLASH_SECTOR_SIZE == 0 && !sector_erased(args.offset)) {
        rc = flash_safe_execute(do_erase, (void *)(uintptr_t)args.offset, 500);
        vTaskDelay(1);
    }
    if (rc == PICO_OK) {
        rc = flash_safe_execute(do_program, &args, 500);
    }
    bool written = rc == PICO_OK
                && record_valid((const flog_record *)flog_page(flog_next));
    if (written) {
        flog_next++;
    }

    int remaining = FLOG_MAX_RECORDS - flog_next;
    xSemaphoreGive(flog_lock);

    if (!written) {
        printf(COLOR_RED("factory log write FAILED (rc %d)") "\n", rc);
    }
    else if (remaining <= 32) {
        printf(COLOR_RED("factory log nearly full: %d slots left") "\n", remaining);
    }
    return written;
}

int factory_log_csv_row(int i, char *buf, size_t len) {
    if (i < 0 || i >= FLOG_MAX_RECORDS) {
        return 0;
    }
    const flog_record *r = (const flog_record *)flog_page(i);
    if (!record_valid(r)) {
        return 0;
    }
    int n = snprintf(buf, len, "%s,%lu,%08lX%08lX%08lX,%.*s,%s,%u,%lu,%lu\n",
                     usbd_serial_str(),
                     (unsigned long)r->seq,
                     (unsigned long)r->uid[0], (unsigned long)r->uid[1],
                     (unsigned long)r->uid[2],
                     (int)strnlen(r->fw_note, sizeof(r->fw_note)), r->fw_note,
                     r->ok ? "ok" : "fail",
                     (unsigned)r->attempts,
                     (unsigned long)r->elapsed_ms,
                     (unsigned long)r->uptime_s);
    return n < 0 ? 0 : (n < (int)len ? n : (int)len - 1);
}

void factory_log_dump(void) {
    xSemaphoreTake(flog_lock, portMAX_DELAY);
    if (flog_next < 0) {
        flog_next = scan_append_point();
    }
    int n = flog_next;

    // The probe's flash unique id, same string as the USB serial number, so
    // merged logs say which programmer flashed which badge. Uptime makes
    // unexpected probe reboots (e.g. insertion power transients) visible on
    // every collection.
    printf("# factory log: probe %s, %d/%d records, uptime %lus\n",
           usbd_serial_str(), n, FLOG_MAX_RECORDS,
           (unsigned long)(xTaskGetTickCount() / configTICK_RATE_HZ));
    printf("# CSV,probe,seq,uid,firmware,status,attempts,ms,uptime_s\n");

    for (int i = 0; i < n; i++) {
        char row[160];
        if (!factory_log_csv_row(i, row, sizeof(row))) {
            printf("# record %d torn/corrupt, skipped\n", i + 1);
            continue;
        }
        printf("CSV,%s", row);
    }
    xSemaphoreGive(flog_lock);
}

bool factory_log_clear(void) {
    xSemaphoreTake(flog_lock, portMAX_DELAY);
    bool okay = true;
    for (int i = 0; i < FLOG_MAX_RECORDS && okay; i += FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE) {
        uint32_t off = flog_offset(i);
        if (sector_erased(off)) {
            continue;       // most of a partly used log: skip the ~45ms erase
        }
        okay = flash_safe_execute(do_erase, (void *)(uintptr_t)off, 500) == PICO_OK;
        // Let USB and everything else breathe between the erases.
        vTaskDelay(1);
    }
    flog_next = okay ? 0 : -1;
    xSemaphoreGive(flog_lock);
    return okay;
}
