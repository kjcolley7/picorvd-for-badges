/**
 * @file factory_image.cpp
 * @brief The factory loop's target image and settings, in the probe's flash.
 *
 * Region layout, just below the factory log (factory_log.cpp): one sector
 * holding the header, then the image. The image is programmed first and the
 * header last, so a power cut mid-update leaves either no valid header or a
 * header whose CRCs match what it describes.
 */

#include "factory_image.h"
#include "shell/console_colors.h"

#include <FreeRTOS.h>
#include <task.h>
#include "pico/flash.h"
#include "hardware/flash.h"
#include "pico/stdlib.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FIMG_MAGIC          0x474D4946u   /* "FIMG" */
#define FIMG_DATA_SECTORS   ((FACTORY_IMAGE_MAX + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE)
#define FIMG_REGION_SIZE    ((1 + FIMG_DATA_SECTORS) * FLASH_SECTOR_SIZE)
#define FIMG_LOG_SIZE       (128 * 1024)   /* factory_log.cpp's region, just above */
#define FIMG_REGION_OFFSET  (PICO_FLASH_SIZE_BYTES - FIMG_LOG_SIZE - FIMG_REGION_SIZE)
#define FIMG_DATA_OFFSET    (FIMG_REGION_OFFSET + FLASH_SECTOR_SIZE)

struct fimg_header {
    uint32_t magic;
    uint32_t size;
    uint32_t data_crc;
    FactoryConfig cfg;
    uint32_t hdr_crc;        // over everything above
};

static_assert(sizeof(fimg_header) <= FLASH_PAGE_SIZE, "header must fit one page");

// Read through the uncached XIP alias, like the factory log: the cache may
// still hold the erased contents of pages we have since programmed.
#define FIMG_READ_BASE XIP_NOCACHE_NOALLOC_BASE

static const fimg_header *stored_header(void) {
    return (const fimg_header *)(FIMG_READ_BASE + FIMG_REGION_OFFSET);
}

uint32_t factory_crc32(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
        }
    }
    return ~crc;
}

void factory_config_defaults(FactoryConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->uart_selftest = -1;
    cfg->uart_baud = 115200;
    cfg->selftest_cmd = -1;
}

bool factory_image_stored(void) {
    const fimg_header *h = stored_header();
    return h->magic == FIMG_MAGIC && h->size != 0 && h->size <= FACTORY_IMAGE_MAX
        && h->hdr_crc == factory_crc32(h, offsetof(fimg_header, hdr_crc));
}

bool factory_image_get(FactoryImage *out) {
    if (!factory_image_stored()) {
        return false;
    }
    const fimg_header *h = stored_header();
    const uint8_t *data = (const uint8_t *)(FIMG_READ_BASE + FIMG_DATA_OFFSET);
    if (factory_crc32(data, h->size) != h->data_crc) {
        return false;
    }
    out->data = data;
    out->size = h->size;
    out->crc = h->data_crc;
    out->cfg = h->cfg;
    out->cfg.note[sizeof(out->cfg.note) - 1] = 0;
    return true;
}

// ---- writing ----
// One flash_safe_execute() per sector erase or page program, like the
// factory log: each call parks the other core with interrupts off, so keep
// each one short.

struct fimg_op {
    uint32_t offset;
    const uint8_t *page;     // NULL = erase the sector at offset
};

static void do_op(void *arg) {
    const fimg_op *op = (const fimg_op *)arg;
    if (op->page) {
        flash_range_program(op->offset, op->page, FLASH_PAGE_SIZE);
    }
    else {
        flash_range_erase(op->offset, FLASH_SECTOR_SIZE);
    }
}

// Every flash_safe_execute() creates a short-lived "flash lockout" task on
// the other core, whose memory only returns to the FreeRTOS heap once the
// idle task reaps it. Dozens of back-to-back calls would pile those up and
// run the heap dry (PICO_ERROR_INSUFFICIENT_RESOURCES), so yield after each
// one, and retry a couple of times if the heap was momentarily short anyway.
static bool run_op(uint32_t offset, const uint8_t *page) {
    fimg_op op = { offset, page };
    int rc = PICO_ERROR_INSUFFICIENT_RESOURCES;
    for (int attempt = 0; attempt < 3 && rc == PICO_ERROR_INSUFFICIENT_RESOURCES; attempt++) {
        rc = flash_safe_execute(do_op, &op, 1000);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (rc != PICO_OK) {
        printf(COLOR_RED("factory image: flash %s at 0x%06lX failed (rc %d)") "\n",
               page ? "program" : "erase", (unsigned long)offset, rc);
    }
    return rc == PICO_OK;
}

static bool write_header(uint32_t size, uint32_t data_crc, const FactoryConfig *cfg) {
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    fimg_header *h = (fimg_header *)page;
    h->magic = FIMG_MAGIC;
    h->size = size;
    h->data_crc = data_crc;
    h->cfg = *cfg;
    h->cfg.note[sizeof(h->cfg.note) - 1] = 0;
    h->hdr_crc = factory_crc32(h, offsetof(fimg_header, hdr_crc));
    return run_op(FIMG_REGION_OFFSET, page);
}

bool factory_config_store(const FactoryConfig *cfg) {
    FactoryImage cur;
    if (!factory_image_get(&cur)) {
        return false;
    }
    if (!run_op(FIMG_REGION_OFFSET, NULL) || !write_header(cur.size, cur.crc, cfg)) {
        return false;
    }
    FactoryImage check;
    return factory_image_get(&check) && !memcmp(&check.cfg, cfg, sizeof(*cfg));
}

bool factory_image_store(const uint8_t *data, uint32_t size, const FactoryConfig *cfg) {
    if (size == 0 || size > FACTORY_IMAGE_MAX) {
        return false;
    }

    // Header sector first: from here until the header is rewritten there is
    // simply no image, never a stale header over new data.
    for (uint32_t s = 0; s < 1 + FIMG_DATA_SECTORS; s++) {
        if (!run_op(FIMG_REGION_OFFSET + s * FLASH_SECTOR_SIZE, NULL)) {
            return false;
        }
    }

    uint8_t page[FLASH_PAGE_SIZE];
    for (uint32_t off = 0; off < size; off += FLASH_PAGE_SIZE) {
        uint32_t n = size - off < FLASH_PAGE_SIZE ? size - off : FLASH_PAGE_SIZE;
        memset(page, 0xFF, sizeof(page));
        memcpy(page, data + off, n);
        if (!run_op(FIMG_DATA_OFFSET + off, page)) {
            return false;
        }
    }

    uint32_t crc = factory_crc32(data, size);
    if (!write_header(size, crc, cfg)) {
        return false;
    }

    FactoryImage check;
    return factory_image_get(&check) && check.size == size && check.crc == crc;
}

// ---- CONFIG.TXT ----

static int fmt_opt(char *buf, size_t len, const char *key, long value, bool hex, bool none) {
    if (none) {
        return snprintf(buf, len, "%s = none\n", key);
    }
    return snprintf(buf, len, hex ? "%s = 0x%02lX\n" : "%s = %ld\n", key, value);
}

int factory_config_format(const FactoryConfig *cfg, char *buf, size_t len) {
    size_t n = 0;
#define PUT(...) do { int r = snprintf(buf + n, len - n, __VA_ARGS__); \
                      if (r > 0) n += (size_t)r < len - n ? (size_t)r : len - n - 1; } while (0)
#define OPT(...) do { int r = fmt_opt(buf + n, len - n, __VA_ARGS__); \
                      if (r > 0) n += (size_t)r < len - n ? (size_t)r : len - n - 1; } while (0)
    PUT("# Factory image settings. Edit and save this file to apply them.\n"
        "# Numbers may be decimal or 0x hex; \"none\" turns a setting off.\n"
        "\n"
        "# Label recorded in the factory log for every board flashed. Left\n"
        "# unchanged, uploading a new .BIN sets it to the file name and CRC.\n"
        "note = %s\n"
        "\n"
        "# Standalone programmer: a byte sent to the target over the SWIO pin as\n"
        "# 8N1 UART once the new firmware has booted (e.g. to start a self-test).\n", cfg->note);
    OPT("uart_selftest", cfg->uart_selftest, true, cfg->uart_selftest < 0);
    OPT("uart_baud", (long)cfg->uart_baud, false, false);
    PUT("\n"
        "# Tethered probe: the SAOv3 VID/PID the booted target must report over\n"
        "# I2C, and a vendor command to send it as a self-test.\n");
    OPT("vid", cfg->vid, true, cfg->vid == 0);
    OPT("pid", cfg->pid, true, cfg->pid == 0);
    OPT("selftest", cfg->selftest_cmd, true, cfg->selftest_cmd < 0);
#undef PUT
#undef OPT
    return (int)n;
}

static bool is_space(char c) {
    return c == ' ' || c == '\t' || c == '\r';
}

void factory_config_parse(const char *text, size_t len, FactoryConfig *cfg) {
    const char *end = text + len;
    while (text < end) {
        const char *eol = (const char *)memchr(text, '\n', (size_t)(end - text));
        if (!eol) {
            eol = end;
        }
        const char *k = text, *ke, *v, *ve = eol;
        text = eol + 1;

        while (k < ve && is_space(*k)) k++;
        if (k == ve || *k == '#') {
            continue;
        }
        const char *eq = (const char *)memchr(k, '=', (size_t)(ve - k));
        if (!eq) {
            continue;
        }
        for (ke = eq; ke > k && is_space(ke[-1]); ke--) { }
        for (v = eq + 1; v < ve && is_space(*v); v++) { }
        while (ve > v && is_space(ve[-1])) ve--;

        char key[16], val[64];
        size_t kl = (size_t)(ke - k), vl = (size_t)(ve - v);
        if (kl == 0 || kl >= sizeof(key)) {
            continue;
        }
        if (vl >= sizeof(val)) {
            vl = sizeof(val) - 1;
        }
        memcpy(key, k, kl); key[kl] = 0;
        memcpy(val, v, vl); val[vl] = 0;

        if (!strcmp(key, "note")) {
            strncpy(cfg->note, val, sizeof(cfg->note) - 1);
            cfg->note[sizeof(cfg->note) - 1] = 0;
            continue;
        }
        bool none = !strcmp(val, "none") || !val[0];
        long num = none ? 0 : strtol(val, NULL, 0);
        if (!strcmp(key, "uart_selftest")) {
            cfg->uart_selftest = none ? -1 : (int16_t)(num & 0xFF);
        }
        else if (!strcmp(key, "uart_baud") && !none && num > 0) {
            cfg->uart_baud = (uint32_t)num;
        }
        else if (!strcmp(key, "vid")) {
            cfg->vid = (uint16_t)num;
        }
        else if (!strcmp(key, "pid")) {
            cfg->pid = (uint16_t)num;
        }
        else if (!strcmp(key, "selftest")) {
            cfg->selftest_cmd = none ? -1 : (int16_t)(num & 0xFF);
        }
    }
}

void factory_image_print(void) {
    FactoryImage img;
    if (!factory_image_get(&img)) {
        printf(COLOR_RED("no factory image stored") " -- copy a .bin onto the PICORVD drive\n");
        return;
    }
    static char text[768];
    factory_config_format(&img.cfg, text, sizeof(text));
    printf("factory image: %lu bytes, crc32 %08lX\n", (unsigned long)img.size, (unsigned long)img.crc);
    for (char *line = strtok(text, "\n"); line; line = strtok(NULL, "\n")) {
        if (line[0] != '#') {
            printf("  %s\n", line);
        }
    }
}
