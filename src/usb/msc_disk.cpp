/**
 * @file msc_disk.cpp
 * @brief The PICORVD USB drive: a small FAT12 volume for the factory image.
 *
 * The first RAM_BLOCKS of the volume live in RAM and are built at boot:
 * boot sector, two FATs, root directory, README.TXT and CONFIG.TXT, plus free
 * space for uploads. The rest is read-only and generated on every read, so it
 * costs no RAM (the probe has little to spare): CURRENT.BIN comes straight
 * from the stored image, and LOG.CSV from the factory log in flash. LOG.CSV
 * is a snapshot: rows appended after boot show up on the next boot. Writes
 * to the read-only blocks are dropped.
 *
 * Uploads: once host writes have been quiet for a moment (or the drive is
 * ejected), msc_disk_task() reads the volume back, stores any new .BIN and/or
 * edited CONFIG.TXT as the factory image, and reboots so the drive reappears
 * showing the new state.
 */

#include "msc_disk.h"
#include "tusb.h"
#include "pico/stdlib.h"
#include "factory_image.h"
#include "factory_log.h"
#include "factory.h"
#include "boot_checkpoints.h"

#include <FreeRTOS.h>
#include <task.h>

#include <string.h>
#include <stdio.h>

#define BLOCK_SIZE      512
#define RAM_BLOCKS      64                      // 32 KB in RAM
#define CUR_BLOCKS      ((FACTORY_IMAGE_MAX + BLOCK_SIZE - 1) / BLOCK_SIZE)
#define LOG_BLOCKS      128                     // 64 KB
#define TOTAL_BLOCKS    (RAM_BLOCKS + CUR_BLOCKS + LOG_BLOCKS)

#define FAT_BLOCKS      1                       // per FAT; 2 FATs
#define ROOT_ENTRIES    64
#define ROOT_BLOCKS     (ROOT_ENTRIES * 32 / BLOCK_SIZE)
#define LBA_FAT1        1
#define LBA_FAT2        (LBA_FAT1 + FAT_BLOCKS)
#define LBA_ROOT        (LBA_FAT2 + FAT_BLOCKS)
#define LBA_DATA        (LBA_ROOT + ROOT_BLOCKS)
#define CLUSTERS        (TOTAL_BLOCKS - LBA_DATA)   // 1 block per cluster
#define CUR_FIRST_LBA   RAM_BLOCKS
#define LOG_FIRST_LBA   (CUR_FIRST_LBA + CUR_BLOCKS)

static_assert(CLUSTERS * 3 / 2 + 3 <= FAT_BLOCKS * BLOCK_SIZE, "FAT12 table too small");

static uint8_t disk[RAM_BLOCKS][BLOCK_SIZE];

struct __attribute__((packed)) dir_entry {
    char     name[11];
    uint8_t  attr;
    uint8_t  ntres;
    uint8_t  crt_tenth;
    uint16_t crt_time;
    uint16_t crt_date;
    uint16_t acc_date;
    uint16_t clus_hi;
    uint16_t wrt_time;
    uint16_t wrt_date;
    uint16_t clus_lo;
    uint32_t size;
};
static_assert(sizeof(dir_entry) == 32, "FAT directory entry is 32 bytes");

#define ATTR_READONLY   0x01
#define ATTR_VOLUME     0x08
#define ATTR_DIR        0x10
#define ATTR_LFN        0x0F

// 2026-01-01 00:00, FAT-encoded.
#define FAT_DATE        (((2026 - 1980) << 9) | (1 << 5) | 1)

static uint32_t log_size;                       // LOG.CSV length, fixed at boot
static int      log_rows;                       // records it covers
static FactoryImage cur_img;                    // CURRENT.BIN, if cur_have
static bool     cur_have;

// One shared text buffer: README.TXT/CONFIG.TXT at boot, CONFIG.TXT in scans.
static char text[1536];

static dir_entry *root(void) {
    return (dir_entry *)disk[LBA_ROOT];
}

static uint32_t cluster_lba(uint32_t cluster) {
    return LBA_DATA + cluster - 2;
}

static void fat_set(uint32_t cluster, uint16_t value) {
    for (int f = 0; f < 2; f++) {
        uint8_t *fat = disk[LBA_FAT1 + f * FAT_BLOCKS];
        uint32_t off = cluster * 3 / 2;
        if (cluster & 1) {
            fat[off] = (uint8_t)((fat[off] & 0x0F) | (value << 4));
            fat[off + 1] = (uint8_t)(value >> 4);
        }
        else {
            fat[off] = (uint8_t)value;
            fat[off + 1] = (uint8_t)((fat[off + 1] & 0xF0) | ((value >> 8) & 0x0F));
        }
    }
}

static uint16_t fat_get(uint32_t cluster) {
    const uint8_t *fat = disk[LBA_FAT1];
    uint32_t off = cluster * 3 / 2;
    uint16_t v = (uint16_t)(fat[off] | (fat[off + 1] << 8));
    return (cluster & 1) ? (uint16_t)(v >> 4) : (uint16_t)(v & 0x0FFF);
}

// ---- building the volume ----

static void make_boot_sector(void) {
    uint8_t *b = disk[0];
    memcpy(b, "\xEB\x3C\x90" "MSWIN4.1", 11);
    b[11] = BLOCK_SIZE & 0xFF; b[12] = BLOCK_SIZE >> 8;   // bytes per sector
    b[13] = 1;                                            // sectors per cluster
    b[14] = 1; b[15] = 0;                                 // reserved sectors
    b[16] = 2;                                            // FATs
    b[17] = ROOT_ENTRIES & 0xFF; b[18] = ROOT_ENTRIES >> 8;
    b[19] = TOTAL_BLOCKS & 0xFF; b[20] = TOTAL_BLOCKS >> 8;
    b[21] = 0xF8;                                         // media: fixed disk
    b[22] = FAT_BLOCKS; b[23] = 0;
    b[24] = 1; b[26] = 1;                                 // sectors/track, heads
    b[36] = 0x80;                                         // drive number
    b[38] = 0x29;                                         // extended boot signature
    memcpy(&b[39], "\x26\x20\x0D\xB0", 4);                // volume serial
    memcpy(&b[43], "PICORVD    ", 11);
    memcpy(&b[54], "FAT12   ", 8);
    b[510] = 0x55; b[511] = 0xAA;
}

static int next_dir_slot;
static uint32_t next_cluster = 2;

// Adds a root directory entry and allocates `size` bytes of clusters for it,
// starting at `first` (0 = the next free RAM cluster). Returns the first
// cluster, or 0 if the file is empty.
static uint32_t add_entry(const char name83[11], uint8_t attr, uint32_t size, uint32_t first) {
    dir_entry *e = &root()[next_dir_slot++];
    memcpy(e->name, name83, 11);
    e->attr = attr;
    e->crt_date = e->acc_date = e->wrt_date = FAT_DATE;
    e->size = (attr & ATTR_VOLUME) ? 0 : size;
    if (!e->size) {
        return 0;
    }

    uint32_t clusters = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;
    if (!first) {
        first = next_cluster;
        next_cluster += clusters;
    }
    for (uint32_t i = 0; i < clusters; i++) {
        fat_set(first + i, i + 1 < clusters ? (uint16_t)(first + i + 1) : 0xFFF);
    }
    e->clus_lo = (uint16_t)first;
    return first;
}

static void add_ram_file(const char name83[11], uint8_t attr, const void *data, uint32_t size) {
    uint32_t first = add_entry(name83, attr, size, 0);
    if (first) {
        memcpy(disk[cluster_lba(first)], data, size);   // contiguous in RAM
    }
}

static const char readme_head[] =
    "PICORVD factory programmer\r\n"
    "==========================\r\n"
    "\r\n"
    "Copy a target firmware .bin onto this drive to make it the factory image.\r\n"
    "Edit and save CONFIG.TXT to change its settings. Once writes settle, the\r\n"
    "probe stores the new image, then restarts, and this drive reappears\r\n"
    "showing it. CURRENT.BIN is the stored image; LOG.CSV is the factory log\r\n"
    "as of this boot (read-only).\r\n"
    "\r\n";

void msc_disk_init(void) {
    memset(disk, 0, sizeof(disk));
    make_boot_sector();
    fat_set(0, 0xFF8);
    fat_set(1, 0xFFF);

    add_entry("PICORVD    ", ATTR_VOLUME, 0, 0);

    cur_have = factory_image_get(&cur_img);
    FactoryConfig cfg;
    if (cur_have) {
        cfg = cur_img.cfg;
    }
    else {
        factory_config_defaults(&cfg);
    }

    int n = snprintf(text, sizeof(text), "%s", readme_head);
    if (cur_have) {
        n += snprintf(text + n, sizeof(text) - n,
                      "Current image: %lu bytes, CRC-32 %08lX\r\nNote: %s\r\n",
                      (unsigned long)cur_img.size, (unsigned long)cur_img.crc, cur_img.cfg.note);
    }
    else {
        n += snprintf(text + n, sizeof(text) - n, "No factory image is stored yet.\r\n");
    }
    add_ram_file("README  TXT", ATTR_READONLY, text, (uint32_t)n);

    n = factory_config_format(&cfg, text, sizeof(text));
    add_ram_file("CONFIG  TXT", 0, text, (uint32_t)n);

    if (cur_have) {
        add_entry("CURRENT BIN", ATTR_READONLY, cur_img.size, CUR_FIRST_LBA - LBA_DATA + 2);
    }

    // LOG.CSV: header plus one row per record, as of now.
    log_rows = factory_log_count();
    log_size = sizeof(FACTORY_LOG_CSV_HEADER) - 1;
    char row[160];
    for (int i = 0; i < log_rows; i++) {
        log_size += (uint32_t)factory_log_csv_row(i, row, sizeof(row));
    }
    if (log_size > LOG_BLOCKS * BLOCK_SIZE) {
        log_size = LOG_BLOCKS * BLOCK_SIZE;
    }
    add_entry("LOG     CSV", ATTR_READONLY, log_size, LOG_FIRST_LBA - LBA_DATA + 2);

    // Every generated-area cluster not holding a file is marked bad, so the
    // host never allocates it: writes there are dropped, and an upload has
    // to land in RAM. The volume's free space is exactly the RAM that's free.
    for (uint32_t c = CUR_FIRST_LBA - LBA_DATA + 2; c < 2 + CLUSTERS; c++) {
        if (fat_get(c) == 0) {
            fat_set(c, 0xFF7);
        }
    }
}

// ---- LOG.CSV, generated on read ----

// Fills one block of LOG.CSV starting at byte `off` of the file.
static void log_read(uint32_t off, uint8_t *buf) {
    memset(buf, 0, BLOCK_SIZE);
    uint32_t end = off + BLOCK_SIZE, pos = 0;
    char row[160];

    for (int i = -1; i < log_rows && pos < end && pos < log_size; i++) {
        int n;
        if (i < 0) {
            n = (int)sizeof(FACTORY_LOG_CSV_HEADER) - 1;
            memcpy(row, FACTORY_LOG_CSV_HEADER, (size_t)n);
        }
        else {
            n = factory_log_csv_row(i, row, sizeof(row));
        }
        for (int k = 0; k < n; k++, pos++) {
            if (pos >= off && pos < end && pos < log_size) {
                buf[pos - off] = (uint8_t)row[k];
            }
        }
    }
}

// ---- TinyUSB mass storage callbacks ----

static volatile bool     dirty;           // host wrote since the last scan
static volatile uint32_t last_write_ms;
static volatile bool     ejected;

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
    (void)lun;
    memcpy(vendor_id, "PicoRVD ", 8);
    memcpy(product_id, "Factory image   ", 16);
    memcpy(product_rev, "1.0 ", 4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void)lun;
    return true;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    (void)lun;
    *block_count = TOTAL_BLOCKS;
    *block_size = BLOCK_SIZE;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) {
    (void)lun; (void)power_condition;
    if (load_eject && !start) {
        ejected = true;            // scan now rather than waiting for quiet
    }
    return true;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
    (void)lun;
    if (lba >= TOTAL_BLOCKS || offset + bufsize > BLOCK_SIZE) {
        return -1;
    }
    if (lba < RAM_BLOCKS) {
        memcpy(buffer, &disk[lba][offset], bufsize);
    }
    else if (lba < LOG_FIRST_LBA) {
        uint32_t pos = (lba - CUR_FIRST_LBA) * BLOCK_SIZE + offset;
        uint32_t have = cur_have && pos < cur_img.size ? cur_img.size - pos : 0;
        if (have > bufsize) {
            have = bufsize;
        }
        memcpy(buffer, cur_img.data + pos, have);
        memset((uint8_t *)buffer + have, 0, bufsize - have);
    }
    else {
        static uint8_t block[BLOCK_SIZE];
        log_read((lba - LOG_FIRST_LBA) * BLOCK_SIZE, block);
        memcpy(buffer, block + offset, bufsize);
    }
    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    (void)lun;
    if (lba >= TOTAL_BLOCKS || offset + bufsize > BLOCK_SIZE) {
        return -1;
    }
    if (lba < RAM_BLOCKS) {      // the generated blocks are read-only: writes vanish
        memcpy(&disk[lba][offset], buffer, bufsize);
        last_write_ms = to_ms_since_boot(get_absolute_time());
        dirty = true;
    }
    return (int32_t)bufsize;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize) {
    (void)buffer; (void)bufsize;
    // Everything TinyUSB doesn't handle itself: unsupported.
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    (void)scsi_cmd;
    return -1;
}

// ---- reading uploads back ----

#define QUIET_MS 1500          // host writes must settle this long before a scan

// Copies a root-directory file out of the RAM area. Returns its length, or
// -1 if its cluster chain is broken or still being written.
static int32_t read_file(const dir_entry *e, uint8_t *buf, uint32_t max) {
    uint32_t size = e->size, done = 0, cluster = e->clus_lo;
    if (size > max) {
        return -1;
    }
    while (done < size) {
        if (cluster < 2 || cluster >= 2 + CLUSTERS || cluster_lba(cluster) >= RAM_BLOCKS) {
            return -1;
        }
        uint32_t n = size - done < BLOCK_SIZE ? size - done : BLOCK_SIZE;
        memcpy(buf + done, disk[cluster_lba(cluster)], n);
        done += n;
        cluster = fat_get(cluster);
    }
    return (int32_t)size;
}

static bool live_file(const dir_entry *e) {
    return (uint8_t)e->name[0] != 0xE5 && e->attr != ATTR_LFN
        && !(e->attr & (ATTR_VOLUME | ATTR_DIR));
}

// A file's contents in place, if its clusters are contiguous and complete
// inside the RAM area -- which is how a host fills a fresh volume. NULL for a
// fragmented, truncated or still-being-written file.
static const uint8_t *file_data(const dir_entry *e) {
    uint32_t first = e->clus_lo, clusters = (e->size + BLOCK_SIZE - 1) / BLOCK_SIZE;
    if (first < 2 || cluster_lba(first) + clusters > RAM_BLOCKS) {
        return NULL;
    }
    for (uint32_t i = 0; i + 1 < clusters; i++) {
        if (fat_get(first + i) != first + i + 1) {
            return NULL;
        }
    }
    return disk[cluster_lba(first)];
}

// "NAME    BIN" -> "NAME.BIN"
static void name83_to_str(const char name83[11], char out[13]) {
    int n = 0;
    for (int i = 0; i < 8 && name83[i] != ' '; i++) out[n++] = name83[i];
    out[n++] = '.';
    for (int i = 8; i < 11 && name83[i] != ' '; i++) out[n++] = name83[i];
    out[n] = 0;
}

static void scan(void) {
    FactoryImage cur;
    bool have = factory_image_get(&cur);
    FactoryConfig cfg;
    if (have) {
        cfg = cur.cfg;
    }
    else {
        factory_config_defaults(&cfg);
    }
    FactoryConfig before = cfg;

    // Settings first, so a new image picks them up in the same pass.
    for (int i = 0; i < ROOT_ENTRIES && root()[i].name[0]; i++) {
        const dir_entry *e = &root()[i];
        if (live_file(e) && !memcmp(e->name, "CONFIG  TXT", 11)) {
            int32_t n = read_file(e, (uint8_t *)text, sizeof(text));
            if (n > 0) {
                factory_config_parse(text, (size_t)n, &cfg);
            }
        }
    }

    // The first .BIN whose contents differ from the stored image (usually
    // there is just the one new file). macOS's "._NAME.BIN" AppleDouble
    // files are skipped by content.
    const uint8_t *data = NULL;
    uint32_t size = 0;
    char name[13] = "";
    for (int i = 0; i < ROOT_ENTRIES && root()[i].name[0]; i++) {
        const dir_entry *e = &root()[i];
        if (!live_file(e) || memcmp(&e->name[8], "BIN", 3) || e->size == 0
            || cluster_lba(e->clus_lo) >= RAM_BLOCKS) {    // CURRENT.BIN itself
            continue;
        }
        name83_to_str(e->name, name);
        if (e->size > FACTORY_IMAGE_MAX) {
            printf("PICORVD drive: %s is %lu bytes, over the %u-byte limit; ignored\n",
                   name, (unsigned long)e->size, FACTORY_IMAGE_MAX);
            continue;
        }
        const uint8_t *d = file_data(e);
        if (!d) {
            printf("PICORVD drive: %s is fragmented or incomplete; ignored\n", name);
            continue;
        }
        if (e->size >= 4 && !memcmp(d, "\x00\x05\x16\x07", 4)) {
            continue;
        }
        if (have && e->size == cur.size && factory_crc32(d, e->size) == cur.crc) {
            continue;
        }
        data = d;
        size = e->size;
        break;
    }

    bool ok;
    if (data) {
        if (!strcmp(cfg.note, before.note) || !cfg.note[0]) {
            snprintf(cfg.note, sizeof(cfg.note), "%s crc %08lX", name,
                     (unsigned long)factory_crc32(data, size));
        }
        ok = factory_image_store(data, size, &cfg);
    }
    else if (have && memcmp(&cfg, &before, sizeof(cfg))) {
        ok = factory_config_store(&cfg);
    }
    else {
        return;                                   // nothing new
    }

    if (!ok) {
        printf("PICORVD drive: storing the factory image FAILED\n");
        return;
    }
    printf("PICORVD drive: factory image updated; restarting\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    stamped_reboot(CRASH_KIND_IMAGE_UPDATED);
}

void vTaskMscDisk(void *pvParams) {
    (void)pvParams;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(200));
        if (!dirty && !ejected) {
            continue;
        }
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (!ejected && now - last_write_ms < QUIET_MS) {
            continue;
        }
        // The factory loop reads the stored image while it flashes a board.
        FactoryLed led = factory_led_state();
        if (led == FACTORY_LED_BUSY || led == FACTORY_LED_BOOTING) {
            continue;
        }
        dirty = false;
        ejected = false;
        scan();
    }
}
