#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// The target firmware the factory loop flashes, plus its per-product
// settings, stored in the probe's own flash (factory_image.cpp). Uploaded at
// runtime over USB mass storage (usb/msc_disk.cpp) -- nothing is compiled in.

// A CH32V003 image must end before the target's last 64-byte flash page,
// which holds the target's own settings and is never erased.
#define FACTORY_IMAGE_MAX (16 * 1024 - 64)

struct FactoryConfig {
    char     note[64];       // the log's firmware column
    int16_t  uart_selftest;  // standalone: byte sent over SWIO-as-UART after boot, -1 = none
    uint32_t uart_baud;
    uint16_t vid;            // tethered: expected SAOv3 VID/PID after boot, 0 = any
    uint16_t pid;
    int16_t  selftest_cmd;   // tethered: SAOv3 vendor self-test command, -1 = none
};

struct FactoryImage {
    const uint8_t *data;     // points into XIP flash
    uint32_t size;
    uint32_t crc;            // CRC-32 of data
    FactoryConfig cfg;
};

// The stored image, or false if none is stored (or it fails its CRC).
bool factory_image_get(FactoryImage *out);

// Erase and program the image region, then verify it reads back. Must be
// called from a FreeRTOS task (uses flash_safe_execute). data may not point
// into the image region itself.
bool factory_image_store(const uint8_t *data, uint32_t size, const FactoryConfig *cfg);

// Replace just the settings of the stored image: rewrites the header sector
// only, so the image itself never needs a copy in RAM.
bool factory_config_store(const FactoryConfig *cfg);

void factory_config_defaults(FactoryConfig *cfg);

// CONFIG.TXT: format the settings as editable "key = value" text, and parse
// them back (only keys present are changed).
int factory_config_format(const FactoryConfig *cfg, char *buf, size_t len);
void factory_config_parse(const char *text, size_t len, FactoryConfig *cfg);

uint32_t factory_crc32(const void *data, size_t len);

// Console summary of what is stored (the `factory image` command).
void factory_image_print(void);
