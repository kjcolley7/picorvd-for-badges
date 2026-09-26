/**
 * @file saoh_pico_hal.c
 * @brief RP2040 port of the saoh_core host HAL, plus a discovery shell command
 *
 * Runs the SAOv3 project's real host-side library against attached SAOs, rather
 * than the hand-rolled ARP commands next door. That matters: those only prove my
 * reading of the spec agrees with the device. This proves the actual badge code
 * and the actual device code agree with each other, which is the thing that has
 * to work.
 *
 * Only the three mandatory callbacks are implemented. SAOH_CFG_SUPPORT_SMBUS_BLKREAD
 * is left off, so saoh_core falls back to fixed/maximum-length reads it sizes
 * itself -- which suits the RP2040, whose I2C block NAKs the final byte of every
 * read and so cannot natively size a transfer from its first byte.
 */

#include "saoh/hal.h"
#include "saoh/smbus.h"
#include "saoh/discovery.h"
#include "saoh/consts/err.h"

#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>

// Logic analyzer trip wire; see la_commands.cpp. A no-op unless armed.
extern void la_note_write_read(uint8_t addr, const uint8_t *wdata, unsigned wlen,
                               const uint8_t *rdata, unsigned rlen, int rc);

// Same bus the i2c_* commands use; see i2c_commands.cpp for the wiring.
#define SAOH_I2C       i2c0
#define SAOH_TIMEOUT_US 50000

// Gap left after every transfer before the next may start.
//
// SMBus specifies a minimum bus-free time between STOP and the next START
// (Tbuf, 4.7us at 100kHz); this is deliberately far longer. A device that has
// real work to do on the STOP -- the CH32 SAO re-arms its I2C peripheral there
// after hand-bit-banging an ARP response -- has to be listening again before the
// next address goes out. Firing straight into the next transfer wins that race
// often enough that a device intermittently misses the round it should have
// answered, which reads as a device that simply is not there.
#define SAOH_BUS_FREE_US 2000

static void saoh_bus_settle(void) { sleep_us(SAOH_BUS_FREE_US); }

// Last read captured verbatim. A memcpy of a couple of dozen bytes, so unlike
// printing it this does not perturb the timing being measured.
volatile uint8_t  saoh_last_addr;
volatile uint16_t saoh_last_wlen;
volatile uint16_t saoh_last_rlen;
volatile int      saoh_last_rc;
volatile uint8_t  saoh_last_rx[24];

static void saoh_note_read(uint8_t addr, uint16_t wlen, uint16_t rlen, int rc, const uint8_t *rx)
{
    saoh_last_addr = addr;
    saoh_last_wlen = wlen;
    saoh_last_rlen = rlen;
    saoh_last_rc = rc;
    unsigned n = (rlen > sizeof(saoh_last_rx)) ? sizeof(saoh_last_rx) : rlen;
    for (unsigned i = 0; i < n; i++) saoh_last_rx[i] = rx[i];
}

// ---------------------------------------------------------------------------
// HAL

saoh_err_t saoh_i2c_write_cb(void *opaque, uint8_t addr, const uint8_t *data, xferlen_t len)
{
    (void) opaque;

    // A zero-length write is the SMBus quick command. The SDK rejects it, so
    // probe with a 1-byte read instead: either way it is only the address phase
    // that matters, and the ACK is the answer.
    if (len == 0) {
        uint8_t dummy;
        int r = i2c_read_timeout_us(SAOH_I2C, addr, &dummy, 1, false, SAOH_TIMEOUT_US);
        saoh_bus_settle();
        return (r < 0) ? SAOH_ERR_NAK : SAOH_ERR_OK;
    }

    int r = i2c_write_timeout_us(SAOH_I2C, addr, data, len, false, SAOH_TIMEOUT_US);
    saoh_bus_settle();
    if (r < 0) return SAOH_ERR_NAK;
    if (r != (int) len) return SAOH_ERR_SYS;
    return SAOH_ERR_OK;
}

saoh_err_t saoh_i2c_read_cb(void *opaque, uint8_t addr, uint8_t *data, xferlen_t len)
{
    (void) opaque;

    if (len == 0) {
        uint8_t dummy;
        int r = i2c_read_timeout_us(SAOH_I2C, addr, &dummy, 1, false, SAOH_TIMEOUT_US);
        saoh_bus_settle();
        return (r < 0) ? SAOH_ERR_NAK : SAOH_ERR_OK;
    }

    int r = i2c_read_timeout_us(SAOH_I2C, addr, data, len, false, SAOH_TIMEOUT_US);
    saoh_bus_settle();
    if (r < 0) return SAOH_ERR_NAK;
    if (r != (int) len) return SAOH_ERR_SYS;
    return SAOH_ERR_OK;
}

saoh_err_t saoh_i2c_write_read_cb(void *opaque, uint8_t addr, const uint8_t *wdata, xferlen_t wlen, uint8_t *rdata,
                                  xferlen_t rlen)
{
    (void) opaque;

    // nostop on the write so the read below issues a repeated START rather than
    // releasing the bus in between, which is what SMBus requires here.
    if (wlen > 0) {
        int w = i2c_write_timeout_us(SAOH_I2C, addr, wdata, wlen, true, SAOH_TIMEOUT_US);
        if (w < 0) return SAOH_ERR_NAK;
        if (w != (int) wlen) return SAOH_ERR_SYS;
    }

    if (rlen == 0) return SAOH_ERR_OK;

    // Deliberately one read for the whole response. Splitting it -- a byte for a
    // length, then the rest -- NAKs that first byte, which tells the device to
    // stop talking and makes a healthy device look like it died mid-reply.
    int r = i2c_read_timeout_us(SAOH_I2C, addr, rdata, rlen, false, SAOH_TIMEOUT_US);

    // Before the bus-free delay, so the logic analyzer's ring still ends on the
    // transaction that just went wrong rather than 2ms of idle bus.
    la_note_write_read(addr, wdata, wlen, rdata, rlen, r);

    saoh_bus_settle();
    saoh_note_read(addr, wlen, rlen, r, rdata);
    if (r < 0) return SAOH_ERR_NAK;
    if (r != (int) rlen) return SAOH_ERR_SYS;
    return SAOH_ERR_OK;
}

// ---------------------------------------------------------------------------
// Discovery, driven by the project's own host library

static saoh_bus_t saoh_bus;
static saoh_discovery_state_t saoh_state;
static int saoh_found_count;

// Set while looping to reproduce a failure. Printing over USB costs milliseconds
// per line and is more than enough to make the bug disappear, so the loop runs
// silent and only the run that fails gets narrated.
static int saoh_quiet;

static void on_discovered(void *arg, uint8_t pec_addr, const smbus_arp_udid_t *udid)
{
    (void) arg;
    saoh_found_count++;
    if (saoh_quiet) return;

    // Bit 7 of pec_addr flags PEC support; the address is the low 7 bits.
    printf("  found 0x%02X (pec %s) udid ", pec_addr & 0x7F, (pec_addr & 0x80) ? "yes" : "no");

    if (udid) {
        for (int i = 0; i < 16; i++) printf("%02X", udid->raw[i]);
    }
    else {
        printf("(none - non-ARP device)");
    }
    printf("\n");
}

static void on_removed(void *arg, uint8_t addr)
{
    (void) arg;
    if (saoh_quiet) return;
    printf("  removed 0x%02X\n", addr);
}

void saoh_log_reset(void);

// One discovery pass with no output at all; returns the device count. This is
// what the reproduce loop drives.
int saoh_pico_discover_quiet(int rescan)
{
    saoh_log_reset();
    saoh_quiet = 1;
    if (!saoh_state.bus) {
        memset(&saoh_bus, 0, sizeof(saoh_bus));
        memset(&saoh_state, 0, sizeof(saoh_state));
        saoh_bus.opaque = NULL;
        saoh_state.bus = &saoh_bus;
        saoh_state.discovery_cb = on_discovered;
        saoh_state.removal_cb = on_removed;
    }

    saoh_found_count = 0;
    if (rescan) saoh_discovery_rescan(&saoh_state);
    else        saoh_discovery_reset(&saoh_state);
    saoh_quiet = 0;

    return saoh_found_count;
}

// Discard all cached state, so the next pass re-runs ARP from scratch. Without
// this a rescan loop converges on "nothing changed" and stops exercising
// arbitration at all.
void saoh_pico_forget(void)
{
    memset(&saoh_state, 0, sizeof(saoh_state));
    saoh_state.bus = NULL;
}

void saoh_pico_discover(int rescan)
{
    saoh_log_reset();
    saoh_quiet = 0;
    if (!saoh_state.bus) {
        memset(&saoh_bus, 0, sizeof(saoh_bus));
        memset(&saoh_state, 0, sizeof(saoh_state));
        saoh_bus.opaque = NULL;
        saoh_state.bus = &saoh_bus;
        saoh_state.discovery_cb = on_discovered;
        saoh_state.removal_cb = on_removed;
    }

    saoh_found_count = 0;

    saoh_err_t err;
    if (rescan) {
        printf("saoh_discovery_rescan:\n");
        err = saoh_discovery_rescan(&saoh_state);
    }
    else {
        printf("saoh_discovery_reset (full bus scan):\n");
        err = saoh_discovery_reset(&saoh_state);
    }

    printf("result: %s, %d device(s)\n", (err == SAOH_ERR_OK) ? "OK" : "ERROR", saoh_found_count);
}

// ---------------------------------------------------------------------------
// Log capture. See saoh_log_capture.h for why this exists.

#undef printf

#include <stdarg.h>
#include "saoh/consts/sao.h"

#define SAOH_LOG_BUF_SZ 6144

static char saoh_log_buf[SAOH_LOG_BUF_SZ];
static volatile unsigned saoh_log_len;
static volatile unsigned saoh_log_dropped;

void saoh_log_reset(void)
{
    saoh_log_len = 0;
    saoh_log_dropped = 0;
    saoh_log_buf[0] = '\0';
}

int saoh_log_capture(const char *fmt, ...)
{
    if (saoh_log_len >= SAOH_LOG_BUF_SZ - 1) {
        saoh_log_dropped++;
        return 0;
    }

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(saoh_log_buf + saoh_log_len, SAOH_LOG_BUF_SZ - saoh_log_len, fmt, ap);
    va_end(ap);

    if (n > 0) {
        saoh_log_len += (unsigned) n;
        if (saoh_log_len >= SAOH_LOG_BUF_SZ) saoh_log_len = SAOH_LOG_BUF_SZ - 1;
    }
    return n;
}

void saoh_log_dump(void)
{
    printf("last write_read: addr=0x%02X wlen=%u rlen=%u rc=%d rx=", saoh_last_addr, saoh_last_wlen,
           saoh_last_rlen, saoh_last_rc);
    unsigned n = (saoh_last_rlen > 24) ? 24 : saoh_last_rlen;
    for (unsigned i = 0; i < n; i++) printf("%02X ", saoh_last_rx[i]);
    printf("\n");

    saoh_log_buf[SAOH_LOG_BUF_SZ - 1] = '\0';
    // Straight out in chunks; this runs after the bus work is finished, so its
    // cost cannot affect what was measured.
    for (unsigned i = 0; i < saoh_log_len; i += 128) {
        char tmp[129];
        unsigned n = saoh_log_len - i;
        if (n > 128) n = 128;
        memcpy(tmp, saoh_log_buf + i, n);
        tmp[n] = '\0';
        fputs(tmp, stdout);
    }
    if (saoh_log_dropped) fputs("\n[log truncated]\n", stdout);
    fputs("\n", stdout);
}

// ---------------------------------------------------------------------------
// Factory LED test via the standard SAOv3 LED class.
//
// Product-agnostic: ask the device which class interfaces it exposes, and if
// one is the LED class, take control of it and paint every LED the same level
// so the operator can eyeball the whole chain -- a dead or missing color die
// shows as a tinted LED against white neighbours (a red-less SK6812 reads
// cyan). Devices without the class are left alone.
//
// Returns 1 if the LEDs were painted, 0 if the device has no LED class (or an
// LED mode this does not know how to fill), and a negative saoh_err_t on a
// bus error.
int saoh_pico_leds_paint(uint8_t addr, uint8_t level)
{
    static saoh_bus_t bus;
    memset(&bus, 0, sizeof(bus));
    uint8_t pec_addr = (uint8_t) (addr | 0x80);   // bit 7 = use PEC (see on_discovered)

    uint8_t classes[32];
    int n = saoh_smbus_block_read_maxlen(&bus, pec_addr, SAO_CMNITF_CMD_GET_CLASS_INTERFACES,
                                         classes, sizeof(classes));
    if (n < 0) return n;

    int base = -1;
    for (int i = 0; i + 1 < n; i += 2) {         // {class id, base command} pairs
        if (classes[i] == SAO_CLASS_LED) { base = classes[i + 1]; break; }
    }
    if (base < 0) return 0;

    uint8_t cfg[3];                              // mode, count lo, count hi
    int rc = saoh_smbus_block_read_fixedlen(&bus, pec_addr,
                                            (uint8_t) (base + SAO_LEDITF_CMD_QUERY_CONFIG), cfg, 3);
    if (rc < 0) return rc;
    uint16_t count = (uint16_t) (cfg[1] | (cfg[2] << 8));

    uint8_t bpp, fill;
    switch (cfg[0]) {
        case SAO_LEDITF_MODE_1B_MONO: bpp = 1; fill = 1;     break;
        case SAO_LEDITF_MODE_8B_MONO: bpp = 1; fill = level; break;
        case SAO_LEDITF_MODE_8B_RGB:
        case SAO_LEDITF_MODE_8B_GRB:  bpp = 3; fill = level; break;
        default: return 0;
    }

    rc = saoh_smbus_write_byte(&bus, pec_addr, (uint8_t) (base + SAO_LEDITF_CMD_CONTROL_ENABLE), 1);
    if (rc != SAOH_ERR_OK) return rc;

    // The CH32V003 device port caps an SMBus block at 32 bytes; two bytes of
    // start index leave 30 for LED data, so 10 RGB LEDs per write.
    uint8_t blk[32];
    const uint16_t per = (uint16_t) (30 / bpp);
    for (uint16_t idx = 0; idx < count; idx += per) {
        uint16_t nled = (uint16_t) ((count - idx) < per ? (count - idx) : per);
        blk[0] = (uint8_t) (idx & 0xFF);
        blk[1] = (uint8_t) (idx >> 8);
        memset(&blk[2], fill, (size_t) nled * bpp);
        rc = saoh_smbus_block_write(&bus, pec_addr,
                                    (uint8_t) (base + SAO_LEDITF_CMD_LED_COMMAND_OFFSET),
                                    blk, (uint8_t) (2 + nled * bpp));
        if (rc != SAOH_ERR_OK) return rc;
    }
    return 1;
}
