/**
 * @file i2c_commands.cpp
 * @brief I2C master commands, for exercising an attached SAO as if this were a host badge
 *
 * Wiring on this setup: GP16 = I2C0 SDA, GP13 = I2C0 SCL, 3k pull-ups to 3V3.
 *
 * The peripheral is brought up lazily on first use rather than from
 * Application::init(). That is deliberate: it keeps the boot path -- USB, the
 * shell, the SWIO debugger -- byte-for-byte what it was, so a mistake in here
 * cannot cost the ability to reach the console and reflash over the 1200-baud
 * BOOTSEL trigger.
 */

#include "shell/Console.h"
#include "shell/console_colors.h"
#include "commands.h"

#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include <stdio.h>

#define SAO_I2C      i2c0
#define SAO_SDA_PIN  16
#define SAO_SCL_PIN  13
#define SAO_BAUD     100000   /* SMBus tops out here */

// Longer than any transfer needs, short enough that a stuck bus does not hang
// the shell. A wedged shell is recoverable now, but still worth not doing.
#define SAO_TIMEOUT_US 50000

static bool i2c_ready = false;

static void i2c_ensure_init(void) {
    if (i2c_ready) return;

    i2c_init(SAO_I2C, SAO_BAUD);

    gpio_set_function(SAO_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SAO_SCL_PIN, GPIO_FUNC_I2C);

    // The board already has 3k pull-ups; the internal ones are far too weak to
    // matter at 100kHz but cost nothing and help if a wire comes off.
    gpio_pull_up(SAO_SDA_PIN);
    gpio_pull_up(SAO_SCL_PIN);

    i2c_ready = true;
}

// SMBus PEC: CRC-8 over the whole transaction including address bytes,
// polynomial x^8 + x^2 + x + 1. Same routine the SAO library uses.
static uint8_t pec_update(uint8_t crc, uint8_t byte) {
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

//------------------------------------------------------------------------------

void command_i2c_scan(Console &c) {
    (void)c;
    i2c_ensure_init();

    printf("Scanning 0x08..0x77 on I2C0 (SDA=GP%d SCL=GP%d)\n", SAO_SDA_PIN, SAO_SCL_PIN);

    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t dummy = 0;
        // A zero-length write is the polite probe, but the SDK rejects it, so
        // use a 1-byte read instead: it only needs the address to be ACKed.
        int r = i2c_read_timeout_us(SAO_I2C, addr, &dummy, 1, false, SAO_TIMEOUT_US);
        if (r >= 0) {
            printf(COLOR_GREEN("  0x%02X responded") "\n", addr);
            found++;
        }
    }

    if (!found) {
        printf(COLOR_RED("  nothing responded") "\n");
    }
}

//------------------------------------------------------------------------------
// Raw access, for poking at things the helpers below do not cover.

void command_i2c_write(Console &c) {
    i2c_ensure_init();

    uint32_t addr = c.packet.take_hex();

    uint8_t buf[16];
    int len = 0;
    while (len < (int)sizeof(buf)) {
        auto v = c.packet.take_int();
        if (!v.is_ok()) break;
        buf[len++] = (uint8_t)v.ok_or(0);
    }

    if (len == 0) {
        printf(COLOR_RED("usage: i2c_w <hex_addr> <byte> [byte...]") "\n");
        return;
    }

    int r = i2c_write_timeout_us(SAO_I2C, (uint8_t)addr, buf, len, false, SAO_TIMEOUT_US);
    if (r < 0) {
        printf(COLOR_RED("write to 0x%02X failed (%d)") "\n", (unsigned)addr, r);
    }
    else {
        printf(COLOR_GREEN("wrote %d byte(s) to 0x%02X") "\n", r, (unsigned)addr);
    }
}

void command_i2c_read(Console &c) {
    i2c_ensure_init();

    uint32_t addr = c.packet.take_hex();
    int len = c.packet.take_int().ok_or(1);

    if (len < 1 || len > 32) {
        printf(COLOR_RED("usage: i2c_r <hex_addr> <len 1..32>") "\n");
        return;
    }

    uint8_t buf[32];
    int r = i2c_read_timeout_us(SAO_I2C, (uint8_t)addr, buf, len, false, SAO_TIMEOUT_US);
    if (r < 0) {
        printf(COLOR_RED("read from 0x%02X failed (%d)") "\n", (unsigned)addr, r);
        return;
    }

    printf(COLOR_GREEN("read %d:") " ", r);
    for (int i = 0; i < r; i++) printf("%02X ", buf[i]);
    printf("\n");
}

//------------------------------------------------------------------------------
// SMBus helpers aimed at the SAO. Each sends the PEC, since the device has PEC
// enabled, and reads back the trailing PEC byte on transfers that return data.

// Send Byte: just the command, no data. Used for the "simple" vendor commands.
void command_sao_cmd(Console &c) {
    i2c_ensure_init();

    uint32_t addr = c.packet.take_hex();
    uint32_t cmd = c.packet.take_hex();

    uint8_t pec = 0;
    pec = pec_update(pec, (uint8_t)(addr << 1));
    pec = pec_update(pec, (uint8_t)cmd);

    uint8_t buf[2] = { (uint8_t)cmd, pec };

    int r = i2c_write_timeout_us(SAO_I2C, (uint8_t)addr, buf, 2, false, SAO_TIMEOUT_US);
    if (r < 0) {
        printf(COLOR_RED("cmd 0x%02X to 0x%02X failed (%d)") "\n", (unsigned)cmd, (unsigned)addr, r);
    }
    else {
        printf(COLOR_GREEN("sent cmd 0x%02X to 0x%02X") "\n", (unsigned)cmd, (unsigned)addr);
    }
}

// Write Byte: command plus one data byte.
void command_sao_wb(Console &c) {
    i2c_ensure_init();

    uint32_t addr = c.packet.take_hex();
    uint32_t cmd = c.packet.take_hex();
    uint32_t val = c.packet.take_hex();

    uint8_t pec = 0;
    pec = pec_update(pec, (uint8_t)(addr << 1));
    pec = pec_update(pec, (uint8_t)cmd);
    pec = pec_update(pec, (uint8_t)val);

    uint8_t buf[3] = { (uint8_t)cmd, (uint8_t)val, pec };

    int r = i2c_write_timeout_us(SAO_I2C, (uint8_t)addr, buf, 3, false, SAO_TIMEOUT_US);
    if (r < 0) {
        printf(COLOR_RED("write 0x%02X=0x%02X to 0x%02X failed (%d)") "\n",
               (unsigned)cmd, (unsigned)val, (unsigned)addr, r);
    }
    else {
        printf(COLOR_GREEN("wrote 0x%02X = 0x%02X") "\n", (unsigned)cmd, (unsigned)val);
    }
}

// SMBus Send Byte with PEC, no console output: the factory loop's toggle.
bool sao_send_byte_quiet(uint8_t addr, uint8_t cmd) {
    i2c_ensure_init();

    uint8_t pec = 0;
    pec = pec_update(pec, (uint8_t)(addr << 1));
    pec = pec_update(pec, cmd);

    uint8_t buf[2] = { cmd, pec };
    return i2c_write_timeout_us(SAO_I2C, addr, buf, 2, false, SAO_TIMEOUT_US) == 2;
}

// SMBus Read Byte with PEC, no console output: for callers that poll a value
// programmatically rather than for a human.
// Returns true and fills *val only on a complete transfer with a valid PEC.
bool sao_read_byte_quiet(uint8_t addr, uint8_t cmd, uint8_t *val) {
    i2c_ensure_init();

    uint8_t cmdbuf = cmd;
    if (i2c_write_timeout_us(SAO_I2C, addr, &cmdbuf, 1, true, SAO_TIMEOUT_US) < 0) {
        return false;
    }

    uint8_t buf[2] = { 0, 0 };
    if (i2c_read_timeout_us(SAO_I2C, addr, buf, 2, false, SAO_TIMEOUT_US) < 0) {
        return false;
    }

    uint8_t pec = 0;
    pec = pec_update(pec, (uint8_t)(addr << 1));
    pec = pec_update(pec, cmd);
    pec = pec_update(pec, (uint8_t)((addr << 1) | 1));
    pec = pec_update(pec, buf[0]);
    if (buf[1] != pec) {
        return false;
    }

    *val = buf[0];
    return true;
}

// SMBus Read Word with PEC, no console output: for the factory loop's
// VID:PID check. Word is little-endian on the wire per SMBus.
bool sao_read_word_quiet(uint8_t addr, uint8_t cmd, uint16_t *val) {
    i2c_ensure_init();

    uint8_t cmdbuf = cmd;
    if (i2c_write_timeout_us(SAO_I2C, addr, &cmdbuf, 1, true, SAO_TIMEOUT_US) < 0) {
        return false;
    }

    uint8_t buf[3] = { 0, 0, 0 };
    if (i2c_read_timeout_us(SAO_I2C, addr, buf, 3, false, SAO_TIMEOUT_US) < 0) {
        return false;
    }

    uint8_t pec = 0;
    pec = pec_update(pec, (uint8_t)(addr << 1));
    pec = pec_update(pec, cmd);
    pec = pec_update(pec, (uint8_t)((addr << 1) | 1));
    pec = pec_update(pec, buf[0]);
    pec = pec_update(pec, buf[1]);
    if (buf[2] != pec) {
        return false;
    }

    *val = (uint16_t)buf[0] | (uint16_t)((uint16_t)buf[1] << 8);
    return true;
}

// Read Byte: write the command, repeated START, then read data + PEC.
void command_sao_rb(Console &c) {
    i2c_ensure_init();

    uint32_t addr = c.packet.take_hex();
    uint32_t cmd = c.packet.take_hex();

    uint8_t cmdbuf = (uint8_t)cmd;
    // nostop=true leaves the bus held for the repeated START below.
    int w = i2c_write_timeout_us(SAO_I2C, (uint8_t)addr, &cmdbuf, 1, true, SAO_TIMEOUT_US);
    if (w < 0) {
        printf(COLOR_RED("cmd phase failed (%d)") "\n", w);
        return;
    }

    uint8_t buf[2] = { 0, 0 };
    int r = i2c_read_timeout_us(SAO_I2C, (uint8_t)addr, buf, 2, false, SAO_TIMEOUT_US);
    if (r < 0) {
        printf(COLOR_RED("read phase failed (%d)") "\n", r);
        return;
    }

    uint8_t pec = 0;
    pec = pec_update(pec, (uint8_t)(addr << 1));
    pec = pec_update(pec, (uint8_t)cmd);
    pec = pec_update(pec, (uint8_t)((addr << 1) | 1));
    pec = pec_update(pec, buf[0]);

    printf(COLOR_GREEN("0x%02X = 0x%02X") "  (pec rx %02X, calc %02X %s)\n",
           (unsigned)cmd, buf[0], buf[1], pec,
           (buf[1] == pec) ? "ok" : "MISMATCH");
}

extern "C" void la_stop(const char *why);

//------------------------------------------------------------------------------
// A master read that keeps the abort reason.
//
// The SDK's i2c_read_timeout_us reads IC_TX_ABRT_SOURCE and then clears it on
// its way out, so by the time it returns PICO_ERROR_GENERIC the one register
// that says *why* is already gone. During a read there are only a handful of
// plausible causes and they mean completely different things -- a slave that
// stopped answering, a lost arbitration, a controller that was starved -- so
// losing that register turns a specific fault into "it failed".

static uint32_t sao_abrt_source;
static int      sao_abrt_index;

static const char *abrt_bit_name(int bit) {
    switch (bit) {
        case 0:  return "7B_ADDR_NOACK";
        case 3:  return "TXDATA_NOACK";
        case 7:  return "SBYTE_ACKDET";
        case 9:  return "SBYTE_NORSTRT";
        case 10: return "10B_RD_NORSTRT";
        case 11: return "MASTER_DIS";
        case 12: return "ARB_LOST";
        case 13: return "SLVFLUSH_TXFIFO";
        case 14: return "SLV_ARBLOST";
        case 15: return "SLVRD_INTX";
        case 16: return "USER_ABRT";
        default: return NULL;
    }
}

static void print_abrt(void) {
    printf("  abort at byte %d, IC_TX_ABRT_SOURCE=0x%08lX:", sao_abrt_index, (unsigned long)sao_abrt_source);
    for (int b = 0; b < 17; b++) {
        if (!(sao_abrt_source & (1u << b))) continue;
        const char *n = abrt_bit_name(b);
        if (n) printf(" %s", n);
        else   printf(" bit%d", b);
    }
    printf("\n");
}

static int sao_i2c_read_dbg(uint8_t addr, uint8_t *dst, int len) {
    i2c_hw_t *hw = i2c_get_hw(SAO_I2C);

    hw->enable = 0;
    hw->tar = addr;
    hw->enable = 1;

    sao_abrt_source = 0;
    sao_abrt_index = -1;

    bool abort = false;
    int i = 0;

    for (; i < len; i++) {
        bool first = (i == 0);
        bool last = (i == len - 1);

        while (!i2c_get_write_available(SAO_I2C)) tight_loop_contents();

        hw->data_cmd = ((first && SAO_I2C->restart_on_next) ? (1u << I2C_IC_DATA_CMD_RESTART_LSB) : 0u) |
                       (last ? (1u << I2C_IC_DATA_CMD_STOP_LSB) : 0u) |
                       I2C_IC_DATA_CMD_CMD_BITS;

        uint32_t spins = 0;
        do {
            uint32_t src = hw->tx_abrt_source;
            if (src) {
                sao_abrt_source = src;
                sao_abrt_index = i;
                hw->clr_tx_abrt;
                abort = true;
                break;
            }
            if (++spins > 40000000u) {
                sao_abrt_source = 0;
                sao_abrt_index = i;
                abort = true;
                break;
            }
        } while (!i2c_get_read_available(SAO_I2C));

        if (abort) break;
        *dst++ = (uint8_t)hw->data_cmd;
    }

    SAO_I2C->restart_on_next = false;
    return abort ? -1 : len;
}

//------------------------------------------------------------------------------
// SMBus ARP, for reproducing enumeration behavior with several devices present.

#define ARP_ADDR        0x61
#define ARP_PREPARE     0x01
#define ARP_GET_UDID    0x03
#define ARP_UDID_BYTES  17   /* 16 UDID + 1 address byte */

// Prepare to ARP: clears every device's address-resolved flag so they all
// contend again on the next Get UDID.
static bool arp_prepare(void) {
    uint8_t pec = 0;
    pec = pec_update(pec, (uint8_t)(ARP_ADDR << 1));
    pec = pec_update(pec, ARP_PREPARE);

    uint8_t buf[2] = { ARP_PREPARE, pec };
    return i2c_write_timeout_us(SAO_I2C, ARP_ADDR, buf, 2, false, SAO_TIMEOUT_US) == 2;
}

// Get UDID (general). Every unresolved device answers at once and the wired-AND
// on SDA decides the winner, so this is the transfer that needs arbitration.
//
// Block read: count byte first, then that many bytes, then the PEC. The count is
// read on its own with the bus held so the length is known before asking for the
// rest.
static int arp_get_udid(uint8_t *out, uint8_t *pec_ok) {
    uint8_t cmd = ARP_GET_UDID;

    if (i2c_write_timeout_us(SAO_I2C, ARP_ADDR, &cmd, 1, true, SAO_TIMEOUT_US) != 1) {
        return -1;
    }

    // One read for the whole block: count, payload, PEC.
    //
    // Emphatically NOT a 1-byte read for the count followed by a second read for
    // the rest. Every i2c_read call NAKs its final byte, so splitting it NAKs the
    // count -- which correctly tells the device to stop talking. The rest of the
    // transfer then reads back as all ones, which looks exactly like a slave that
    // died one byte in. The length is fixed for Get UDID, so read it in one go
    // and check the count afterwards.
    uint8_t body[ARP_UDID_BYTES + 2];   // count + payload + PEC
    int want = ARP_UDID_BYTES + 2;

    int got = sao_i2c_read_dbg(ARP_ADDR, body, want);

    // Freeze the capture before printing anything. Diagnostics go out over USB
    // and cost milliseconds, which is enough to push the transfer being
    // diagnosed out of a 32ms ring.
    la_stop(got == want ? "get-udid ok" : "get-udid aborted");

    if (got != want) {
        print_abrt();
        // Report what actually came back, not just "it failed" -- a short read
        // and a NAK are very different faults.
        printf("  got %d of %d bytes:", sao_abrt_index < 0 ? 0 : sao_abrt_index, want);
        for (int i = 0; i < sao_abrt_index && i < want; i++) printf(" %02X", body[i]);
        printf("\n");
        return -2;
    }

    if (body[0] != ARP_UDID_BYTES) {
        return -(100 + body[0]);
    }

    uint8_t pec = 0;
    pec = pec_update(pec, (uint8_t)(ARP_ADDR << 1));
    pec = pec_update(pec, ARP_GET_UDID);
    pec = pec_update(pec, (uint8_t)((ARP_ADDR << 1) | 1));
    for (int i = 0; i < ARP_UDID_BYTES + 1; i++) pec = pec_update(pec, body[i]);

    *pec_ok = (body[ARP_UDID_BYTES + 1] == pec) ? 1 : 0;
    for (int i = 0; i < ARP_UDID_BYTES; i++) out[i] = body[i + 1];
    return ARP_UDID_BYTES;
}

void command_arp_udid(Console &c) {
    i2c_ensure_init();
    (void)c;

    if (!arp_prepare()) {
        la_stop("arp_udid: prepare failed");
        printf(COLOR_RED("prepare-to-arp failed") "\n");
        return;
    }

    uint8_t udid[ARP_UDID_BYTES];
    uint8_t pec_ok = 0;
    int r = arp_get_udid(udid, &pec_ok);

    // Freeze the analyzer here rather than leaving it to a separate command:
    // the ring only holds 32ms, so anything typed afterwards arrives far too
    // late to still contain the transfer.
    la_stop(r < 0 ? "arp_udid: get-udid failed" : "arp_udid: ok");

    if (r < 0) {
        printf(COLOR_RED("get-udid failed (%d)") "\n", r);
        return;
    }

    printf(COLOR_GREEN("udid:") " ");
    for (int i = 0; i < ARP_UDID_BYTES; i++) printf("%02X", udid[i]);
    printf("  pec %s\n", pec_ok ? "ok" : COLOR_RED("BAD"));
}

// Hammer Get UDID and report how often it comes back intact. With more than one
// unresolved device on the bus this is the arbitration path, so the pass rate is
// the number that matters.
void command_arp_test(Console &c) {
    i2c_ensure_init();

    int runs = c.packet.take_int().ok_or(20);
    if (runs < 1 || runs > 500) runs = 20;

    int ok = 0, bad_pec = 0, failed = 0;
    uint8_t first[ARP_UDID_BYTES];
    bool have_first = false;
    int differing = 0;

    for (int i = 0; i < runs; i++) {
        if (!arp_prepare()) { failed++; continue; }

        uint8_t udid[ARP_UDID_BYTES];
        uint8_t pec_ok = 0;
        int r = arp_get_udid(udid, &pec_ok);

        if (r < 0)        { failed++;  continue; }
        if (!pec_ok)      { bad_pec++; continue; }

        ok++;
        if (!have_first) {
            for (int j = 0; j < ARP_UDID_BYTES; j++) first[j] = udid[j];
            have_first = true;
        }
        else {
            for (int j = 0; j < ARP_UDID_BYTES; j++) {
                if (udid[j] != first[j]) { differing++; break; }
            }
        }
    }

    printf("runs %d: " COLOR_GREEN("ok %d") "  bad-pec %d  failed %d\n", runs, ok, bad_pec, failed);
    if (have_first) {
        printf("winner: ");
        for (int i = 0; i < ARP_UDID_BYTES; i++) printf("%02X", first[i]);
        printf("\n");
        // Arbitration is deterministic -- the lowest UDID always wins -- so a
        // varying winner means someone is mis-arbitrating.
        printf("%s\n", differing ? COLOR_RED("winner VARIED between runs") : "winner consistent");
    }
}

// Assign Address: block write of the winner's UDID plus the address it should take.
static bool arp_assign(const uint8_t *udid, uint8_t new_addr) {
    uint8_t buf[24];
    int n = 0;
    buf[n++] = 0x04;                 // Assign Address
    buf[n++] = ARP_UDID_BYTES;       // block count
    for (int i = 0; i < ARP_UDID_BYTES - 1; i++) buf[n++] = udid[i];
    buf[n++] = (uint8_t)(new_addr << 1);

    uint8_t pec = 0;
    pec = pec_update(pec, (uint8_t)(ARP_ADDR << 1));
    for (int i = 0; i < n; i++) pec = pec_update(pec, buf[i]);
    buf[n++] = pec;

    return i2c_write_timeout_us(SAO_I2C, ARP_ADDR, buf, n, false, SAO_TIMEOUT_US) == n;
}

// Full enumeration, the way a badge does it: keep asking for a UDID and assigning
// the winner an address until nobody answers. This is what actually proves
// arbitration works -- with a single device it would pass trivially, but with
// several, each round must surface a DIFFERENT device as the previous winner
// resolves and drops out of the contention.
void command_arp_enum(Console &c) {
    i2c_ensure_init();
    (void)c;

    if (!arp_prepare()) {
        printf(COLOR_RED("prepare-to-arp failed") "\n");
        return;
    }

    for (int round = 0; round < 6; round++) {
        uint8_t udid[ARP_UDID_BYTES];
        uint8_t pec_ok = 0;
        int r = arp_get_udid(udid, &pec_ok);

        if (r < 0) {
            printf("round %d: no more devices (%d)\n", round, r);
            break;
        }

        uint8_t addr = (uint8_t)(0x10 + round);
        printf("round %d: ", round);
        for (int i = 0; i < ARP_UDID_BYTES - 1; i++) printf("%02X", udid[i]);
        printf("  pec %s -> assign 0x%02X ", pec_ok ? "ok" : COLOR_RED("BAD"), addr);
        printf("%s\n", arp_assign(udid, addr) ? COLOR_GREEN("ok") : COLOR_RED("FAILED"));
    }
}

//------------------------------------------------------------------------------
// Discovery via the SAOv3 project's own host library (see saoh_pico_hal.c).

extern "C" void saoh_pico_discover(int rescan);
extern "C" void saoh_log_dump(void);

void command_sao_discover(Console &c) {
    i2c_ensure_init();
    saoh_pico_discover(c.packet.take_int().ok_or(0));
}

void command_sao_log(Console &c) {
    (void)c;
    saoh_log_dump();
}

extern "C" int  saoh_pico_discover_quiet(int rescan);
extern "C" void saoh_pico_forget(void);
extern "C" void la_arm(const char *why);
extern "C" bool la_triggered(void);

// Run discovery over and over with the logic analyzer armed, and stop the moment
// a pass comes back with fewer devices than the one before it. The point is to
// leave the ring buffer holding the transfers that produced the failure, without
// a human having to be watching when it happens.
void command_sao_loop(Console &c) {
    i2c_ensure_init();

    int runs   = c.packet.take_int().ok_or(40);
    int expect = c.packet.take_int().ok_or(2);
    if (runs < 1 || runs > 2000) runs = 40;

    printf("looping discovery, expecting %d device(s), up to %d run(s)\n", expect, runs);

    int ok = 0;
    for (int i = 0; i < runs; i++) {
        la_arm("armed, run in progress");
        saoh_pico_forget();
        int found = saoh_pico_discover_quiet(0);

        if (found >= expect) {
            ok++;
            continue;
        }

        printf(COLOR_RED("run %d FAILED") ": found %d, expected %d  (%d/%d ok before this)\n",
               i + 1, found, expect, ok, i);
        printf("capture held; use " COLOR_YELLOW("la_dump") " / " COLOR_YELLOW("la_raw") " / "
               COLOR_YELLOW("sao_log") "\n");
        return;
    }

    printf(COLOR_GREEN("%d/%d runs ok") ", no failure captured\n", ok, runs);
}
