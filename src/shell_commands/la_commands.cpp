/**
 * @file la_commands.cpp
 * @brief A two-channel logic analyzer on the I2C bus this probe is mastering
 *
 * The ARP arbitration workaround on the CH32 SAO fails perhaps one time in ten,
 * and every attempt to instrument it from inside the device firmware moved the
 * failure instead of exposing it. The bus itself is the only observer that costs
 * the device nothing, so: PIO1 free-runs a sampler over SCL and SDA into a DMA
 * write-ring, and the host HAL trips it the instant a Get UDID comes back wrong.
 * What survives in the ring is the couple of transfers leading up to the failure,
 * captured with no timing penalty whatsoever.
 *
 * PIO0 belongs to the SWIO debugger and is not touched.
 *
 * The ring is 32 KB because that is the RP2040 DMA's largest RING_SIZE; at 2 Msps
 * that is 32.7 ms of history, which comfortably spans Prepare-to-ARP through the
 * failing round of Get UDID with the 2 ms bus-free gap between each.
 */

#include "shell/Console.h"
#include "shell/console_colors.h"
#include "commands.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include "la_capture.pio.h"

#include <stdio.h>
#include <string.h>

// Must match the wiring in i2c_commands.cpp. SCL is the IN base so it lands in
// bit 0 of every sample; SDA is three pins further up, hence a 4-bit sample.
#define LA_PIN_BASE   13
#define LA_PIN_WIDTH  4
#define LA_BIT_SCL    (1u << 0)   /* GP13 */
#define LA_BIT_SDA    (1u << 3)   /* GP16 */

#define LA_RING_BITS   15                        /* 32 KB - the DMA's maximum */
#define LA_RING_BYTES  (1u << LA_RING_BITS)
#define LA_WORDS       (LA_RING_BYTES / 4)
#define LA_SAMPLES     (LA_WORDS * 8)            /* 8 nibbles per word */

#define LA_RATE_HZ     2000000u
#define LA_XFER_COUNT  0x0FFFFFFFu               /* effectively unbounded */

// The DMA write-ring requires the buffer aligned to its own size. That costs up
// to 32 KB of padding in .bss, which came straight out of the newlib heap and
// was enough on its own to stop the probe booting -- Console and GDBServer alloc
// 48 KB of packet buffers between them. FreeRTOS's heap was cut to match; see
// configTOTAL_HEAP_SIZE.
static uint32_t la_buf[LA_WORDS] __attribute__((aligned(LA_RING_BYTES)));

static PIO      la_pio = pio1;
static int      la_sm  = -1;
static int      la_dma = -1;
static uint     la_offset;

static volatile bool     la_armed;
static volatile bool     la_have;        /* a completed capture is in the buffer */
static unsigned          la_oldest;      /* index of oldest valid sample */
static unsigned          la_valid;       /* number of valid samples */
static float             la_rate_actual;
static char              la_reason[48];

//------------------------------------------------------------------------------

static bool la_hw_init(void) {
    if (la_sm >= 0) return true;

    int sm = pio_claim_unused_sm(la_pio, false);
    if (sm < 0) return false;

    int dma = dma_claim_unused_channel(false);
    if (dma < 0) { pio_sm_unclaim(la_pio, sm); return false; }

    la_offset = pio_add_program(la_pio, &la4_program);

    pio_sm_config c = la4_program_get_default_config(la_offset);
    sm_config_set_in_pins(&c, LA_PIN_BASE);
    // Shift right with autopush: sample N lands in nibble N, so the oldest
    // sample in a word is the low nibble. Nothing else in here has to care about
    // FIFO ordering, which is worth the one line of explanation.
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

    float div = (float)clock_get_hz(clk_sys) / (float)LA_RATE_HZ;
    sm_config_set_clkdiv(&c, div);
    la_rate_actual = (float)clock_get_hz(clk_sys) / div;

    pio_sm_init(la_pio, sm, la_offset, &c);

    la_sm = sm;
    la_dma = dma;
    return true;
}

extern "C" void la_arm(const char *why) {
    if (!la_hw_init()) return;

    pio_sm_set_enabled(la_pio, la_sm, false);
    if (dma_channel_is_busy(la_dma)) dma_channel_abort(la_dma);

    pio_sm_clear_fifos(la_pio, la_sm);
    pio_sm_restart(la_pio, la_sm);
    pio_sm_exec(la_pio, la_sm, pio_encode_jmp(la_offset));

    dma_channel_config dc = dma_channel_get_default_config(la_dma);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_ring(&dc, true, LA_RING_BITS);
    channel_config_set_dreq(&dc, pio_get_dreq(la_pio, la_sm, false));

    dma_channel_configure(la_dma, &dc, la_buf, &la_pio->rxf[la_sm], LA_XFER_COUNT, true);

    // DMA first, then the sampler: the other order spills the first few samples
    // into a FIFO nobody is draining yet.
    pio_sm_set_enabled(la_pio, la_sm, true);

    la_have = false;
    la_valid = 0;
    snprintf(la_reason, sizeof(la_reason), "%s", why ? why : "manual");
    la_armed = true;
}

// Freeze the ring and work out which end of it is the newest sample. Called from
// the middle of a failing transaction, so it does the absolute minimum: two
// register writes and a pointer read.
// Sampling continues briefly past the trigger. Whatever went wrong, how the bus
// recovers from it -- a STOP, a stall, a NAK -- is part of the evidence, and it
// happens after the call that noticed the problem has already returned.
#define LA_POST_TRIGGER_US 600

extern "C" void la_stop(const char *why) {
    if (!la_armed) return;
    la_armed = false;

    busy_wait_us(LA_POST_TRIGGER_US);

    pio_sm_set_enabled(la_pio, la_sm, false);

    uint32_t remaining = dma_hw->ch[la_dma].transfer_count;
    uint32_t wr = dma_hw->ch[la_dma].write_addr;
    dma_channel_abort(la_dma);

    uint32_t done_words = LA_XFER_COUNT - remaining;

    // write_addr is where the next word would have gone, so it is simultaneously
    // the oldest word once the ring has wrapped.
    unsigned next_word = (unsigned)((wr - (uint32_t)(uintptr_t)la_buf) / 4) % LA_WORDS;

    if (done_words >= LA_WORDS) {
        la_oldest = next_word * 8;
        la_valid  = LA_SAMPLES;
    }
    else {
        la_oldest = 0;
        la_valid  = done_words * 8;
    }

    if (why) snprintf(la_reason, sizeof(la_reason), "%s", why);
    la_have = true;
}

// Trip point for the host HAL: a Get UDID whose block-count byte is not the 17 a
// healthy device sends is exactly the failure being hunted, and stopping here
// means the ring still holds the transfer that produced it.
extern "C" void la_note_write_read(uint8_t addr, const uint8_t *wdata, unsigned wlen,
                                   const uint8_t *rdata, unsigned rlen, int rc) {
    if (!la_armed) return;
    if (addr != 0x61 || wlen < 1 || wdata[0] != 0x03) return;

    // A NAK is NOT a failure here. Enumeration ends with a Get UDID that nobody
    // answers -- that is how the host learns there is nothing left to find. Only
    // a device that answers with the wrong block count is the fault being hunted.
    char why[48];
    if (rc >= 0 && rlen >= 1 && rdata[0] != 17) {
        snprintf(why, sizeof(why), "get-udid count=0x%02X (want 0x11)", rdata[0]);
    }
    else {
        return;
    }
    la_stop(why);
}

extern "C" bool la_triggered(void) {
    return la_have && !la_armed;
}

//------------------------------------------------------------------------------
// Decode

static inline unsigned la_sample(unsigned i) {
    unsigned idx = (la_oldest + i) % LA_SAMPLES;
    return (la_buf[idx >> 3] >> ((idx & 7) * 4)) & 0xF;
}

static inline float la_us(unsigned i) {
    // Time relative to the end of the capture, i.e. to the moment of the trigger.
    return ((float)i - (float)la_valid) / (la_rate_actual / 1000000.0f);
}

struct Decoder {
    bool     in_frame = false;
    int      bitcnt = 0;
    unsigned shifter = 0;
    int      nbytes = 0;
    unsigned fall_idx = 0;
    bool     had_fall = false;
    unsigned max_stretch = 0;

    // Data setup time: how long SDA had been stable before SCL rose. I2C wants
    // at least 250ns; a transmitter that changes SDA *at* the rising edge is
    // indistinguishable from a START or STOP, which is exactly how a master
    // ends up reporting arbitration lost in the middle of a read it is winning.
    unsigned last_change = 0;
    unsigned min_setup = 0xFFFFFFFF;
    int      min_setup_bit = -1;

    void flush_line() {
        if (in_frame) printf("\n");
        in_frame = false;
    }
};

// I2C is decoded from edges alone: START and STOP are SDA transitions while SCL
// is high, and every other bit is whatever SDA holds when SCL rises.
static void la_decode(int max_lines) {
    Decoder d;
    unsigned prev = la_sample(0);
    int lines = 0;

    for (unsigned i = 1; i < la_valid && lines < max_lines; i++) {
        unsigned cur = la_sample(i);
        bool scl = cur & LA_BIT_SCL, psc = prev & LA_BIT_SCL;
        bool sda = cur & LA_BIT_SDA, psd = prev & LA_BIT_SDA;

        if (scl && psc && sda != psd) {
            if (!sda) {
                // START, or a repeated START if we were already inside a frame.
                if (d.in_frame) {
                    printf(" Sr");
                }
                else {
                    printf("%9.1f  S", la_us(i));
                    d.in_frame = true;
                    lines++;
                }
                d.bitcnt = 0;
                d.shifter = 0;
                d.nbytes = 0;
            }
            else {
                printf(" P\n");
                d.in_frame = false;
                d.bitcnt = 0;
            }
        }
        else if (scl && !psc) {
            if (d.had_fall) {
                unsigned low = i - d.fall_idx;
                if (low > d.max_stretch) d.max_stretch = low;
                // The master's own low period is ~5us at 100kHz; anything much
                // beyond that is the slave holding the clock down.
                if (low > (unsigned)(30.0f * la_rate_actual / 1000000.0f) && d.in_frame) {
                    printf(" " COLOR_YELLOW("[stretch %.0fus]"), (float)low / (la_rate_actual / 1000000.0f));
                }
                d.had_fall = false;
            }

            if (!d.in_frame) { prev = cur; continue; }

            if (d.bitcnt < 8) {
                unsigned setup = i - d.last_change;
                if (setup < d.min_setup) {
                    d.min_setup = setup;
                    d.min_setup_bit = 7 - d.bitcnt;
                }
                d.shifter = (d.shifter << 1) | (sda ? 1u : 0u);
                d.bitcnt++;
            }
            else {
                // Ninth clock: the acknowledge slot.
                bool ack = !sda;
                const char *tag = "";
                if (d.nbytes == 0) tag = (d.shifter & 1) ? "+R" : "+W";
                printf(" %02X%s%c", d.shifter, tag, ack ? 'a' : 'n');
                if (d.min_setup < (unsigned)(3.0f * la_rate_actual / 1000000.0f)) {
                    printf(COLOR_RED("!su=%.1fus@bit%d"),
                           (float)d.min_setup / (la_rate_actual / 1000000.0f), d.min_setup_bit);
                }
                d.nbytes++;
                d.bitcnt = 0;
                d.shifter = 0;
                d.min_setup = 0xFFFFFFFF;
                d.min_setup_bit = -1;
            }
        }
        else if (!scl && psc) {
            d.fall_idx = i;
            d.had_fall = true;
        }

        if (sda != psd) d.last_change = i;

        prev = cur;
    }

    d.flush_line();
}

//------------------------------------------------------------------------------
// Commands

void command_la_arm(Console &c) {
    (void)c;
    if (!la_hw_init()) {
        printf(COLOR_RED("no free PIO state machine or DMA channel") "\n");
        return;
    }
    la_arm("manual");
    printf("armed: %.3f Msps, %u samples (%.1f ms window), SCL=GP%d SDA=GP%d\n",
           la_rate_actual / 1e6f, LA_SAMPLES, LA_SAMPLES / (la_rate_actual / 1000.0f),
           LA_PIN_BASE, LA_PIN_BASE + 3);
}

void command_la_stop(Console &c) {
    (void)c;
    if (!la_armed) { printf("not armed\n"); return; }
    la_stop("manual");
    printf("stopped, %u samples valid\n", la_valid);
}

void command_la_dump(Console &c) {
    if (la_armed) la_stop("dump while armed");
    if (!la_have || !la_valid) { printf("no capture\n"); return; }

    int max_lines = c.packet.take_int().ok_or(200);

    printf("capture: %.3f Msps, %u samples (%.2f ms), trigger: " COLOR_YELLOW("%s") "\n",
           la_rate_actual / 1e6f, la_valid, la_valid / (la_rate_actual / 1000.0f), la_reason);
    printf("times are microseconds relative to the trigger\n");
    la_decode(max_lines);
}

// Raw waveform, for the places the decoder has nothing useful to say -- a
// half-driven bit, a START that does not quite happen, a slave letting go of SDA
// mid-byte. Window is given as microseconds before the trigger.
void command_la_raw(Console &c) {
    if (la_armed) la_stop("raw while armed");
    if (!la_have || !la_valid) { printf("no capture\n"); return; }

    float before = (float)c.packet.take_int().ok_or(200);
    float span   = (float)c.packet.take_int().ok_or(100);

    float sper_us = la_rate_actual / 1000000.0f;
    long start = (long)la_valid - (long)(before * sper_us);
    long count = (long)(span * sper_us);
    if (start < 0) { count += start; start = 0; }
    if (start + count > (long)la_valid) count = (long)la_valid - start;
    if (count <= 0) { printf("window outside capture\n"); return; }

    printf("raw %.1fus .. %.1fus, %ld samples, %.3f us/char\n",
           la_us((unsigned)start), la_us((unsigned)(start + count)), count, 1.0f / sper_us);

    const int W = 100;
    for (long off = 0; off < count; off += W) {
        long n = (count - off < W) ? (count - off) : W;
        char rs[W + 1], rd[W + 1];
        for (long k = 0; k < n; k++) {
            unsigned s = la_sample((unsigned)(start + off + k));
            rs[k] = (s & LA_BIT_SCL) ? '-' : '_';
            rd[k] = (s & LA_BIT_SDA) ? '-' : '_';
        }
        rs[n] = rd[n] = '\0';
        printf("%9.1f SCL %s\n", la_us((unsigned)(start + off)), rs);
        printf("%9s SDA %s\n\n", "", rd);
    }
}
