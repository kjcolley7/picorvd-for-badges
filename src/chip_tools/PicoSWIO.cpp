#include "PicoSWIO.h"
#include "singlewire.pio.h"
#include "debug_defines.h"

#include "utils.h"

#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/uart.h"

//#define DUMP_COMMANDS

__attribute__((noinline)) void busy_wait(int count) {
    volatile int c = count;
    while (c) c = c - 1;
}

// WCH-specific debug interface config registers
static const int WCH_DM_CPBR = 0x7C;
static const int WCH_DM_CFGR = 0x7D;
static const int WCH_DM_SHDWCFGR = 0x7E;
static const int WCH_DM_PART = 0x7F; // not in doc but appears to be part info

//------------------------------------------------------------------------------

void PicoSWIO::init(int pin) {
    CHECK(pin != -1);
    this->pin = pin;

    // Configure GPIO
    gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_slew_rate(pin, GPIO_SLEW_RATE_SLOW);
    gpio_set_function(pin, GPIO_FUNC_PIO0);

    // Reset PIO module
    pio0->ctrl = 0b000100010001;
    pio_sm_set_enabled(pio0, pio_sm, false);

    // Upload PIO program
    pio_clear_instruction_memory(pio0);
    pio_offset = pio_add_program(pio0, &singlewire_program);

    // Configure PIO module
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pio_offset + singlewire_wrap_target, pio_offset + singlewire_wrap);
    sm_config_set_sideset(&c, 1, /*optional*/ false, /*pindirs*/ true);
    sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_set_pins(&c, pin, 1);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, /*shift_right*/ false, /*autopull*/ false, /*pull_threshold*/ 32);
    sm_config_set_in_shift(&c, /*shift_right*/ false, /*autopush*/ true,  /*push_threshold*/ 32);

    // 125 mhz / 12 = 96 nanoseconds per tick, close enough to 100 ns.
    sm_config_set_clkdiv(&c, 12);

    pio_sm_init(pio0, pio_sm, pio_offset, &c);
    pio_sm_set_pins(pio0, pio_sm, 0);
    pio_sm_set_enabled(pio0, pio_sm, true);
}

void PicoSWIO::reset() {
    // Grab pin and send an 8 usec low pulse to reset debug module
    // If we use the sdk functions to do this we get jitter :/
    sio_hw->gpio_clr = (1 << pin);
    sio_hw->gpio_oe_set = (1 << pin);
    io_bank0_hw->io[pin].ctrl = GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    busy_wait(100); // ~8 usec
    sio_hw->gpio_oe_clr = (1 << pin);
    io_bank0_hw->io[pin].ctrl = GPIO_FUNC_PIO0 << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;

    // Enable debug output pin on target
    put(WCH_DM_SHDWCFGR, 0x5AA50400);
    put(WCH_DM_CFGR, 0x5AA50400);

    // Reset debug module on target
    put(DM_DMCONTROL, 0x00000000);
    put(DM_DMCONTROL, 0x00000001);
}

//------------------------------------------------------------------------------

// A whole SWIO transaction is ~10us; a millisecond of no progress means the
// state machine is stalled, not slow. That happens in the field: a hot-plug
// contact bounce mid-transaction can park the singlewire program at a `wait`
// for a line edge that never arrives, and the pio_sm_*_blocking calls then
// spin forever -- with put() doing so with interrupts off, which takes down
// the whole probe (USB, watchdog task and all). Every FIFO wait is therefore
// bounded, and a timeout rebuilds the state machine and reports the read as
// dead (0xFFFFFFFF), which every caller already handles as "no target".
#define SWIO_FIFO_TIMEOUT_US 1000u

void PicoSWIO::recover() {
    pio_sm_set_enabled(pio0, pio_sm, false);
    pio_sm_clear_fifos(pio0, pio_sm);
    pio_sm_restart(pio0, pio_sm);
    pio_sm_exec(pio0, pio_sm, pio_encode_jmp(pio_offset));
    pio_sm_set_enabled(pio0, pio_sm, true);
}

static bool wait_tx_room(int sm, uint32_t words, uint32_t timeout_us) {
    uint32_t start = time_us_32();
    while (pio_sm_get_tx_fifo_level(pio0, sm) > 4 - words) {
        if (time_us_32() - start > timeout_us) return false;
        tight_loop_contents();
    }
    return true;
}

uint32_t PicoSWIO::get(uint32_t addr) {
    cmd_count++;

    if (!wait_tx_room(pio_sm, 1, SWIO_FIFO_TIMEOUT_US)) {
        recover();
        return 0xFFFFFFFF;
    }
    pio_sm_put(pio0, pio_sm, ((~addr) << 1) | 1);

    uint32_t start = time_us_32();
    while (pio_sm_is_rx_fifo_empty(pio0, pio_sm)) {
        if (time_us_32() - start > SWIO_FIFO_TIMEOUT_US) {
            recover();
            return 0xFFFFFFFF;
        }
        tight_loop_contents();
    }
    auto data = pio_sm_get(pio0, pio_sm);
#ifdef DUMP_COMMANDS
    printf("get_dbg %15s 0x%08x\n", addr_to_regname(addr), data);
#endif
    return data;
}

//------------------------------------------------------------------------------

void PicoSWIO::put(uint32_t addr, uint32_t data) {
    cmd_count++;
#ifdef DUMP_COMMANDS
    printf("set_dbg %15s 0x%08x\n", addr_to_regname(addr), data);
#endif
    // The two words have to reach the FIFO back to back.
    //
    // The PIO program blocks at the `pull` between the address phase and the
    // data phase, holding the line released (high) while it waits. The target
    // ends a frame after ~2.25us of idle, so if anything delays the second push
    // past that - a FreeRTOS context switch, an interrupt, a USB callback - the
    // target has already given up, and it then resynchronizes on the first data
    // bit. A first bit that is a short low pulse is exactly what a start bit
    // looks like, and that is the case precisely when bit 31 of the data is set
    // (the wire carries ~data, so a 1 goes out as the short pulse).
    //
    // So every write with bit 31 set is silently swallowed and everything else
    // gets through. Of what this probe sends that is DMCONTROL's HALTREQ
    // (0x80000001) and NDMRESET (0x80000003) and nothing else - which shows up
    // as a target that reads back perfectly, reports itself running, and just
    // ignores halt and reset, while SWIO init and every register read work
    // because those values all have bit 31 clear.
    //
    // Wait for room for BOTH words BEFORE masking interrupts, so the critical
    // section is exactly two non-blocking stores: it can neither delay the
    // second word (the resync hazard above) nor spin with interrupts off on a
    // stalled state machine (which would take the whole probe down).
    if (!wait_tx_room(pio_sm, 2, SWIO_FIFO_TIMEOUT_US)) {
        recover();
        return;   // dropped write == what a dead link looks like; readers bail
    }

    uint32_t irq = save_and_disable_interrupts();
    pio_sm_put(pio0, pio_sm, ((~addr) << 1) | 0);
    pio_sm_put(pio0, pio_sm, ~data);
    restore_interrupts(irq);
}

//------------------------------------------------------------------------------

Reg_CPBR PicoSWIO::get_cpbr() {
    return get(WCH_DM_CPBR);
}

//------------------------------------------------------------------------------

Reg_CFGR PicoSWIO::get_cfgr() {
    return get(WCH_DM_CFGR);
}

//------------------------------------------------------------------------------

Reg_SHDWCFGR PicoSWIO::get_shdwcfgr() {
    return get(WCH_DM_SHDWCFGR);
}

//------------------------------------------------------------------------------

uint32_t PicoSWIO::get_partid() {
    return get(WCH_DM_PART);
}

//------------------------------------------------------------------------------
// When the swio pin stays HIGH or LOW (target unpowered, disconnected, or not
// in debug mode) every read comes back as all-ones or all-zeroes. The part ID
// is never 0x00000000 or 0xFFFFFFFF on a live target, so it's a cheap way to
// tell whether talking to the debug module is worth attempting at all.

bool PicoSWIO::is_link_alive() {
    uint32_t part_id = get_partid();
    if (part_id == 0 || part_id == 0xFFFFFFFF) {
        return false;
    }

    // In theory it's possible to catch the moment when the pin is switching and
    // part_id looks like 0xFFFF0000, so double check the part id to be sure.
    return part_id == get_partid();
}

//------------------------------------------------------------------------------

bool PicoSWIO::send_uart_byte(uint8_t byte, uint32_t baud) {
    // On the RP2040 only GPIO 0, 4, 8, ... 28 carry a UART TX function, and
    // which UART alternates: GP0 UART0, GP4/GP8 UART1, GP12/GP16 UART0, ...
    if (pin < 0 || (pin & 3) != 0) {
        return false;
    }
    uart_inst_t *uart = uart_get_instance(((pin >> 2) ^ (pin >> 3)) & 1);

    // UART up first, so its TX is already idling high when it takes the pin:
    // the line sits high under the probe's 1k pull-up either way, and the
    // target sees no stray edge at the switch-over. Stiffer edges than the
    // SWIO engine's gentle 2mA/slow config, since this is push-pull.
    uart_init(uart, baud);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
    gpio_set_function(pin, GPIO_FUNC_UART);

    uart_putc_raw(uart, (char)byte);
    uart_tx_wait_blocking(uart);    // returns once the stop bit is out

    // Pin back to the SWIO engine before the UART is shut down, so the reset
    // UART never gets to drive it. The state machine idles with the pin
    // released, so the line just stays high on the pull-up.
    gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_slew_rate(pin, GPIO_SLEW_RATE_SLOW);
    gpio_set_function(pin, GPIO_FUNC_PIO0);
    uart_deinit(uart);
    return true;
}

//------------------------------------------------------------------------------

void PicoSWIO::dump() {
    get_cpbr().dump();
    get_cfgr().dump();
    get_shdwcfgr().dump();
    printf("DM_PARTID = 0x%08x\n", get_partid());
}

//------------------------------------------------------------------------------

const char *PicoSWIO::addr_to_regname(uint8_t addr) {
    switch (addr) {
        case WCH_DM_CPBR:
            return "WCH_DM_CPBR";
        case WCH_DM_CFGR:
            return "WCH_DM_CFGR";
        case WCH_DM_SHDWCFGR:
            return "WCH_DM_SHDWCFGR";
        case WCH_DM_PART:
            return "WCH_DM_PART";

        case DM_DATA0:
            return "DM_DATA0";
        case DM_DATA1:
            return "DM_DATA1";
        case DM_DMCONTROL:
            return "DM_DMCONTROL";
        case DM_DMSTATUS:
            return "DM_DMSTATUS";
        case DM_HARTINFO:
            return "DM_HARTINFO";
        case DM_ABSTRACTCS:
            return "DM_ABSTRACTCS";
        case DM_COMMAND:
            return "DM_COMMAND";
        case DM_ABSTRACTAUTO:
            return "DM_ABSTRACTAUTO";
        case DM_PROGBUF0:
            return "DM_PROGBUF0";
        case DM_PROGBUF1:
            return "DM_PROGBUF1";
        case DM_PROGBUF2:
            return "DM_PROGBUF2";
        case DM_PROGBUF3:
            return "DM_PROGBUF3";
        case DM_PROGBUF4:
            return "DM_PROGBUF4";
        case DM_PROGBUF5:
            return "DM_PROGBUF5";
        case DM_PROGBUF6:
            return "DM_PROGBUF6";
        case DM_PROGBUF7:
            return "DM_PROGBUF7";
        case DM_HALTSUM0:
            return "DM_HALTSUM0";
        default:
            return "???";
    }
}

//------------------------------------------------------------------------------

void Reg_CPBR::dump() {
    printf("DM_CPBR = 0x%08x\n", raw);
    printf("  TDIV:%d  SOPN:%d  CHECKSTA:%d  CMDEXTENSTA:%d  OUTSTA:%d  IOMODE:%d  VERSION:%d\n",
           TDIV, SOPN, CHECKSTA, CMDEXTENSTA, OUTSTA, IOMODE, VERSION);
}

void Reg_CFGR::dump() {
    printf("DM_CFGR = 0x%08x\n", raw);
    printf("  TDIVCFG:%d  SOPNCFG:%d  CHECKEN:%d  CMDEXTEN:%d  OUTEN:%d  IOMODECFG:%d  KEY:0x%04x\n",
           TDIVCFG, SOPNCFG, CHECKEN, CMDEXTEN, OUTEN, IOMODECFG, KEY);
}

void Reg_SHDWCFGR::dump() {
    printf("DM_SHDWCFGR = 0x%08x\n", raw);
    printf("  TDIVCFG:%d  SOPNCFG:%d  CHECKEN:%d  CMDEXTEN:%d  OUTEN:%d  IOMODECFG:%d  KEY:0x%04x\n",
           TDIVCFG, SOPNCFG, CHECKEN, CMDEXTEN, OUTEN, IOMODECFG, KEY);
}
