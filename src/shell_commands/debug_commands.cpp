#include "commands.h"
#include "pico/time.h"
#include "debug_defines.h"

#ifndef DUMP_DEFAULT_ADDRESS
#define DUMP_DEFAULT_ADDRESS     (0x08000000)
#endif


static inline bool check() {
    if (!gApp->rvd) {
        printf(COLOR_RED("rvd is null") "\n");
        return false;
    }

    if (!gApp->swio) {
        printf(COLOR_RED("swio is null") "\n");
        return false;
    }

    return true;
}

// Prints the standard "the target is not there" message.
static void print_link_error() {
    printf(COLOR_RED("target not responding (SWIO reads 0x%08lX) - try halt_on_reset, or replug") "\n",
           gApp->swio->get_partid());
}

// True if the target answers at all. Commands use this to fail with a clear
// message instead of grinding through timeouts on a dead link.
static bool check_link() {
    if (gApp->swio->is_link_alive()) return true;

    print_link_error();
    return false;
}


void command_dump(Console &c) {
    if (!check()) return;
    if (!check_link()) return;

    auto addr = c.packet.take_int().ok_or(DUMP_DEFAULT_ADDRESS);
    printf("addr 0x%08x\n", addr);

    if (addr & 3) {
        printf("dump - bad addr 0x%08x\n", addr);
        return;
    }

    uint32_t buf[24 * 8];
    gApp->rvd->get_block_aligned(addr, buf, 24 * 8 * 4);
    for (int y = 0; y < 24; y++) {
        for (int x = 0; x < 8; x++) {
            printf("0x%08x ", buf[x + 8 * y]);
        }
        printf("\n");
    }
}

void command_dump2(Console &c) {
    if (!check()) return;
    if (!check_link()) return;

    auto addr = c.packet.take_int().ok_or(DUMP_DEFAULT_ADDRESS);
    printf("addr 0x%08x\n", addr);

    if (addr & 3) {
        printf("dump - bad addr 0x%08x\n", addr);
        return;
    }

    const unsigned int per_line = 8;

    // ------------------------
    // header
    printf("         | ");
    for (int x = 0; x < per_line; x++) {
        printf("%02X %02X %02X %02X | ", (x * 4 + 0), (x * 4 + 1), (x * 4 + 2), (x * 4 + 3));
    }
    printf("\n---------|-");
    for (int x = 0; x < per_line; x++) {
        printf("--------------");
    }
    printf("\n");

    // ------------------------
    // body
    uint32_t buf[24 * per_line];
    gApp->rvd->get_block_aligned(addr, buf, sizeof(buf));
    for (int y = 0; y < 24; y++) {
        printf("%08x | ", addr + y * per_line * sizeof(buf[0]));
        for (int x = 0; x < per_line; x++) {
            uint32_t n = buf[x + per_line * y];
            printf(
                    "%02X %02X %02X %02X | ",
                    ((n >> 0) & 0xFF),
                    ((n >> 8) & 0xFF),
                    ((n >> 16) & 0xFF),
                    ((n >> 24) & 0xFF)
            );
        }
        printf("\n");
    }
}

void command_reset(Console &c) {
    if (!check()) return;
    if (!check_link()) return;

    if (gApp->rvd->reset()) {
        printf(COLOR_GREEN("Reset OK") "\n");
    } else {
        printf(COLOR_RED("Reset failed") "\n");
        if (!gApp->swio->is_link_alive()) print_link_error();
    }
}

void command_halt(Console &c) {
    if (!check()) return;
    if (!check_link()) return;

    if (gApp->rvd->halt()) {
        printf(COLOR_GREEN("Halted at DPC = 0x%08lx") "\n",  gApp->rvd->get_dpc());
    } else {
        printf(COLOR_RED("Halt failed") "\n");
        if (!gApp->swio->is_link_alive()) print_link_error();
    }
}

void command_resume(Console &c) {
    if (!check()) return;
    if (!check_link()) return;

    if (gApp->rvd->resume()) {
        printf(COLOR_GREEN("Resume OK") "\n");
    } else {
        printf(COLOR_RED("Resume failed") "\n");
        if (!gApp->swio->is_link_alive()) print_link_error();
    }
}

void command_step(Console &c) {
    if (!check()) return;
    if (!check_link()) return;

    int count = c.packet.take_int().ok_or(1);
    if (count < 1) {
        printf(COLOR_RED("Invalid step count") "\n");
        return;
    }

    for (int i = 0; i < count; i++) {
        if (gApp->rvd->step()) {
            printf(COLOR_GREEN("%d. Stepped to DPC = 0x%08lx") "\n", i,  gApp->rvd->get_dpc());
        } else {
            printf(COLOR_RED("Step failed") "\n");
            if (!gApp->swio->is_link_alive()) print_link_error();
            break;
        }
    }

}

void command_status(Console &c) {
    if (!check()) return;

    // Still dump on a dead link - the register values are useful for diagnosis -
    // but say up front that they can't be trusted.
    if (!gApp->swio->is_link_alive()) {
        print_link_error();
        printf(COLOR_YELLOW("(the register values below are not real)") "\n");
    }

    gApp->rvd->dump();
}

void command_init_swio(Console &c) {
    if (!check()) return;

    gApp->swio->reset();

    if (gApp->swio->is_link_alive()) {
        printf(COLOR_GREEN("SWIO init OK") "\n");
    } else {
        print_link_error();
    }
}

// Loopback test: DM_DATA0 is plain read/write scratch in the debug module, so
// writing a pattern and reading it back exercises the write path end to end.
// Reads can look perfect while writes are being dropped, and the two failure
// modes need telling apart before anything else makes sense.
void command_swio_test(Console &c) {
    if (!check()) return;

    static const uint32_t pats[] = {
        0x00000001, 0x80000000, 0x80000001, 0xFFFFFFFF,
        0x5A5A5A5A, 0xA5A5A5A5, 0x00000003, 0xDEADBEEF,
    };
    int bad = 0;
    for (unsigned i = 0; i < count_of(pats); i++) {
        gApp->swio->put(DM_DATA0, pats[i]);
        uint32_t rd = gApp->swio->get(DM_DATA0);
        if (rd != pats[i]) {
            bad++;
            printf(COLOR_RED("  wrote 0x%08lX read 0x%08lX") "\n",
                   (unsigned long) pats[i], (unsigned long) rd);
        } else {
            printf("  wrote 0x%08lX ok\n", (unsigned long) pats[i]);
        }
    }

    int bitbad = 0;
    for (int b = 0; b < 32; b++) {
        uint32_t v = 1u << b;
        gApp->swio->put(DM_DATA0, 0);
        gApp->swio->put(DM_DATA0, v);
        if (gApp->swio->get(DM_DATA0) != v) {
            bitbad++;
            printf(COLOR_RED("  bit %d failed") "\n", b);
        }
    }
    printf(bad || bitbad ? COLOR_RED("swio_test: %d pattern, %d bit failures") "\n"
                         : COLOR_GREEN("swio_test: %d pattern, %d bit failures") "\n",
           bad, bitbad);
}

// Watch DMSTATUS while trying to take the hart, and say which step fails.
// "Halt failed" on its own cannot distinguish a hart that ignores HALTREQ from
// one that keeps resetting out from under it.
void command_why(Console &c) {
    if (!check()) return;

    uint32_t st = gApp->swio->get(DM_DMSTATUS);
    printf("dmstatus       = 0x%08lX  halted=%d running=%d havereset=%d unavail=%d\n",
           (unsigned long) st, (int) ((st >> 9) & 1), (int) ((st >> 11) & 1),
           (int) ((st >> 19) & 1), (int) ((st >> 13) & 1));

    // Is the hart resetting over and over? Ack HAVERESET and see if it comes
    // back on its own - that is what a watchdog or a boot loop looks like.
    gApp->swio->put(DM_DMCONTROL, 0x10000001);
    st = gApp->swio->get(DM_DMSTATUS);
    printf("after ack      = 0x%08lX  havereset=%d\n",
           (unsigned long) st, (int) ((st >> 19) & 1));

    int reasserts = 0;
    for (int i = 0; i < 40; i++) {
        busy_wait_us(2000);
        if (gApp->swio->get(DM_DMSTATUS) & (1u << 19)) {
            reasserts++;
            gApp->swio->put(DM_DMCONTROL, 0x10000001);
        }
    }
    printf("havereset re-asserted %d/40 times over ~80ms%s\n", reasserts,
           reasserts ? "  <-- the hart is resetting repeatedly" : "");

    // Now try to halt, reporting how DMSTATUS moves.
    gApp->swio->put(DM_DMCONTROL, 0x80000001);
    for (int i = 0; i < 20; i++) {
        st = gApp->swio->get(DM_DMSTATUS);
        if (st & (1u << 9)) break;
        busy_wait_us(1000);
    }
    printf("after haltreq  = 0x%08lX  halted=%d running=%d havereset=%d\n",
           (unsigned long) st, (int) ((st >> 9) & 1), (int) ((st >> 11) & 1),
           (int) ((st >> 19) & 1));
    gApp->swio->put(DM_DMCONTROL, 0x00000001);
}

void command_part_id(Console &c) {
    if (!check()) return;

    uint32_t part_id = gApp->swio->get_partid();
    printf(COLOR_GREEN("DM_PARTID = 0x%08lX") "\n",  part_id);
}

// When swio pin stay HIGH or LOW all reads returns only 1 or 0
// part_id should be not 0xffffffff of 0x00000000, so we can use it to identify when swio is actually in debug mode
static bool check_part_id() {
    return gApp->swio->is_link_alive();
}

void command_halt_on_reset(Console &c) {
    if (!check()) return;

    // No check_link() here - reviving a dead link is exactly what this command
    // is for.

    printf("Trying enter debug mode in normal way... ");
    gApp->rvd->halt();
    if (!check_part_id()) {
        gApp->swio->reset();
        gApp->rvd->halt();
    }

    if (check_part_id()) {
        printf(COLOR_GREEN("success") "\n");
        return;
    }

    printf(COLOR_RED("fail") "\n");
    printf("\n" COLOR_WHITE("Please toggle chips VCC pin in next 10 seconds...") "\n");
    auto start_time = get_absolute_time();
    static const char progress_array[] = {'-', '\\', '|', '/'};
    printf(" ");

    for (int i = 0;;) {
        printf("\b%c", progress_array[i]);
        if (++i >= count_of(progress_array)) i = 0;

        gApp->swio->reset();
        gApp->rvd->halt();

        if (check_part_id()) {
            sleep_us(1000);
            if (check_part_id()) {
                printf("\b" COLOR_GREEN("Successfully halted") "\n");
                break;
            }
        }

        if (absolute_time_diff_us(start_time, get_absolute_time()) > 10'000'000) {
            printf("\b" COLOR_RED("Halt failed") "\n");
            break;
        }
    }
}

void command_chip_id(Console &c) {
    if (!check()) return;
    if (!check_link()) return;

    static const struct {
        uint32_t part_id;
        const char *name;
    } part_id_map[] = {
            {0x00300500, "CH32V003F4P6"},
            {0x00310500, "CH32V003F4U6"},
            {0x00320500, "CH32V003A4M6"},
            {0x00330500, "CH32V003J4M6"},
    };

    struct {
        uint32_t unk_01;
        uint32_t part_id;
        uint32_t unk_02;
        uint32_t unk_03;
        uint32_t unk_04;
        uint32_t unk_05;
        uint32_t unk_06;
        uint32_t unk_07;
        uint16_t R16_ESIG_FLACAP;
        uint16_t pad_01;
        uint32_t pad_02;
        uint32_t R32_ESIG_UNIID1;
        uint32_t R32_ESIG_UNIID2;
        uint32_t R32_ESIG_UNIID3;
    } buf;

    gApp->rvd->get_block_aligned(0x1FFFF7C0, &buf, sizeof(buf));

    const char *part_id_name = "Unknown";
    auto part_id_fact = buf.part_id & 0xFFFFFF0F;
    for (auto [part_id_cmp, name]: part_id_map) {
        if (part_id_fact == part_id_cmp) {
            part_id_name = name;
        }
    }

    printf("Chip: " "\u001b[%sm" "%s" "\u001b[0m" "\n", "1;37", part_id_name);
    printf("Part ID: %08lX\n", buf.part_id);
    printf("Flash Capacity: %d Kb\n", buf.R16_ESIG_FLACAP);
    printf("Unique ID: %08lX%08lX%08lX\n", buf.R32_ESIG_UNIID1, buf.R32_ESIG_UNIID2, buf.R32_ESIG_UNIID3);
}
