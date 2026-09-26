#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Persistent production log in the probe's own flash (see factory_log.cpp).
// One record per flashed board: UID, firmware note, outcome, timings. Survives
// power loss, so a standalone (PC-less) programmer keeps its history until the
// log is dumped and cleared at a bench.

// Call once from main() before the scheduler starts (creates the lock).
void factory_log_init(void);

// Append one record. Returns false if the log is full or the write failed.
// Successes always append -- every completed flash, including a reflash of
// the same badge, gets its own row. A failure whose UID matches an
// immediately preceding failure is skipped (and reported as success), so a
// badge that keeps failing in the retry loop makes one line, not many.
bool factory_log_append(const uint32_t uid[3], const char *fw_note,
                        bool ok, uint8_t attempts, uint32_t elapsed_ms);

// Number of used record slots (including any torn/corrupt ones).
int factory_log_count(void);
int factory_log_capacity(void);

// Print every record as a CSV line on the console.
void factory_log_dump(void);

// Header for, and one record as, a CSV row ending in a newline (no "CSV,"
// prefix). The row function returns its length, or 0 for a torn record.
#define FACTORY_LOG_CSV_HEADER "probe,seq,uid,firmware,status,attempts,ms,uptime_s\n"
int factory_log_csv_row(int i, char *buf, size_t len);

// Erase the whole log region. Returns false on erase failure.
bool factory_log_clear(void);
