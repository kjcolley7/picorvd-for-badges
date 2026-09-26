/**
 * @file saoh_log_capture.h
 * @brief Force-included into saoh_core so its logging lands in RAM, not on USB
 *
 * saoh_core's log macros call printf directly with no override hook. Writing
 * those to the USB console costs milliseconds per line, which is enough to
 * change the very bus timing being investigated -- turning logging on made an
 * intermittent discovery failure disappear entirely. Capturing into a ring
 * buffer instead costs a vsnprintf and no I/O, so the run behaves the same
 * whether or not anyone is looking.
 */
#ifndef SAOH_LOG_CAPTURE_H
#define SAOH_LOG_CAPTURE_H

int saoh_log_capture(const char *fmt, ...);

#define printf saoh_log_capture

#endif
