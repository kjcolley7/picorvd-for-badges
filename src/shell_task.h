#pragma once

#include <stdint.h>

void vTaskShell(void *pvParams);

// Bumped every time the shell task goes round one of its polling loops. The
// watchdog task in main.cpp watches this to tell "busy in a long command" from
// "wedged": a command that never returns stops the counter forever.
extern volatile uint32_t g_shell_heartbeat;
