#include "commands.h"


static inline bool check() {
    if (!gApp->rvd) {
        printf(COLOR_RED("rvd is null") "\n");
        return false;
    }

    if (!gApp->flash) {
        printf(COLOR_RED("flash is null") "\n");
        return false;
    }

    if (!gApp->swio) {
        printf(COLOR_RED("swio is null") "\n");
        return false;
    }

    if (!gApp->swio->is_link_alive()) {
        printf(COLOR_RED("target not responding (SWIO reads 0x%08lX) - try halt_on_reset, or replug") "\n",
               gApp->swio->get_partid());
        return false;
    }

    return true;
}

void command_wipe(Console &c) {
    if (!check()) return;

    if (gApp->flash->wipe_chip()) {
        printf(COLOR_GREEN("Wipe OK") "\n");
    } else {
        printf(COLOR_RED("Wipe failed") "\n");
    }
}
