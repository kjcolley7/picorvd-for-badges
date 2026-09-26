#include "Application.h"
#include "boot_checkpoints.h"

Application *gApp;

void Application::init() {
    const int ch32v003_flash_size = 16 * 1024;

    printf(COLOR_GREEN("// Starting PicoSWIO") "\n");
    swio = new PicoSWIO();
    swio->init(4); // pin on pico connected to swio pin on ch32
    boot_checkpoint(BOOT_CP_SWIO);
    swio->reset();
    boot_checkpoint(BOOT_CP_SWIO_RESET);

    printf(COLOR_GREEN("// Starting RVDebug") "\n");
    rvd = new RVDebug(swio, 16);
    rvd->init();
    boot_checkpoint(BOOT_CP_RVDEBUG);

    printf(COLOR_GREEN("// Starting WCHFlash") "\n");
    flash = new WCHFlash(rvd, ch32v003_flash_size);
    flash->reset();
    boot_checkpoint(BOOT_CP_FLASH);
    //flash->dump();

    printf(COLOR_GREEN("// Starting SoftBreak") "\n");
    soft = new SoftBreak(rvd, flash);
    soft->init();
    boot_checkpoint(BOOT_CP_SOFTBREAK);
    //soft->dump();

    printf(COLOR_GREEN("// Starting GDBServer") "\n");
    gdb = new GDBServer(rvd, flash, soft);
    gdb->reset();
    boot_checkpoint(BOOT_CP_GDBSERVER);
    //gdb->dump();
}
