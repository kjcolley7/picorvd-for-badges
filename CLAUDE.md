# picorvd-freertos — working notes for Claude

A Raspberry Pi Pico (RP2040, FreeRTOS SMP) acting as a CH32V003 SWIO
debugger, GDB server and SAOv3 I2C host. Its production role is the
`factory` loop that flashes CH32V003-based badges and SAOs. This file is the
stuff that is easy to get wrong.

## Build and flash the probe

- `cmake -B build -G Ninja && ninja -C build pico_rvd` (PICO_BOARD defaults to
  `pico`; the tethered probe is an original Pi Pico). `pico_rvd_factory` is
  the standalone pass-around programmer; the RP2040-Zero build lives in
  `build-zero` with `-DPICO_BOARD=waveshare_rp2040_zero`.
- Flash without touching BOOTSEL: open the console port at 1200 baud, then
  `~/.local/bin/picotool load -x build/pico_rvd.uf2`. Use that picotool; the
  brew and pico-sdk copies segfault. `picotool --force` does not work on this
  firmware (custom USB descriptors, no reset interface).
- A reflash keeps the factory image and the factory flash log intact
  (layout in `flash_layout.h`: log in two segments, 2048 records, with the
  image between them).

## RAM is tight

- The C heap has only ~12 KB free after boot (the shell prints both heaps on
  connect). A large static buffer starves it, the GDB server's allocations at
  startup then fail, the probe hangs before USB comes up and the boot
  failsafe drops it into BOOTSEL. The next successful boot reports the
  checkpoint it died at (`boot_checkpoints.h`).
- Task stacks come from the FreeRTOS heap (~10 KB free); the USB task needs
  4x the minimal stack because it builds the PICORVD drive and formats
  LOG.CSV rows.

## Console

- Two CDC ports enumerate; the lower-numbered one is the console, the higher
  is GDB (on macOS e.g. `/dev/cu.usbmodem2101` / `2103`; names vary by port).
- Only one process may hold the console tty. A second opener (even a shell
  `printf > /dev/cu...`) hangs up the line and kills the first reader. Drive
  the console from a single process and feed it commands through a file.
- Argument syntax: `take_hex` arguments are bare hex with no `0x`
  (`sao_rb 10 85`, not `sao_rb 0x10 0x85`); `i2c_w` data bytes are decimal;
  `dump` wants its address without a leading space quirk, use `chip_id`
  instead for the signature area.
- Output is discarded whenever no host holds the console open (DTR), so a
  factory run without a terminal attached loses the per-board lines; the
  flash log keeps the record.
- `help` lists commands. There is no task-list command; `factory where` is
  the instrumentation for the factory loop.

## Factory mode

- `factory` starts the loop immediately. `factory stop`, `factory image`
  (what is stored), `factory log` (CSV dump), `factory clear`, `factory where`
  (tick stamps of each post-flash step plus the LED task's beat counter).
- The tethered build does NOT auto-start factory mode after a reboot; the
  standalone build does.
- The image and its settings live in the probe's flash just below the log
  (`factory_image.cpp`) and are uploaded over the PICORVD USB drive
  (`usb/msc_disk.cpp`): copy a `.bin` on, or edit `CONFIG.TXT`, and the probe
  stores it and restarts (reboot stamp "factory image updated"). The drive's
  writable part is 32 KB of RAM; CURRENT.BIN and LOG.CSV are generated on
  read, and their unused clusters are marked bad so the host can only
  allocate in RAM. An upload must land in contiguous clusters.
- `flash_safe_execute()` spawns a lockout task per call that is reaped only
  when the idle task runs; back-to-back calls must yield (`vTaskDelay`) or the
  FreeRTOS heap runs out (PICO_ERROR_INSUFFICIENT_RESOURCES, -9).
- `uart_selftest` in CONFIG.TXT (standalone build only): after booting the
  fresh firmware, the probe sends that byte out of the SWIO pin as 8N1 UART
  at `uart_baud` (GP4 is UART1 TX) every 100 ms for 1.5 s, returning the pin
  to SWIO between bytes (a single byte can be missed while the target boots).
  Useful for targets whose SWIO pin doubles as a console RX. `none` sends
  nothing.
- Flashing never erases the target's last 64-byte page (0x3FC0), where
  CH32V003 firmware commonly keeps its settings: whole sectors up to the last sector, then page
  erases only as far as the image reaches. An image that would need that
  page (over 16320 bytes) is refused.
- Confirm sequence (tethered) after a verified flash: SAOv3 scan, identity
  check against CONFIG.TXT `vid`/`pid`, the `selftest` command, then the LED-class paint (white, 64/channel) if the
  device exposes the standard LED class. Removal is watched over SWIO.
  Firmware that leaves SWIO alive reads as present until physically
  unplugged, and re-plugging a programmed board re-flashes it.
- LED semantics (plain LED on the Pico): fast blink = flashing (and booting/
  confirming); on 90 percent of the time = last board passed, holds until the
  next board is detected; double blip = failed; short blip = waiting;
  slow 1 s on/off = no image stored. A
  *completely* solid or dark LED means the LED task is not running: a starved
  or wedged probe.
- WS2812 boards (RP2040-Zero standalone programmer), all dim: blue =
  waiting for a target; fast-blinking red = flashing; orange = flashed and
  verified, booting it and sending the UART self-test byte; green = passed;
  red double blip = failed; magenta = no image stored. The steady colors pulse at 0.5 Hz with 80
  percent duty, so the frozen-LED wedge tell works here too.
- Chip UID: CH32V003 reads `D7xx ABCD 41yy BE72 FFFFFFFF` style; the third
  word is always FFFFFFFF and the `ABCD`/`BE7x` halves are fixed fields, so
  the usable unique part is the upper 16 bits of each of the first two words.
  UIDs must be read with `RVDebug::get_mem_u32_sync`; the fast read returns
  the previous word on blank chips.
- Collect logs by copying LOG.CSV off the drive, or with `tools/collect_factory_logs.py <console> --master
  factory_master.csv --clear`; probes have no RTC, so collection time is
  stamped on the host.

## Scheduling rules (learned the hard way)

- Every task loop must block somewhere (`vTaskDelay` or a blocking call).
  A yield-free loop at priority 2 permanently owns one of the two cores.
- `flash_safe_execute` (pico_flash, FreeRTOS SMP helper) parks a max-priority
  task on the other core with interrupts off until the writer finishes. With
  a spinner at priority 2 that starved priority 1 (factory loop, status LED)
  for minutes while the shell answered normally. Fixed by making the GDB task
  yield when idle; guarded by a priority-1 canary in `vTaskWatchdog` (30 s)
  and a runtime hardware watchdog (8 s) fed by that task.
- Deliberate reboots are stamped (`stamped_reboot`, kinds in
  `boot_checkpoints.h`) and the shell prints the cause on the next connect.
  A picotool reflash is not reported as a crash.
- `i2c_scan` typed with no device attached once wedged the shell for 30 s and
  tripped the watchdog. Cause not established; treat it as suspect.

## SAOv3 over the shell

- `sao_discover` runs ARP and assigns addresses (ARP-capable SAOs are
  assigned from 0x10 up). Use the discovered address afterwards.
- Class region starts at 0x20; the LED class base is discovered via common
  command 0x0B (id, base pairs). PEC is CRC-8 poly 0x07 over addr<<1, cmd, count, data. Blocks are capped at
  32 bytes on the CH32V003 device port.
- The probe (host side) carries the I2C pull-ups; SAOs have none by spec.

## Related code

- SAOv3 host library used by the probe: `SAOH_CORE_DIR` in CMakeLists.txt.
