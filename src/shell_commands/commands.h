#pragma once

#include "shell/Console.h"
#include "Application.h"

void command_clocks(Console &c);

void command_dump(Console &c);

void command_dump2(Console &c);

void command_reset(Console &c);

void command_halt(Console &c);

void command_resume(Console &c);

void command_step(Console &c);

void command_status(Console &c);

void command_wipe(Console &c);

void command_init_swio(Console &c);

void command_part_id(Console &c);
void command_swio_test(Console &c);
void command_why(Console &c);

void command_halt_on_reset(Console &c);

void command_chip_id(Console &c);

// I2C master, for driving an attached SAO as if this were a host badge
void command_i2c_scan(Console &console);
void command_i2c_write(Console &console);
void command_i2c_read(Console &console);
void command_sao_cmd(Console &console);
void command_sao_wb(Console &console);
void command_sao_rb(Console &console);
void command_arp_udid(Console &console);
void command_arp_test(Console &console);
void command_arp_enum(Console &console);
void command_sao_discover(Console &console);
void command_sao_log(Console &console);
void command_sao_loop(Console &console);

// Two-channel logic analyzer on the I2C bus (see la_commands.cpp)
void command_la_arm(Console &console);
void command_la_stop(Console &console);
void command_la_dump(Console &console);
void command_la_raw(Console &console);


// SMBus Read Byte / Read Word with PEC and no console chatter, for
// programmatic polling (implemented in i2c_commands.cpp)
bool sao_read_byte_quiet(uint8_t addr, uint8_t cmd, uint8_t *val);
bool sao_read_word_quiet(uint8_t addr, uint8_t cmd, uint16_t *val);
bool sao_send_byte_quiet(uint8_t addr, uint8_t cmd);

// Production flashing loop (factory.cpp)
void command_factory(Console &console);
