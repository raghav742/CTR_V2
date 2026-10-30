#pragma once
// Firmware version: <phase>.<step>.<patch>
#define FW_VERSION "2.1.0"

// Serial protocol (docs/protocol.md). Bump PROTO_VERSION when a
// laptop program would need changing to keep working.
#define PROTO_VERSION 1
#define CONSOLE_BAUD  250000UL   // exact on a 16 MHz Mega (115200 is 2 % off)
