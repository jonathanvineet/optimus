// Types shared by function signatures in autobot_mapping.ino. They live
// here because the Arduino builder inserts its generated function
// prototypes above any struct defined in the .ino itself.
#pragma once
#include <stdint.h>

// One command from a network client, applied by loop().
enum CmdType : uint8_t {
  CMD_DRIVE, CMD_STOP, CMD_MODE, CMD_SPEED, CMD_CAL,
  CMD_WS_OPEN, CMD_WS_CLOSE
};

struct Cmd {
  uint8_t type;
  uint8_t ip;        // last octet of the client's address
  int16_t a;
  int16_t b;
  int fd;
  uint32_t seq;
  const char* tag;   // string literal
};
