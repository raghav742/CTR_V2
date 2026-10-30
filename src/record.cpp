#include "record.h"
#include <EEPROM.h>
#include "serial_io.h"
#include "motion.h"

namespace record {
namespace {
  constexpr int      ADDR  = 64;          // after the settings record (settings.cpp, address 0)
  constexpr uint16_t MAGIC = 0x7E5C;
  struct Stored {
    uint16_t magic;
    uint8_t  n;
    int16_t  p[MAX_POSES][hw::NUM_AXES];
    uint8_t  check;
  };
  Stored d;

  uint8_t checksum() {
    const uint8_t* b = (const uint8_t*)&d;
    uint8_t c = 0xA5;
    for (size_t i = 0; i < offsetof(Stored, check); i++) c = (uint8_t)((c << 1) | (c >> 7)) ^ b[i];
    return c;
  }
  void save() { d.magic = MAGIC; d.check = checksum(); EEPROM.put(ADDR, d); }
}

void begin() {
  EEPROM.get(ADDR, d);
  if (d.magic != MAGIC || d.n > MAX_POSES || d.check != checksum()) d.n = 0;
}

uint8_t count() { return d.n; }
const int16_t* pose(uint8_t i) { return d.p[i]; }

bool add() {
  if (d.n >= MAX_POSES) { Out.println(F("ERR record full (10 poses)")); return false; }
  for (uint8_t a = 0; a < hw::NUM_AXES; a++) {
    float v = motion::position(a) * 10.0f;
    d.p[d.n][a] = (int16_t)constrain(lroundf(v), -32000L, 32000L);
  }
  d.n++;
  save();
  Out.print(F("EV rec pose ")); Out.print(d.n); Out.println(F(" saved"));
  return true;
}

void clear() { d.n = 0; save(); Out.println(F("EV rec cleared")); }

void report() {
  Out.print(F("OK rec poses=")); Out.print(d.n);
  for (uint8_t i = 0; i < d.n; i++) {
    Out.print(F(" ["));
    for (uint8_t a = 0; a < hw::NUM_AXES; a++) { if (a) Out.print(','); Out.print(d.p[i][a] / 10.0f, 1); }
    Out.print(']');
  }
  Out.println();
}
}  // namespace record
