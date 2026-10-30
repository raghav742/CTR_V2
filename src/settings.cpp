#include "settings.h"
#include <EEPROM.h>
#include "serial_io.h"
#include "hw_profile.h"

namespace settings {
namespace {
  constexpr uint16_t MAGIC   = 0xC7A2;
  constexpr uint8_t  VERSION = 1;
  constexpr int      ADDR    = 0;

  struct Stored {
    uint16_t magic;
    uint8_t  version;
    uint8_t  calibValid;
    float    frontMax;
    float    gap[hw::NUM_HOMED_AXES - 1];
    uint8_t  parked;
    uint8_t  check;          // checksum over everything before it
  };
  Stored d;
  bool   loaded = false;     // EEPROM content was valid at startup

  uint8_t checksum(const Stored& s) {
    const uint8_t* p = (const uint8_t*)&s;
    uint8_t c = 0x5A;
    for (size_t i = 0; i < offsetof(Stored, check); i++) c = (uint8_t)((c << 1) | (c >> 7)) ^ p[i];
    return c;
  }

  void write() {
    d.magic = MAGIC; d.version = VERSION;
    d.check = checksum(d);
    EEPROM.put(ADDR, d);     // put() only writes bytes that changed
    loaded = true;
  }
}

void begin() {
  EEPROM.get(ADDR, d);
  loaded = d.magic == MAGIC && d.version == VERSION && d.check == checksum(d);
  if (!loaded) {             // empty or old/corrupt: start clean (written on first change)
    memset(&d, 0, sizeof(d));
    Out.println(F("EV settings none stored (using profile values)"));
  }
}

bool calibStored() { return d.calibValid; }

void calibValues(float& frontMax, float gap[]) {
  frontMax = d.frontMax;
  for (uint8_t k = 0; k + 1 < hw::NUM_HOMED_AXES; k++) gap[k] = d.gap[k];
}

void saveCalib(float frontMax, const float gap[]) {
  d.calibValid = 1;
  d.frontMax = frontMax;
  for (uint8_t k = 0; k + 1 < hw::NUM_HOMED_AXES; k++) d.gap[k] = gap[k];
  write();
}

void forgetCalib() { d.calibValid = 0; write(); }

bool parked() { return d.parked; }

void setParked(bool p) {
  if ((bool)d.parked == p) return;
  d.parked = p;
  write();
}

void report() {
  Out.print(F("OK settings eeprom=")); Out.print(loaded ? F("valid") : F("empty"));
  Out.print(F(" calib=")); Out.print(d.calibValid ? 1 : 0);
  if (d.calibValid) {
    Out.print(F(" front_max=")); Out.print(d.frontMax, 2);
    for (uint8_t k = 0; k + 1 < hw::NUM_HOMED_AXES; k++) { Out.print(F(" gap")); Out.print(k + 1); Out.print('='); Out.print(d.gap[k], 2); }
  }
  Out.print(F(" parked=")); Out.println(d.parked ? 1 : 0);
}
}  // namespace settings
