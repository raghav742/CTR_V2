#include "serial_io.h"

SerialOut Out;

size_t SerialOut::write(uint8_t c) {
  if (atLineStart && tag >= 0 && c != '\r' && c != '\n') {
    Serial.write('#'); Serial.print(tag); Serial.write(' ');
  }
  atLineStart = (c == '\n');
  return Serial.write(c);
}

void SerialOut::beginReply(int32_t id) { tag = id; }
void SerialOut::endReply()             { tag = -1; }

// Decimal number as text without float maths per digit:
// scale to an integer once (e.g. 12.345 -> 1235 for 2 digits),
// then split into whole part and fraction. Sent in one write.
size_t SerialOut::print(double v, int digits) {
  static const uint16_t SCALE[] = { 1, 10, 100, 1000, 10000 };
  if (digits < 0) digits = 0;
  if (digits > 4) digits = 4;
  if (isnan(v) || isinf(v) || fabs(v) * SCALE[digits] > 4.0e9)
    return Print::print(v, digits);                 // rare: use the slow original
  char buf[16];
  uint8_t n = 0;
  if (v < 0) { buf[n++] = '-'; v = -v; }
  uint32_t scaled = (uint32_t)(v * SCALE[digits] + 0.5);
  if (scaled == 0 && n) n = 0;                      // no "-0.00"
  uint16_t frac;
  if (scaled <= 0xFFFF) {                           // common case: 16-bit maths, ~4x faster
    uint16_t s16 = scaled;
    utoa(s16 / SCALE[digits], buf + n, 10);
    frac = s16 % SCALE[digits];
  } else {
    ultoa(scaled / SCALE[digits], buf + n, 10);
    frac = scaled % SCALE[digits];
  }
  n = strlen(buf);
  if (digits) {
    buf[n++] = '.';
    for (int8_t d = digits - 1; d >= 0; d--) { buf[n + d] = '0' + frac % 10; frac /= 10; }
    n += digits;
  }
  return write((const uint8_t*)buf, n);
}

size_t SerialOut::println(double v, int digits) {
  size_t r = print(v, digits);
  return r + println();
}
