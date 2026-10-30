#include "fastgfx.h"
#include "display.h"
#include <glcdfont.c>        // Adafruit's 5x7 font table (own copy: theirs is 'static')

namespace fastgfx {
namespace {
  constexpr uint8_t W = 128, PAGES = 8;
  uint8_t* buf() { return display::gfx().getBuffer(); }

  // OR (normal) or clear (inverse) one 8-bit column at (x, y)
  inline void column(uint8_t x, uint8_t y, uint8_t bits, bool inverse) {
    if (x >= W || y >= 64) return;
    uint8_t* b = buf();
    uint8_t page = y >> 3, sh = y & 7;
    uint16_t i = page * W + x;
    uint8_t lo = bits << sh;
    if (inverse) b[i] &= ~lo; else b[i] |= lo;
    if (sh && page + 1 < PAGES) {
      uint8_t hi = bits >> (8 - sh);
      if (inverse) b[i + W] &= ~hi; else b[i + W] |= hi;
    }
  }
}

void clear() { memset(buf(), 0, W * PAGES); }

void fill(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool on) {
  if (x1 >= W) x1 = W - 1;
  if (y1 > 63) y1 = 63;
  if (x0 > x1 || y0 > y1) return;
  uint8_t* b = buf();
  for (uint8_t page = y0 >> 3; page <= (y1 >> 3); page++) {
    // bits of this page that lie inside y0..y1
    uint8_t top = (page == (y0 >> 3)) ? (y0 & 7) : 0;
    uint8_t bot = (page == (y1 >> 3)) ? (y1 & 7) : 7;
    uint8_t mask = (uint8_t)((0xFF << top) & (0xFF >> (7 - bot)));
    uint8_t* row = b + page * W;
    for (uint8_t x = x0; x <= x1; x++) {
      if (on) row[x] |= mask; else row[x] &= ~mask;
    }
  }
}

uint8_t chr(uint8_t x, uint8_t y, char c, bool inverse) {
  const uint8_t* g = font + 5 * (uint8_t)c;
  for (uint8_t i = 0; i < 5; i++) column(x + i, y, pgm_read_byte(g + i), inverse);
  return x + CHAR_W;
}

uint8_t text(uint8_t x, uint8_t y, const char* s, bool inverse) {
  while (*s && x < W) x = chr(x, y, *s++, inverse);
  return x;
}

uint8_t text(uint8_t x, uint8_t y, const __FlashStringHelper* s, bool inverse) {
  const char* p = (const char*)s;
  char c;
  while ((c = pgm_read_byte(p++)) && x < W) x = chr(x, y, c, inverse);
  return x;
}

// Double size: every font pixel becomes 2x2. A glyph column (7 bits) is spread
// to 14 bits and written as two 8-pixel columns stacked (y and y + 8), twice.
uint8_t text2(uint8_t x, uint8_t y, const char* s) {
  while (*s && x < W) {
    const uint8_t* g = font + 5 * (uint8_t)*s++;
    for (uint8_t i = 0; i < 5; i++) {
      uint8_t b = pgm_read_byte(g + i);
      uint16_t w = 0;
      for (uint8_t k = 0; k < 8; k++) if (b & (1 << k)) w |= (uint16_t)3 << (2 * k);
      for (uint8_t r = 0; r < 2; r++) {
        column(x + 2 * i + r, y,     (uint8_t)(w & 0xFF), false);
        column(x + 2 * i + r, y + 8, (uint8_t)(w >> 8),   false);
      }
    }
    x += 2 * CHAR_W;
  }
  return x;
}

uint8_t num(uint8_t x, uint8_t y, float v, uint8_t width, uint8_t dec, bool inverse) {
  char b[14];
  dtostrf(v, width, dec, b);
  return text(x, y, b, inverse);
}
}  // namespace fastgfx
