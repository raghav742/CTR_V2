#pragma once
// ============================================================
//  Fast drawing straight into the SSD1306 frame buffer (step 3.1)
//  The display memory stores 8 vertical pixels per byte (one
//  "page" = 8 rows). A 5x7 character is 5 column bytes, so text
//  is written byte-wise (shifted when y is not a multiple of 8)
//  instead of pixel by pixel: ~50x faster than Adafruit's print.
//  Same font as Adafruit GFX (glcdfont, code page 437).
//  Use for everything redrawn often; Adafruit for one-off graphics.
// ============================================================
#include <Arduino.h>

namespace fastgfx {
constexpr uint8_t CHAR_W = 6;          // 5 pixels + 1 space
void clear();
// Rectangle x0..x1, y0..y1 (inclusive): white (on) or black (off)
void fill(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool on);
// Text at pixel position (x, y = top). inverse: black text, for use on a white fill.
// Returns the x after the text.
uint8_t text(uint8_t x, uint8_t y, const char* s, bool inverse = false);
uint8_t text(uint8_t x, uint8_t y, const __FlashStringHelper* s, bool inverse = false);
uint8_t chr(uint8_t x, uint8_t y, char c, bool inverse = false);
// Number right-aligned in 'width' characters with 'dec' decimals
uint8_t num(uint8_t x, uint8_t y, float v, uint8_t width, uint8_t dec, bool inverse = false);
}  // namespace fastgfx
