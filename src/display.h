#pragma once
// ============================================================
//  Display (step 3.1; V2 TFT prepared in step 5.2, UNTESTED)
//  All screens are drawn into a 128x64 monochrome frame (the
//  V1 OLED format). V1 sends it to the OLED; V2 shows the same
//  frame twice as large on the 320x240 TFT (see display.cpp).
//
//  OLED display (step 3.1)
//  Drawing uses the Adafruit GFX library into a 1 KB RAM buffer.
//  Sending that buffer with Adafruit's display() takes ~25 ms at
//  400 kHz, during which nothing else runs. Instead, commit()
//  sends the frame in small pieces: one 16-byte piece per loop
//  pass (~0.5 ms each), 64 pieces per frame.
//  Rule: only draw when ready() is true (the previous frame has
//  been sent); then draw the whole frame and call commit().
//  Without a display (not found on I2C) everything is a no-op.
// ============================================================
#include <Arduino.h>
#include <Adafruit_SSD1306.h>     // V1 driver; on V2 only the colour names are used
#include "hw_profile.h"

#if HW_HAS_TFT
// V2: the 128x64 frame lives in RAM in the same byte layout as the SSD1306
// (8 vertical pixels per byte), so all screens and fastgfx work unchanged.
class FrameBuffer : public Adafruit_GFX {
 public:
  FrameBuffer() : Adafruit_GFX(128, 64) {}
  void drawPixel(int16_t x, int16_t y, uint16_t color) override;
  uint8_t* getBuffer() { return buf; }
  void clearDisplay() { memset(buf, 0, sizeof(buf)); }
 private:
  uint8_t buf[128 * 64 / 8];
};
#endif

namespace display {
#if HW_HAS_TFT
using Gfx = FrameBuffer;
#else
using Gfx = Adafruit_SSD1306;
#endif
bool begin();                 // true if the display answered (V2 TFT: always true)
void update();                // call every loop pass: sends one piece
bool present();
bool ready();                 // previous frame fully sent: OK to draw
void commit();                // start sending the drawn frame
Gfx& gfx();                   // drawing functions (clearDisplay, print, ...)
uint32_t frames();            // frames sent (diagnostics)
}  // namespace display
