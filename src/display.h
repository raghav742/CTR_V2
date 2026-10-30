#pragma once
// ============================================================
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
#include <Adafruit_SSD1306.h>

namespace display {
bool begin();                 // true if the OLED answered
void update();                // call every loop pass: sends one piece
bool present();
bool ready();                 // previous frame fully sent: OK to draw
void commit();                // start sending the drawn frame
Adafruit_SSD1306& gfx();      // drawing functions (clearDisplay, print, ...)
uint32_t frames();            // frames sent (diagnostics)
}  // namespace display
