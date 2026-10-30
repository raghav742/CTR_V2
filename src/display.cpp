#include "display.h"
#include <Wire.h>
#include "hw_profile.h"

namespace display {
namespace {
  constexpr uint8_t W = 128, H = 64;
  constexpr uint16_t BYTES = W * H / 8;     // 1024
  constexpr uint8_t  CHUNK = 16;            // data bytes per I2C transfer
  Adafruit_SSD1306 oled(W, H, &Wire, -1);
  bool     ok = false;
  bool     sending = false;
  uint16_t pos = 0;
  uint32_t frameCount = 0;

  // Address window = whole screen, so data simply fills it from (0,0)
  void startFrame() {
    Wire.beginTransmission(hw::OLED_ADDR);
    Wire.write((uint8_t)0x00);                       // command stream
    Wire.write((uint8_t)0x21); Wire.write((uint8_t)0); Wire.write((uint8_t)(W - 1));   // columns
    Wire.write((uint8_t)0x22); Wire.write((uint8_t)0); Wire.write((uint8_t)(H / 8 - 1)); // pages
    Wire.endTransmission();
  }
}

bool begin() {
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);                   // never hang on a stuck bus
  // Adafruit's begin() reports success even without a display: ask the bus first
#ifndef SIM_FAKE_DISPLAY
  Wire.beginTransmission(hw::OLED_ADDR);
  if (Wire.endTransmission() != 0) { ok = false; return false; }
#endif
  ok = oled.begin(SSD1306_SWITCHCAPVCC, hw::OLED_ADDR);
  if (ok) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.cp437(true);
  }
  return ok;
}

bool present() { return ok; }
bool ready()   { return ok && !sending; }
Adafruit_SSD1306& gfx() { return oled; }
uint32_t frames() { return frameCount; }

void commit() {
  if (!ok || sending) return;
  startFrame();
  pos = 0;
  sending = true;
}

void update() {
  if (!sending) return;
  const uint8_t* buf = oled.getBuffer();
  Wire.beginTransmission(hw::OLED_ADDR);
  Wire.write((uint8_t)0x40);                         // data stream
  Wire.write(buf + pos, CHUNK);
  Wire.endTransmission();
  pos += CHUNK;
  if (pos >= BYTES) { sending = false; frameCount++; }
}
}  // namespace display
