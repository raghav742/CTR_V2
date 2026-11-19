#include "display.h"
#include <Wire.h>
#include "hw_profile.h"
#if HW_HAS_TFT
#include <SPI.h>
#include <Adafruit_ILI9341.h>
#endif

namespace display {
namespace {
  constexpr uint8_t W = 128, H = 64;
  constexpr uint16_t BYTES = W * H / 8;     // 1024
  constexpr uint8_t  CHUNK = 16;            // data bytes per I2C transfer
#if HW_HAS_TFT
  FrameBuffer oled;                 // frame buffer only (name kept: same code below)
#else
  Adafruit_SSD1306 oled(W, H, &Wire, -1);
#endif
  bool     ok = false;
  bool     sending = false;
  uint16_t pos = 0;
  uint32_t frameCount = 0;

#if HW_HAS_TFT
  // ---- V2: 2.8" TFT (ILI9341, 320x240), UNTESTED ON HARDWARE ----
  // The TFT has no frame buffer we could afford (150 KB), so the 128x64
  // frame is shown at 2x (256x128 pixels) in the middle of the screen.
  // Like the OLED, it is sent in small pieces, one per loop pass: the
  // frame is cut into 128 pieces of 8 bytes (8 columns x 8 rows); a
  // 16-bit checksum per piece skips pieces that did not change, so a
  // typical frame (a few changed numbers) costs only a few passes.
  // One drawn piece = 16x16 TFT pixels (~0.5-1 ms of SPI at 8 MHz).
  constexpr uint8_t  SCALE  = 2;
  constexpr uint16_t TFT_X0 = (320 - W * SCALE) / 2;   // 32
  constexpr uint16_t TFT_Y0 = 64;                      // below the header
  constexpr uint8_t  TCHUNK = 8;                       // frame bytes per piece
  constexpr uint8_t  NPIECES = BYTES / TCHUNK;         // 128
  constexpr uint16_t FG = ILI9341_WHITE;
  constexpr uint16_t BG = ILI9341_BLACK;
  constexpr uint8_t  FULL_REDRAW_EVERY = 50;           // frames: heals any missed piece
  Adafruit_ILI9341 tft(hw::PIN_TFT_CS, hw::PIN_TFT_DC, hw::PIN_TFT_RST);
  uint16_t sums[NPIECES];                              // checksum of each piece as drawn
  uint8_t  framesSinceFull = 0;
  bool     forceAll = true;

  uint16_t pieceSum(const uint8_t* p) {               // Fletcher-16 over 8 bytes
    uint8_t a = 0, b = 0;
    for (uint8_t i = 0; i < TCHUNK; i++) { a += p[i]; b += a; }
    return ((uint16_t)b << 8) | a;
  }

  // Draw piece n: frame bytes = 8 columns of one 8-row page; each bit = 2x2 pixels
  void drawPiece(uint8_t n, const uint8_t* buf) {
    uint8_t page = n / (W / TCHUNK), col0 = (n % (W / TCHUNK)) * TCHUNK;
    const uint8_t* p = buf + (uint16_t)page * W + col0;
    tft.startWrite();
    tft.setAddrWindow(TFT_X0 + col0 * SCALE, TFT_Y0 + page * 8 * SCALE, TCHUNK * SCALE, 8 * SCALE);
    for (uint8_t row = 0; row < 8 * SCALE; row++) {
      uint8_t mask = 1 << (row / SCALE);
      bool cur = p[0] & mask;
      uint8_t run = 0;
      for (uint8_t c = 0; c < TCHUNK; c++) {           // runs of equal colour: fewer SPI calls
        bool on = p[c] & mask;
        if (on != cur) { tft.writeColor(cur ? FG : BG, run); run = 0; cur = on; }
        run += SCALE;
      }
      tft.writeColor(cur ? FG : BG, run);
    }
    tft.endWrite();
  }

  void drawStatic() {                                  // once at start: header and help line
    tft.fillScreen(BG);
    tft.setTextColor(ILI9341_CYAN);
    tft.setTextSize(2);
    tft.setCursor(TFT_X0, 20);
    tft.print(F("Concentric Tube Robot"));
    tft.drawRect(TFT_X0 - 3, TFT_Y0 - 3, W * SCALE + 6, H * SCALE + 6, ILI9341_DARKGREY);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_LIGHTGREY);
    tft.setCursor(TFT_X0, TFT_Y0 + H * SCALE + 16);
    tft.print(F("Turn: choose   Press: select / STOP   Hold: back"));
  }
  uint8_t piece = 0;
#endif

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
#if HW_HAS_TFT
  // No I2C on V2: the screens draw into FrameBuffer (RAM), sent to the TFT
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.cp437(true);
  pinMode(hw::PIN_TFT_LITE, OUTPUT);
  digitalWrite(hw::PIN_TFT_LITE, HIGH);
  tft.begin();                     // SPI at the fastest the Mega allows (8 MHz)
  tft.setRotation(1);              // landscape, 320 x 240
  drawStatic();
  forceAll = true;
  ok = true;                       // the ILI9341 cannot be detected without MISO
  return ok;
#else
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);                   // never hang on a stuck bus
  // Adafruit's begin() reports success even without a display: ask the bus first
  Wire.beginTransmission(hw::OLED_ADDR);
  if (Wire.endTransmission() != 0) { ok = false; return false; }
  ok = oled.begin(SSD1306_SWITCHCAPVCC, hw::OLED_ADDR);
  if (ok) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.cp437(true);
  }
  return ok;
#endif
}

bool present() { return ok; }
bool ready()   { return ok && !sending; }
Gfx& gfx() { return oled; }
uint32_t frames() { return frameCount; }

void commit() {
  if (!ok || sending) return;
#if HW_HAS_TFT
  piece = 0;
  if (++framesSinceFull >= FULL_REDRAW_EVERY) { framesSinceFull = 0; forceAll = true; }
  sending = true;
  return;
#endif
  startFrame();
  pos = 0;
  sending = true;
}

void update() {
  if (!sending) return;
  const uint8_t* buf = oled.getBuffer();
#if HW_HAS_TFT
  // Skip unchanged pieces (cheap), draw at most one changed piece per pass
  while (piece < NPIECES) {
    uint8_t n = piece++;
    uint16_t sum = pieceSum(buf + (uint16_t)(n / (W / TCHUNK)) * W + (n % (W / TCHUNK)) * TCHUNK);
    if (forceAll || sum != sums[n]) { drawPiece(n, buf); sums[n] = sum; break; }
  }
  if (piece >= NPIECES) { sending = false; forceAll = false; frameCount++; }
  return;
#endif
  Wire.beginTransmission(hw::OLED_ADDR);
  Wire.write((uint8_t)0x40);                         // data stream
  Wire.write(buf + pos, CHUNK);
  Wire.endTransmission();
  pos += CHUNK;
  if (pos >= BYTES) { sending = false; frameCount++; }
}
}  // namespace display

#if HW_HAS_TFT
// SSD1306 byte layout: byte (y/8)*128 + x, bit y%8
void FrameBuffer::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if (x < 0 || y < 0 || x >= 128 || y >= 64) return;
  uint8_t& b = buf[(y >> 3) * 128 + x];
  uint8_t m = 1 << (y & 7);
  if (color == SSD1306_WHITE)      b |= m;
  else if (color == SSD1306_BLACK) b &= ~m;
  else                             b ^= m;   // SSD1306_INVERSE
}
#endif
