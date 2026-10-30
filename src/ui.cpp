#include "ui.h"
#include "display.h"
#include "fastgfx.h"
#include "buzzer.h"
#include "serial_io.h"
#include "hw_profile.h"
#include "version.h"
#include "motion.h"
#include "homing.h"
#include "calib.h"
#include "rotation_zero.h"
#include "drivers.h"
#include "system_state.h"

// ============================================================
//  Screen manager (steps 3.1, 3.2)
//  Screens: Splash, Status, Menu, Service, Progress (homing /
//  calib / park), Message (short notice), Fault.
//  Every screen is drawn in parts, one part per loop pass, then
//  sent to the OLED in pieces (display.cpp): no pass takes long.
//  Visual style from Casper's V1 firmware: white title bar with
//  black text, small icons, inverted selection bar, strong
//  inverted footer ("PUSH = STOP").
// ============================================================
namespace ui {
namespace {
  using namespace fastgfx;

  constexpr uint16_t SPLASH_MS     = 2000;
  constexpr uint16_t LIVE_MS       = 200;     // status / progress: 5 frames per second
  constexpr uint16_t MENU_MS       = 1000;    // menus: on input, else once a second
  constexpr uint16_t MESSAGE_MS    = 1800;    // how long a notice stays
  constexpr uint32_t IDLE_MS       = 120000;  // no input for 2 min -> status screen
  constexpr uint8_t  TITLE_H       = 11;      // white title bar (yellow zone is y < 16)
  constexpr uint8_t  ROW_H         = 12;
  constexpr uint8_t  ROWS          = 4;       // menu rows visible (y 16..63)

  enum class Screen : uint8_t { Splash, Status, Menu, Service, Progress, Message, Fault };
  enum class Job    : uint8_t { None, Home, Calib, Park };

  Screen   screen = Screen::Splash;
  uint32_t screenSince = 0, lastDraw = 0, lastInput = 0;
  bool     dirty = true;                       // redraw as soon as possible
  int8_t   drawPart = -1;                      // -1 = not drawing a frame
  Job      job = Job::None;                    // what the progress screen shows

  // Main menu
  enum : uint8_t { M_DEMO, M_JOG, M_HOME, M_STATUS, M_SERVICE, M_COUNT };
  // Service menu
  enum : uint8_t { S_CALIB, S_ZERO, S_PARK, S_DRIVERS, S_SOUND, S_BACK, S_COUNT };
  uint8_t mainSel = 0, mainTop = 0, svcSel = 0, svcTop = 0;

  // Message screen
  const __FlashStringHelper* msgTitle = nullptr;
  char     msg1[22], msg2[22];
  uint32_t msgSince = 0;
  Screen   msgReturn = Screen::Menu;

  Adafruit_SSD1306& g() { return display::gfx(); }

  void go(Screen s) { screen = s; screenSince = millis(); dirty = true; }

  void message(const __FlashStringHelper* t, const __FlashStringHelper* l1, const __FlashStringHelper* l2,
               Screen back = Screen::Menu) {
    msgTitle = t;
    strncpy_P(msg1, l1 ? (const char*)l1 : "", sizeof(msg1) - 1); msg1[sizeof(msg1) - 1] = 0;
    strncpy_P(msg2, l2 ? (const char*)l2 : "", sizeof(msg2) - 1); msg2[sizeof(msg2) - 1] = 0;
    msgReturn = back;
    msgSince = millis();
    go(Screen::Message);
  }

  bool busy() { return motion::anyBusy() || homing::active() || calib::active() || rotzero::parking(); }

  // ---------------- drawing helpers (Casper's style) ----------------
  void title(const __FlashStringHelper* txt) {
    fill(0, 0, 127, TITLE_H - 1, true);
    text(3, 2, txt, true);
  }
  void titleTag(const __FlashStringHelper* tag) {
    uint8_t x = 126 - CHAR_W * strlen_P((const char*)tag);
    text(x, 2, tag, true);
  }
  // Strong inverted footer (rows 54..63) with centred text
  void footer(const __FlashStringHelper* s) {
    fill(0, 54, 127, 63, true);
    uint8_t x = (128 - CHAR_W * strlen_P((const char*)s)) / 2;
    text(x, 55, s, true);
  }
  void heroGlyph(int cx, int cy) {
    g().drawCircle(cx, cy, 7, SSD1306_WHITE);
    g().drawCircle(cx, cy, 4, SSD1306_WHITE);
    g().fillCircle(cx, cy, 1, SSD1306_WHITE);
    g().drawLine(cx, cy, cx + 8, cy - 6, SSD1306_WHITE);
  }
  // 10x10 menu icons; cy0 = top of the cell, col = row colour
  void icon(uint8_t item, int cy0, uint16_t col) {
    switch (item) {
      case M_DEMO:                                       // play (Casper)
        g().fillTriangle(4, cy0 + 1, 4, cy0 + 9, 12, cy0 + 5, col);
        break;
      case M_JOG:                                        // four-way arrows
        g().drawLine(3, cy0 + 5, 13, cy0 + 5, col);
        g().drawLine(8, cy0, 8, cy0 + 10, col);
        g().drawLine(3, cy0 + 5, 5, cy0 + 3, col);  g().drawLine(3, cy0 + 5, 5, cy0 + 7, col);
        g().drawLine(13, cy0 + 5, 11, cy0 + 3, col); g().drawLine(13, cy0 + 5, 11, cy0 + 7, col);
        g().drawLine(8, cy0, 6, cy0 + 2, col);      g().drawLine(8, cy0, 10, cy0 + 2, col);
        g().drawLine(8, cy0 + 10, 6, cy0 + 8, col); g().drawLine(8, cy0 + 10, 10, cy0 + 8, col);
        break;
      case M_HOME:                                       // datum (Casper's calibrate icon)
        g().drawCircle(8, cy0 + 4, 4, col);
        g().fillCircle(8, cy0 + 4, 1, col);
        g().drawLine(8, cy0 - 1, 8, cy0, col);
        g().drawLine(8, cy0 + 8, 8, cy0 + 9, col);
        g().drawLine(2, cy0 + 4, 3, cy0 + 4, col);
        g().drawLine(13, cy0 + 4, 14, cy0 + 4, col);
        break;
      case M_STATUS: {                                   // ECG (Casper)
        int m = cy0 + 5;
        g().drawLine(3, m, 5, m, col);
        g().drawLine(5, m, 7, cy0 + 1, col);
        g().drawLine(7, cy0 + 1, 9, cy0 + 9, col);
        g().drawLine(9, cy0 + 9, 10, m, col);
        g().drawLine(10, m, 12, m, col);
        break;
      }
      case M_SERVICE:                                    // gear
        g().drawCircle(8, cy0 + 5, 3, col);
        g().drawPixel(8, cy0, col);  g().drawPixel(8, cy0 + 10, col);
        g().drawPixel(3, cy0 + 5, col); g().drawPixel(13, cy0 + 5, col);
        g().drawPixel(5, cy0 + 2, col); g().drawPixel(11, cy0 + 2, col);
        g().drawPixel(5, cy0 + 8, col); g().drawPixel(11, cy0 + 8, col);
        break;
    }
  }

  // ---------------- menu content ----------------
  const __FlashStringHelper* mainName(uint8_t i) {
    switch (i) {
      case M_DEMO:    return F("Demo");
      case M_JOG:     return F("Jog");
      case M_HOME:    return F("Home");
      case M_STATUS:  return F("Status");
      default:        return F("Service");
    }
  }
  const __FlashStringHelper* svcName(uint8_t i) {
    switch (i) {
      case S_CALIB:   return F("Calibrate limits");
      case S_ZERO:    return F("Zero rotations");
      case S_PARK:    return F("Park (power off)");
      case S_DRIVERS: return F("Driver scan");
      case S_SOUND:   return buzzer::muted() ? F("Sound: off") : F("Sound: on");
      default:        return F("< Back");
    }
  }
  bool needsHome(uint8_t i) { return (i == M_DEMO || i == M_JOG) && !sys::homed(); }

  // One menu row (row = 0..3 on screen); icons only in the main menu
  void menuRow(uint8_t row, uint8_t item, bool sel, bool isMain, uint8_t count, uint8_t top) {
    uint8_t y = 16 + ROW_H * row;
    if (sel) {
      fill(0, y, 127, y + ROW_H - 1, true);
      fill(125, y + 1, 126, y + ROW_H - 2, false);       // Casper's notch
    }
    uint16_t col = sel ? SSD1306_BLACK : SSD1306_WHITE;
    uint8_t x = 4;
    if (isMain) { icon(item, y + 1, col); x = 17; }
    text(x, y + 2, isMain ? mainName(item) : svcName(item), sel);
    if (isMain && needsHome(item)) text(80, y + 2, F("(home)"), sel);
    // scroll hints: more items above / below
    if (row == 0 && top > 0)                text(118, y + 2, F("\x18"), sel);
    if (row == ROWS - 1 && top + ROWS < count) text(118, y + 2, F("\x19"), sel);
  }

  // Keep the selected item inside the visible window
  void scrollTo(uint8_t sel, uint8_t& top) {
    if (sel < top) top = sel;
    if (sel >= top + ROWS) top = sel - ROWS + 1;
  }

  // ---------------- screens, drawn in parts ----------------
  // Each returns true when the frame is complete.

  void tubeRow(uint8_t k, uint8_t y) {                  // positions of one tube
    uint8_t rot = 2 * k, tra = 2 * k + 1;               // axis order: Rot, Tra per tube
    uint8_t x = chr(0, y, 'T');
    chr(x, y, '1' + k);
    x = num(16, y, motion::position(tra), 6, 1);
    x = text(x, y, homing::axisHomed(tra) ? F("mm") : F("?m"));
    chr(x, y, motion::busy(tra) ? '*' : ' ');
    x = num(76, y, motion::position(rot), 6, 0);
    x = chr(x, y, (char)0xF8);                          // degree sign (cp437)
    x = chr(x, y, rotzero::referenced(rot) ? ' ' : '?');
    chr(x, y, motion::busy(rot) ? '*' : ' ');
  }

  bool drawStatus(uint8_t part) {
    if (part == 0) { clear(); title(F("STATUS")); titleTag(sys::name(sys::get())); return false; }
    if (part <= 3) { tubeRow(part - 1, 16 + 12 * (part - 1)); return false; }
    if (!sys::homed()) text(0, 55, F("Not homed"));
    else { uint8_t x = text(0, 55, F("fw ")); text(x, 55, F(FW_VERSION)); }
    text(80, 55, F("PUSH:menu"));
    return true;
  }

  bool drawMenu(uint8_t part, bool isMain) {
    if (part == 0) {
      clear();
      title(isMain ? F("MAIN MENU") : F("SERVICE"));
      titleTag(sys::name(sys::get()));
      return false;
    }
    uint8_t row = part - 1;
    uint8_t count = isMain ? (uint8_t)M_COUNT : (uint8_t)S_COUNT;
    uint8_t top = isMain ? mainTop : svcTop, sel = isMain ? mainSel : svcSel;
    uint8_t item = top + row;
    if (item < count) menuRow(row, item, item == sel, isMain, count, top);
    return row == ROWS - 1;
  }

  bool drawProgress(uint8_t part) {
    if (part == 0) {
      clear();
      title(job == Job::Home ? F("HOMING") : job == Job::Calib ? F("CALIBRATING") : F("PARKING"));
      titleTag(F("RUN"));
      return false;
    }
    if (part <= 3) {
      uint8_t k = part - 1, y = 16 + 12 * k;
      if (job == Job::Home) {                           // Casper's calibration list
        uint8_t tra = 2 * k + 1;
        chr(30, y, 'T'); chr(36, y, '1' + k);
        if (homing::axisHomed(tra)) {
          g().drawLine(58, y + 3, 60, y + 5, SSD1306_WHITE);   // check mark
          g().drawLine(60, y + 5, 64, y, SSD1306_WHITE);
          text(72, y, F("ready"));
        } else {
          g().drawCircle(60, y + 3, 2, SSD1306_WHITE);
          text(72, y, F("homing"));
        }
      } else {
        tubeRow(k, y);
      }
      return false;
    }
    footer(F("PUSH = STOP"));
    return true;
  }

  bool drawMessage(uint8_t) {
    clear();
    title(msgTitle ? msgTitle : F(""));
    text(3, 22, msg1);
    text(3, 34, msg2);
    return true;
  }

  bool drawFault(uint8_t part) {
    if (part == 0) {
      clear();
      title(F("ERROR"));
      titleTag(F("STOP"));
      g().drawTriangle(16, 16, 6, 38, 26, 38, SSD1306_WHITE);  // warning triangle (Casper)
      fill(15, 23, 16, 30, true);
      fill(15, 33, 16, 34, true);
      return false;
    }
    // Reason on up to two lines of 15 characters
    const char* r = sys::faultReason();
    uint8_t len = strlen(r);
    if (len <= 15) {
      text(34, 24, r);
    } else {
      uint8_t sp = 15;
      while (sp > 1 && r[sp] != ' ') sp--;
      if (sp <= 1) sp = 15;
      char a[16];
      memcpy(a, r, sp); a[sp] = 0;
      text(34, 20, a);
      text(34, 30, r + sp + (r[sp] == ' ' ? 1 : 0));
    }
    footer(F("PUSH = CONFIRM"));
    return true;
  }

  // Splash (Casper's design): drawn once; afterwards only the progress bar grows
  void drawSplashOnce() {
    g().clearDisplay();
    g().drawRect(0, 0, 128, 64, SSD1306_WHITE);
    heroGlyph(64, 12);
    g().setTextSize(2);
    g().setTextColor(SSD1306_WHITE);
    g().setCursor(28, 22);
    g().print(F("CTR V2"));
    text(1, 40, F("Concentric Tube Robot"));
    text(1, 49, F("Benchmark  Saxion  UT"));
    g().drawRect(8, 57, 112, 6, SSD1306_WHITE);
  }
  void drawSplashBar() {
    uint32_t el = millis() - screenSince;
    uint8_t w = (uint8_t)(108L * (el > SPLASH_MS ? SPLASH_MS : el) / SPLASH_MS);
    if (w) fill(10, 59, 10 + w - 1, 60, true);
  }

  bool drawPartOf(uint8_t part) {
    switch (screen) {
      case Screen::Status:   return drawStatus(part);
      case Screen::Menu:     return drawMenu(part, true);
      case Screen::Service:  return drawMenu(part, false);
      case Screen::Progress: return drawProgress(part);
      case Screen::Message:  return drawMessage(part);
      case Screen::Fault:    return drawFault(part);
      default:               drawSplashBar(); return true;
    }
  }

  // ---------------- actions ----------------
  void startJob(Job j, bool ok) {
    if (ok) { job = j; go(Screen::Progress); buzzer::play(buzzer::Sound::Click); }
    else    message(F("NOT POSSIBLE"), F("See serial log"), nullptr);
  }

  void selectMain() {
    if (needsHome(mainSel)) { message(F("HOME FIRST"), F("Choose 'Home' first"), nullptr); return; }
    switch (mainSel) {
      case M_DEMO:    message(F("DEMO"), F("Coming in step 3.4"), nullptr); break;
      case M_JOG:     message(F("JOG"),  F("Coming in step 3.3"), nullptr); break;
      case M_HOME:    Out.println(F("EV ui home"));   startJob(Job::Home, homing::start(-1, true)); break;
      case M_STATUS:  go(Screen::Status); break;
      case M_SERVICE: svcSel = 0; svcTop = 0; go(Screen::Service); break;
    }
  }

  void selectService() {
    switch (svcSel) {
      case S_CALIB:
        if (!sys::homed()) { message(F("HOME FIRST"), F("Choose 'Home' first"), nullptr, Screen::Service); return; }
        Out.println(F("EV ui calib"));
        startJob(Job::Calib, calib::start(true));
        break;
      case S_ZERO:
        if (rotzero::setZero(-1, true))
          message(F("ZERO SET"), F("Rotations: this"), F("angle is now 0"), Screen::Service);
        break;
      case S_PARK:
        Out.println(F("EV ui park"));
        startJob(Job::Park, rotzero::park(true));
        break;
      case S_DRIVERS: {
        uint8_t n = 0;
        for (uint8_t i = 0; i < hw::NUM_AXES; i++) if (drivers::present(i)) n++;
        msgTitle = F("DRIVER SCAN");
        snprintf_P(msg1, sizeof(msg1), PSTR("Found %u of %u"), n, hw::NUM_AXES);
        msg2[0] = 0;
        for (uint8_t i = 0; i < hw::NUM_AXES && !msg2[0]; i++)
          if (!drivers::present(i)) snprintf_P(msg2, sizeof(msg2), PSTR("Missing: %s"), hw::AXES[i].name);
        if (!msg2[0]) strcpy_P(msg2, PSTR("All drivers OK"));
        msgReturn = Screen::Service; msgSince = millis();
        go(Screen::Message);
        break;
      }
      case S_SOUND:
        buzzer::setMuted(!buzzer::muted());
        dirty = true;
        break;
      default:
        go(Screen::Menu);
        break;
    }
  }

  // When a progress screen ends: short notice of the result
  void jobFinished() {
    Job j = job;
    job = Job::None;
    if (sys::get() == sys::State::Fault) return;          // fault screen takes over
    switch (j) {
      case Job::Home:  message(sys::homed() ? F("HOMING DONE") : F("HOMING STOPPED"),
                               sys::homed() ? F("All tubes at zero") : F("Not all homed"), nullptr); break;
      case Job::Calib: message(F("CALIBRATION"), F("Finished:"), F("see serial log"), Screen::Service); break;
      case Job::Park:  message(F("PARKED"), F("Safe to"), F("power off"), Screen::Service); break;
      default: go(Screen::Menu); break;
    }
  }

  // Screen changes caused by the robot itself (not by the encoder)
  void followRobot() {
    bool fault = sys::get() == sys::State::Fault;
    if (fault && screen != Screen::Fault && screen != Screen::Splash) { job = Job::None; go(Screen::Fault); return; }
    if (!fault && screen == Screen::Fault) { go(Screen::Menu); return; }

    // Homing / calib / park started from the console: show progress too
    bool running = homing::active() || calib::active() || rotzero::parking();
    if (running && screen != Screen::Progress && screen != Screen::Splash && screen != Screen::Fault) {
      job = calib::active() ? Job::Calib : homing::active() ? Job::Home : Job::Park;
      go(Screen::Progress);
      return;
    }
    if (screen == Screen::Progress) {
      // calib ends with a re-home: keep showing progress until everything is done
      if (calib::active()) job = Job::Calib;
      if (!running) jobFinished();
      return;
    }
    if (screen == Screen::Message && millis() - msgSince > MESSAGE_MS) { go(msgReturn); return; }
    if ((screen == Screen::Menu || screen == Screen::Service) && millis() - lastInput > IDLE_MS) go(Screen::Status);
  }
}

// ================================================================
void begin() {
  screen = Screen::Splash;
  screenSince = millis();
  lastInput = millis();
  buzzer::play(buzzer::Sound::Boot);
  if (display::present()) { drawSplashOnce(); display::commit(); }   // once, in setup()
}

bool motionActive() { return busy(); }

void onTurn(int16_t d) {
  lastInput = millis();
  if (screen == Screen::Menu) {
    int16_t s = mainSel + d;
    mainSel = s < 0 ? 0 : s >= M_COUNT ? M_COUNT - 1 : s;
    scrollTo(mainSel, mainTop);
  } else if (screen == Screen::Service) {
    int16_t s = svcSel + d;
    svcSel = s < 0 ? 0 : s >= S_COUNT ? S_COUNT - 1 : s;
    scrollTo(svcSel, svcTop);
  } else if (screen == Screen::Status) {
    go(Screen::Menu);                           // turning wakes the menu
  }
  dirty = true;
}

void onSelect() {
  lastInput = millis();
  switch (screen) {
    case Screen::Menu:    buzzer::play(buzzer::Sound::Click); selectMain(); break;
    case Screen::Service: buzzer::play(buzzer::Sound::Click); selectService(); break;
    case Screen::Status:  go(Screen::Menu); break;
    case Screen::Message: go(msgReturn); break;
    case Screen::Fault:   sys::acknowledge(); break;
    default: break;
  }
}

void onBack() {
  lastInput = millis();
  switch (screen) {
    case Screen::Service: go(Screen::Menu); break;
    case Screen::Menu:    go(Screen::Status); break;
    case Screen::Message: go(msgReturn); break;
    case Screen::Fault:   sys::acknowledge(); break;
    default:              go(Screen::Menu); break;
  }
}

void update() {
  uint32_t now = millis();

  if (screen == Screen::Splash && now - screenSince >= SPLASH_MS) {
    // after the splash: main menu, with "Home" selected if the robot is not homed
    mainSel = sys::homed() ? M_DEMO : M_HOME;
    scrollTo(mainSel, mainTop);
    go(Screen::Menu);
  }
  if (screen != Screen::Splash) followRobot();

  if (!display::ready()) return;                 // previous frame still being sent

  // A frame being drawn in parts: one part per pass
  if (drawPart >= 0) {
    if (drawPartOf(drawPart)) { drawPart = -1; display::commit(); }
    else drawPart++;
    return;
  }

  uint16_t period = (screen == Screen::Status || screen == Screen::Progress || screen == Screen::Splash)
                    ? LIVE_MS : MENU_MS;
  if (!dirty && now - lastDraw < period) return;
  dirty = false;
  lastDraw = now;
  if (screen == Screen::Splash) { drawSplashBar(); display::commit(); }
  else drawPart = 0;
}
}  // namespace ui
