#include "console.h"
#include "hw_profile.h"
#include "version.h"
#include "loop_stats.h"
#include "encoder.h"
#include "limit_switches.h"
#include "status_led.h"

namespace console {
namespace {
  constexpr uint8_t LINE_MAX = 64;
  char    line[LINE_MAX];
  uint8_t len      = 0;
  bool    overflow = false;
  bool    watchOn  = false;

  // ---------- commands ----------
  void cmdHelp() {
    Serial.println(F("OK commands:"));
    Serial.println(F("  help              this list"));
    Serial.println(F("  status            firmware, profile, loop speed"));
    Serial.println(F("  inputs            limit switches and encoder state"));
    Serial.println(F("  watch on|off      print input events live"));
    Serial.println(F("  led g|b|c|r|off [slow|fast]   set status LED"));
  }

  void cmdStatus() {
    Serial.print(F("OK fw="));       Serial.print(FW_VERSION);
    Serial.print(F(" profile=\""));  Serial.print(hw::PROFILE_NAME);
    Serial.print(F("\" loops/s="));  Serial.print(loop_stats::loopsPerSecond());
    Serial.print(F(" maxloop_us=")); Serial.print(loop_stats::maxLoopUs());
    Serial.print(F(" uptime_s="));   Serial.println(millis() / 1000);
  }

  void cmdInputs() {
    Serial.print(F("OK"));
    for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++) {
      Serial.print(' '); Serial.print(hw::SWITCHES[i].name);
      Serial.print('=');  Serial.print(limit_switches::pressed(i) ? 1 : 0);
    }
    Serial.print(F(" enc_pos=")); Serial.print(encoder::position());
    Serial.print(F(" enc_btn=")); Serial.println(encoder::isDown() ? 1 : 0);
  }

  void cmdWatch(const char* arg) {
    if      (arg && !strcmp(arg, "on"))  watchOn = true;
    else if (arg && !strcmp(arg, "off")) watchOn = false;
    else { Serial.println(F("ERR usage: watch on|off")); return; }
    Serial.print(F("OK watch ")); Serial.println(watchOn ? F("on") : F("off"));
  }

  void cmdLed(const char* c, const char* p) {
    using status_led::Color; using status_led::Pattern;
    Color col;
    if      (!c)                 { Serial.println(F("ERR usage: led g|b|c|r|off [slow|fast]")); return; }
    else if (!strcmp(c, "g"))    col = Color::Green;
    else if (!strcmp(c, "b"))    col = Color::Blue;
    else if (!strcmp(c, "c"))    col = Color::Cyan;
    else if (!strcmp(c, "r"))    col = Color::Red;
    else if (!strcmp(c, "off"))  col = Color::Off;
    else { Serial.println(F("ERR unknown colour")); return; }
    Pattern pat = Pattern::Solid;
    if (p && !strcmp(p, "slow")) pat = Pattern::SlowBlink;
    if (p && !strcmp(p, "fast")) pat = Pattern::FastBlink;
    status_led::set(col, pat);
    Serial.print(F("OK led ")); Serial.println(status_led::colorName(col));
  }

  // Split the line into words and dispatch
  void execute(char* s) {
    char* argv[4] = { nullptr };
    uint8_t argc = 0;
    for (char* tok = strtok(s, " \t"); tok && argc < 4; tok = strtok(nullptr, " \t"))
      argv[argc++] = tok;
    if (argc == 0) return;
    for (char* p = argv[0]; *p; p++) *p = tolower(*p);   // commands are case-insensitive

    if      (!strcmp(argv[0], "help"))   cmdHelp();
    else if (!strcmp(argv[0], "status")) cmdStatus();
    else if (!strcmp(argv[0], "inputs")) cmdInputs();
    else if (!strcmp(argv[0], "watch"))  cmdWatch(argv[1]);
    else if (!strcmp(argv[0], "led"))    cmdLed(argv[1], argv[2]);
    else { Serial.print(F("ERR unknown command: ")); Serial.println(argv[0]); }
  }
}

void begin() { len = 0; overflow = false; }

void update() {
  // Handle at most what has already arrived; never wait for more.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (overflow)   Serial.println(F("ERR line too long"));
      else if (len)   { line[len] = '\0'; execute(line); }
      len = 0; overflow = false;
    } else if (len < LINE_MAX - 1) {
      line[len++] = c;
    } else {
      overflow = true;
    }
  }
}

bool watching() { return watchOn; }
}  // namespace console
