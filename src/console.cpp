#include "console.h"
#include "hw_profile.h"
#include "version.h"
#include "loop_stats.h"
#include "encoder.h"
#include "limit_switches.h"
#include "status_led.h"
#include "drivers.h"
#include "steppers.h"
#include "motion.h"
#include "system_state.h"
#include "safety.h"

namespace console {
namespace {
  constexpr uint8_t LINE_MAX = 96;
  constexpr uint8_t MAX_ARGS = 10;
  char    line[LINE_MAX];
  uint8_t len      = 0;
  bool    overflow = false;
  bool    watchOn  = false;

  // ---------- commands ----------
  void cmdHelp() {
    Serial.println(F("OK commands:"));
    Serial.println(F("  help              this list"));
    Serial.println(F("  status            firmware, profile, state, loop speed"));
    Serial.println(F("  state             current state (and fault reason)"));
    Serial.println(F("  ack               acknowledge a fault (or hold the encoder 1 s)"));
    Serial.println(F("  safety            reset cause, last stop reaction time, watchdog"));
    Serial.println(F("  wdtest            freeze on purpose to test the watchdog"));
    Serial.println(F("  inputs            limit switches and encoder state"));
    Serial.println(F("  watch on|off      print input events live"));
    Serial.println(F("  led g|b|c|r|off [slow|fast]   set status LED"));
    Serial.println(F("  drivers           driver detection, current, microsteps"));
    Serial.println(F("  move <1-6> <amount> [speed]   mm & mm/s (translation) or deg & rpm (rotation)"));
    Serial.println(F("  mv <axis>=<amount> ... [t=<s>] [v=<%>]   coordinated move, all finish together"));
    Serial.println(F("  stop              stop all motors"));
    Serial.println(F("  pos               positions of all axes"));
    Serial.println(F("  en <1-6|all> / dis <1-6|all>  enable / disable motor drivers"));
    Serial.println(F("  isr reset         reset the interrupt timing statistic"));
    Serial.println(F("  chop <1-6> stealth|spread|hybrid [mm/s or rpm]   driver chopper mode"));
    Serial.println(F("  ms <1-6> <2|4|8|16|32>   microstepping (position is kept)"));
    Serial.println(F("  cur <1-6> <mA>    motor current (RMS)"));
  }

  // Parse "1".."6" -> 0..5, or -1
  int8_t parseAxis(const char* a) {
    if (!a) return -1;
    int n = atoi(a);
    return (n >= 1 && n <= (int)hw::NUM_AXES) ? n - 1 : -1;
  }

  void cmdMove(const char* a, const char* amt, const char* spd) {
    int8_t i = parseAxis(a);
    if (i < 0 || !amt) { Serial.println(F("ERR usage: move <1-6> <amount> [speed]")); return; }
    motion::moveRelative(i, atof(amt), spd ? atof(spd) : 0);
  }

  // mv 2=30 4=30 6=30 [t=5] [v=50]
  void cmdMv(uint8_t argc, char** argv) {
    float amount[hw::NUM_AXES] = { 0 };
    uint8_t mask = 0;
    float pct = 100, dur = 0;
    for (uint8_t k = 1; k < argc; k++) {
      char* eq = strchr(argv[k], '=');
      if (!eq) { Serial.print(F("ERR bad argument: ")); Serial.println(argv[k]); return; }
      *eq = '\0';
      const char* key = argv[k]; float val = atof(eq + 1);
      if      (!strcmp(key, "t")) dur = val;
      else if (!strcmp(key, "v")) pct = val;
      else {
        int8_t i = parseAxis(key);
        if (i < 0) { Serial.print(F("ERR bad axis: ")); Serial.println(key); return; }
        amount[i] = val; mask |= _BV(i);
      }
    }
    if (!mask) { Serial.println(F("ERR usage: mv <axis>=<amount> ... [t=<s>] [v=<%>]")); return; }
    motion::moveGroup(amount, mask, pct, dur);
  }

  void cmdPos() {
    Serial.print(F("OK"));
    for (uint8_t i = 0; i < hw::NUM_AXES; i++) {
      Serial.print(' '); Serial.print(hw::AXES[i].name); Serial.print('=');
      Serial.print(motion::position(i), 2);
      if (motion::busy(i)) Serial.print('*');
    }
    Serial.println();
  }

  void cmdEnable(const char* a, bool on) {
    if (a && !strcmp(a, "all")) {
      if (!on) motion::stopAll(F("drivers disabled"));
      for (uint8_t i = 0; i < hw::NUM_AXES; i++) drivers::enable(i, on);
    } else {
      int8_t i = parseAxis(a);
      if (i < 0) { Serial.println(F("ERR usage: en|dis <1-6|all>")); return; }
      if (!on && motion::busy(i)) motion::stopAll(F("driver disabled"));
      drivers::enable(i, on);
    }
    Serial.println(on ? F("OK enabled") : F("OK disabled"));
  }

  void cmdChop(const char* a, const char* m, const char* thr) {
    int8_t i = parseAxis(a);
    if (i < 0 || !m) { Serial.println(F("ERR usage: chop <1-6> stealth|spread|hybrid [speed]")); return; }
    drivers::Chopper c;
    if      (!strcmp(m, "stealth")) c = drivers::Chopper::Stealth;
    else if (!strcmp(m, "spread"))  c = drivers::Chopper::Spread;
    else if (!strcmp(m, "hybrid"))  c = drivers::Chopper::Hybrid;
    else { Serial.println(F("ERR mode: stealth|spread|hybrid")); return; }
    float sps = 0;
    if (c == drivers::Chopper::Hybrid) {
      float v = thr ? atof(thr) : 5.0f;   // default switch-over: 5 mm/s or 5 rpm
      bool tra = hw::AXES[i].type == AxisType::Translation;
      sps = tra ? v * motion::stepsPerUnit(i) : v * 6.0f * motion::stepsPerUnit(i);
    }
    drivers::setChopper(i, c, sps);
    Serial.println(F("OK chopper set"));
  }

  void cmdMs(const char* a, const char* v) {
    int8_t i = parseAxis(a);
    int ms = v ? atoi(v) : 0;
    if (i < 0 || !(ms == 2 || ms == 4 || ms == 8 || ms == 16 || ms == 32)) {
      Serial.println(F("ERR usage: ms <1-6> <2|4|8|16|32>")); return; }
    if (motion::busy(i)) { Serial.println(F("ERR axis busy")); return; }
    uint16_t old = drivers::microsteps(i);
    int32_t pos = steppers::position(i);
    drivers::setMicrosteps(i, ms);
    steppers::setPosition(i, (int32_t)((int64_t)pos * ms / old));   // same physical position
    Serial.print(F("OK ms=")); Serial.println(ms);
  }

  void cmdCur(const char* a, const char* v) {
    int8_t i = parseAxis(a);
    int mA = v ? atoi(v) : 0;
    if (i < 0 || mA < 100 || mA > 2000) { Serial.println(F("ERR usage: cur <1-6> <100..2000>")); return; }
    drivers::setCurrent(i, mA);
    Serial.print(F("OK current=")); Serial.print(mA); Serial.println(F("mA"));
  }

  void cmdStatus() {
    Serial.print(F("OK fw="));       Serial.print(FW_VERSION);
    Serial.print(F(" profile=\""));  Serial.print(hw::PROFILE_NAME);
    Serial.print(F("\" state="));   Serial.print(sys::name(sys::get()));
    Serial.print(F(" loops/s="));    Serial.print(loop_stats::loopsPerSecond());
    Serial.print(F(" maxloop_us=")); Serial.print(loop_stats::maxLoopUs());
    Serial.print(F(" isr_max_us=")); Serial.print(steppers::isrMaxUs(), 1);
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
    char* argv[MAX_ARGS] = { nullptr };
    uint8_t argc = 0;
    for (char* tok = strtok(s, " \t"); tok && argc < MAX_ARGS; tok = strtok(nullptr, " \t"))
      argv[argc++] = tok;
    if (argc == 0) return;
    for (char* p = argv[0]; *p; p++) *p = tolower(*p);   // commands are case-insensitive

    if      (!strcmp(argv[0], "help"))   cmdHelp();
    else if (!strcmp(argv[0], "status")) cmdStatus();
    else if (!strcmp(argv[0], "state"))  sys::report();
    else if (!strcmp(argv[0], "ack"))    { if (sys::acknowledge()) Serial.println(F("OK acknowledged")); }
    else if (!strcmp(argv[0], "inputs")) cmdInputs();
    else if (!strcmp(argv[0], "watch"))  cmdWatch(argv[1]);
    else if (!strcmp(argv[0], "led"))    cmdLed(argv[1], argv[2]);
    else if (!strcmp(argv[0], "drivers")) drivers::report();
    else if (!strcmp(argv[0], "move"))   cmdMove(argv[1], argv[2], argv[3]);
    else if (!strcmp(argv[0], "mv"))     cmdMv(argc, argv);
    else if (!strcmp(argv[0], "stop"))   { safety::userStop(F("command"), micros()); Serial.println(F("OK stop")); }
    else if (!strcmp(argv[0], "safety")) safety::report();
    else if (!strcmp(argv[0], "wdtest")) safety::watchdogTest();
    else if (!strcmp(argv[0], "pos"))    cmdPos();
    else if (!strcmp(argv[0], "en"))     cmdEnable(argv[1], true);
    else if (!strcmp(argv[0], "dis"))    cmdEnable(argv[1], false);
    else if (!strcmp(argv[0], "chop"))   cmdChop(argv[1], argv[2], argv[3]);
    else if (!strcmp(argv[0], "ms"))     cmdMs(argv[1], argv[2]);
    else if (!strcmp(argv[0], "cur"))    cmdCur(argv[1], argv[2]);
    else if (!strcmp(argv[0], "isr"))    { steppers::resetIsrStats(); Serial.println(F("OK isr stats reset")); }
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
