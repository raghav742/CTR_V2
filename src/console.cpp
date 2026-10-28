#include "console.h"
#include "serial_io.h"
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

  // Status stream ("stream <ms>"): one ST line every period.
  // If the send buffer has no room, the line is skipped (counted),
  // so streaming can never slow the main loop down.
  constexpr int ST_ROOM = 128;          // longest ST line, with margin
  uint16_t streamMs   = 0;              // 0 = off
  uint32_t lastStream = 0;
  uint32_t stSkipped  = 0;

  // ---------- commands ----------
  void cmdHelp() {
    Out.println(F("OK commands:  (prefix any command with #<id> to tag its reply)"));
    Out.println(F("  help              this list"));
    Out.println(F("  status            firmware, profile, state, loop speed"));
    Out.println(F("  info              protocol version, axes, units, switches (for programs)"));
    Out.println(F("  ping              reply 'OK pong' (link check)"));
    Out.println(F("  st                one status line: state, busy axes, positions"));
    Out.println(F("  stream <ms>|off   status line every <ms> (20..10000)"));
    Out.println(F("  state             current state (and fault reason)"));
    Out.println(F("  ack               acknowledge a fault (or hold the encoder 1 s)"));
    Out.println(F("  safety            reset cause, last stop reaction time, watchdog"));
    Out.println(F("  wdtest            freeze on purpose to test the watchdog"));
    Out.println(F("  inputs            limit switches and encoder state"));
    Out.println(F("  watch on|off      print input events live"));
    Out.println(F("  led g|b|c|r|off [slow|fast]   set status LED"));
    Out.println(F("  drivers           driver detection, current, microsteps"));
    Out.println(F("  move <1-6> <amount> [speed]   mm & mm/s (translation) or deg & rpm (rotation)"));
    Out.println(F("  mv <axis>=<amount> ... [t=<s>] [v=<%>]   coordinated move, all finish together"));
    Out.println(F("  stop              stop all motors"));
    Out.println(F("  pos               positions of all axes"));
    Out.println(F("  en <1-6|all> / dis <1-6|all>  enable / disable motor drivers"));
    Out.println(F("  isr reset         reset the interrupt timing statistic"));
    Out.println(F("  chop <1-6> stealth|spread|hybrid [mm/s or rpm]   driver chopper mode"));
    Out.println(F("  ms <1-6> <2|4|8|16|32>   microstepping (position is kept)"));
    Out.println(F("  cur <1-6> <mA>    motor current (RMS)"));
  }

  // Parse "1".."6" -> 0..5, or -1
  int8_t parseAxis(const char* a) {
    if (!a) return -1;
    int n = atoi(a);
    return (n >= 1 && n <= (int)hw::NUM_AXES) ? n - 1 : -1;
  }

  void cmdMove(const char* a, const char* amt, const char* spd) {
    int8_t i = parseAxis(a);
    if (i < 0 || !amt) { Out.println(F("ERR usage: move <1-6> <amount> [speed]")); return; }
    motion::moveRelative(i, atof(amt), spd ? atof(spd) : 0);
  }

  // mv 2=30 4=30 6=30 [t=5] [v=50]
  void cmdMv(uint8_t argc, char** argv) {
    float amount[hw::NUM_AXES] = { 0 };
    uint8_t mask = 0;
    float pct = 100, dur = 0;
    for (uint8_t k = 1; k < argc; k++) {
      char* eq = strchr(argv[k], '=');
      if (!eq) { Out.print(F("ERR bad argument: ")); Out.println(argv[k]); return; }
      *eq = '\0';
      const char* key = argv[k]; float val = atof(eq + 1);
      if      (!strcmp(key, "t")) dur = val;
      else if (!strcmp(key, "v")) pct = val;
      else {
        int8_t i = parseAxis(key);
        if (i < 0) { Out.print(F("ERR bad axis: ")); Out.println(key); return; }
        amount[i] = val; mask |= _BV(i);
      }
    }
    if (!mask) { Out.println(F("ERR usage: mv <axis>=<amount> ... [t=<s>] [v=<%>]")); return; }
    motion::moveGroup(amount, mask, pct, dur);
  }

  void cmdPos() {
    Out.print(F("OK"));
    for (uint8_t i = 0; i < hw::NUM_AXES; i++) {
      Out.print(' '); Out.print(hw::AXES[i].name); Out.print('=');
      Out.print(motion::position(i), 2);
      if (motion::busy(i)) Out.print('*');
    }
    Out.println();
  }

  void cmdEnable(const char* a, bool on) {
    if (a && !strcmp(a, "all")) {
      if (!on) motion::stopAll(F("drivers disabled"));
      for (uint8_t i = 0; i < hw::NUM_AXES; i++) drivers::enable(i, on);
    } else {
      int8_t i = parseAxis(a);
      if (i < 0) { Out.println(F("ERR usage: en|dis <1-6|all>")); return; }
      if (!on && motion::busy(i)) motion::stopAll(F("driver disabled"));
      drivers::enable(i, on);
    }
    Out.println(on ? F("OK enabled") : F("OK disabled"));
  }

  void cmdChop(const char* a, const char* m, const char* thr) {
    int8_t i = parseAxis(a);
    if (i < 0 || !m) { Out.println(F("ERR usage: chop <1-6> stealth|spread|hybrid [speed]")); return; }
    drivers::Chopper c;
    if      (!strcmp(m, "stealth")) c = drivers::Chopper::Stealth;
    else if (!strcmp(m, "spread"))  c = drivers::Chopper::Spread;
    else if (!strcmp(m, "hybrid"))  c = drivers::Chopper::Hybrid;
    else { Out.println(F("ERR mode: stealth|spread|hybrid")); return; }
    float sps = 0;
    if (c == drivers::Chopper::Hybrid) {
      float v = thr ? atof(thr) : 5.0f;   // default switch-over: 5 mm/s or 5 rpm
      bool tra = hw::AXES[i].type == AxisType::Translation;
      sps = tra ? v * motion::stepsPerUnit(i) : v * 6.0f * motion::stepsPerUnit(i);
    }
    drivers::setChopper(i, c, sps);
    Out.println(F("OK chopper set"));
  }

  void cmdMs(const char* a, const char* v) {
    int8_t i = parseAxis(a);
    int ms = v ? atoi(v) : 0;
    if (i < 0 || !(ms == 2 || ms == 4 || ms == 8 || ms == 16 || ms == 32)) {
      Out.println(F("ERR usage: ms <1-6> <2|4|8|16|32>")); return; }
    if (motion::busy(i)) { Out.println(F("ERR axis busy")); return; }
    uint16_t old = drivers::microsteps(i);
    int32_t pos = steppers::position(i);
    drivers::setMicrosteps(i, ms);
    steppers::setPosition(i, (int32_t)((int64_t)pos * ms / old));   // same physical position
    Out.print(F("OK ms=")); Out.println(ms);
  }

  void cmdCur(const char* a, const char* v) {
    int8_t i = parseAxis(a);
    int mA = v ? atoi(v) : 0;
    if (i < 0 || mA < 100 || mA > 2000) { Out.println(F("ERR usage: cur <1-6> <100..2000>")); return; }
    drivers::setCurrent(i, mA);
    Out.print(F("OK current=")); Out.print(mA); Out.println(F("mA"));
  }

  void cmdStatus() {
    Out.print(F("OK fw="));       Out.print(FW_VERSION);
    Out.print(F(" profile=\""));  Out.print(hw::PROFILE_NAME);
    Out.print(F("\" state="));   Out.print(sys::name(sys::get()));
    Out.print(F(" loops/s="));    Out.print(loop_stats::loopsPerSecond());
    Out.print(F(" maxloop_us=")); Out.print(loop_stats::maxLoopUs());
    Out.print(F(" isr_max_us=")); Out.print(steppers::isrMaxUs(), 1);
    Out.print(F(" uptime_s="));   Out.print(millis() / 1000);
    Out.print(F(" st_skipped="));  Out.println(stSkipped);
  }

  void cmdInputs() {
    Out.print(F("OK"));
    for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++) {
      Out.print(' '); Out.print(hw::SWITCHES[i].name);
      Out.print('=');  Out.print(limit_switches::pressed(i) ? 1 : 0);
    }
    Out.print(F(" enc_pos=")); Out.print(encoder::position());
    Out.print(F(" enc_btn=")); Out.println(encoder::isDown() ? 1 : 0);
  }

  void cmdWatch(const char* arg) {
    if      (arg && !strcmp(arg, "on"))  watchOn = true;
    else if (arg && !strcmp(arg, "off")) watchOn = false;
    else { Out.println(F("ERR usage: watch on|off")); return; }
    Out.print(F("OK watch ")); Out.println(watchOn ? F("on") : F("off"));
  }

  void cmdLed(const char* c, const char* p) {
    using status_led::Color; using status_led::Pattern;
    Color col;
    if      (!c)                 { Out.println(F("ERR usage: led g|b|c|r|off [slow|fast]")); return; }
    else if (!strcmp(c, "g"))    col = Color::Green;
    else if (!strcmp(c, "b"))    col = Color::Blue;
    else if (!strcmp(c, "c"))    col = Color::Cyan;
    else if (!strcmp(c, "r"))    col = Color::Red;
    else if (!strcmp(c, "off"))  col = Color::Off;
    else { Out.println(F("ERR unknown colour")); return; }
    Pattern pat = Pattern::Solid;
    if (p && !strcmp(p, "slow")) pat = Pattern::SlowBlink;
    if (p && !strcmp(p, "fast")) pat = Pattern::FastBlink;
    status_led::set(col, pat);
    Out.print(F("OK led ")); Out.println(status_led::colorName(col));
  }

  // info: everything a laptop program needs to configure itself.
  // Same line on V1 and V2; only the values differ.
  void cmdInfo() {
    Out.print(F("OK info proto=")); Out.print(PROTO_VERSION);
    Out.print(F(" fw="));           Out.print(FW_VERSION);
    Out.print(F(" profile=\""));    Out.print(hw::PROFILE_NAME);
    Out.print(F("\" axes="));
    for (uint8_t i = 0; i < hw::NUM_AXES; i++) {
      if (i) Out.print(',');
      Out.print(hw::AXES[i].name); Out.print(':'); Out.print(motion::unit(i));
    }
    Out.print(F(" switches="));
    for (uint8_t s = 0; s < hw::NUM_SWITCHES; s++) {
      if (s) Out.print(',');
      Out.print(hw::SWITCHES[s].name);
    }
    Out.println();
  }

  // The fields shared by "st" (reply) and the stream (ST lines):
  //   t=<ms since start> state=<STATE> busy=<one 0/1 per axis> pos=<a1>,..,<a6>
  void printStatusFields() {
    Out.print(F("t="));       Out.print(millis());
    Out.print(F(" state="));  Out.print(sys::name(sys::get()));
    Out.print(F(" busy="));
    for (uint8_t i = 0; i < hw::NUM_AXES; i++) Out.print(motion::busy(i) ? '1' : '0');
    Out.print(F(" pos="));
    for (uint8_t i = 0; i < hw::NUM_AXES; i++) {
      if (i) Out.print(',');
      Out.print(motion::position(i), 2);
    }
    Out.println();
  }

  void cmdStream(const char* a) {
    if (!a) {
      Out.print(F("OK stream=")); Out.print(streamMs);
      Out.print(F(" st_skipped=")); Out.println(stSkipped);
      return;
    }
    long ms = strcmp(a, "off") ? atol(a) : 0;
    if (ms != 0 && (ms < 20 || ms > 10000)) { Out.println(F("ERR usage: stream <20..10000 ms>|off")); return; }
    streamMs = ms;
    lastStream = millis();
    Out.print(F("OK stream=")); Out.println(streamMs);
  }

  // Split the line into words and dispatch
  void dispatch(uint8_t argc, char** argv);

  void execute(char* s) {
    char* argv[MAX_ARGS] = { nullptr };
    uint8_t argc = 0;
    for (char* tok = strtok(s, " \t"); tok && argc < MAX_ARGS; tok = strtok(nullptr, " \t"))
      argv[argc++] = tok;
    if (argc == 0) return;

    // Optional request id: "#12 pos" -> every reply line starts with "#12 "
    int32_t id = -1;
    if (argv[0][0] == '#') {
      id = atol(argv[0] + 1);
      if (id < 0) id = -1;
      for (uint8_t k = 1; k < argc; k++) argv[k - 1] = argv[k];
      argv[--argc] = nullptr;
    }
    Out.beginReply(id);
    if (argc == 0) { Out.println(F("ERR empty command")); Out.endReply(); return; }
    dispatch(argc, argv);
    Out.endReply();
  }

  void dispatch(uint8_t argc, char** argv) {
    for (char* p = argv[0]; *p; p++) *p = tolower(*p);   // commands are case-insensitive

    if      (!strcmp(argv[0], "help"))   cmdHelp();
    else if (!strcmp(argv[0], "status")) cmdStatus();
    else if (!strcmp(argv[0], "info"))   cmdInfo();
    else if (!strcmp(argv[0], "ping"))   Out.println(F("OK pong"));
    else if (!strcmp(argv[0], "st"))     { Out.print(F("OK ")); printStatusFields(); }
    else if (!strcmp(argv[0], "stream")) cmdStream(argv[1]);
    else if (!strcmp(argv[0], "state"))  sys::report();
    else if (!strcmp(argv[0], "ack"))    { if (sys::acknowledge()) Out.println(F("OK acknowledged")); }
    else if (!strcmp(argv[0], "inputs")) cmdInputs();
    else if (!strcmp(argv[0], "watch"))  cmdWatch(argv[1]);
    else if (!strcmp(argv[0], "led"))    cmdLed(argv[1], argv[2]);
    else if (!strcmp(argv[0], "drivers")) drivers::report();
    else if (!strcmp(argv[0], "move"))   cmdMove(argv[1], argv[2], argv[3]);
    else if (!strcmp(argv[0], "mv"))     cmdMv(argc, argv);
    else if (!strcmp(argv[0], "stop"))   { safety::userStop(F("command"), micros()); Out.println(F("OK stop")); }
    else if (!strcmp(argv[0], "safety")) safety::report();
    else if (!strcmp(argv[0], "wdtest")) safety::watchdogTest();
    else if (!strcmp(argv[0], "pos"))    cmdPos();
    else if (!strcmp(argv[0], "en"))     cmdEnable(argv[1], true);
    else if (!strcmp(argv[0], "dis"))    cmdEnable(argv[1], false);
    else if (!strcmp(argv[0], "chop"))   cmdChop(argv[1], argv[2], argv[3]);
    else if (!strcmp(argv[0], "ms"))     cmdMs(argv[1], argv[2]);
    else if (!strcmp(argv[0], "cur"))    cmdCur(argv[1], argv[2]);
    else if (!strcmp(argv[0], "isr"))    { steppers::resetIsrStats(); Out.println(F("OK isr stats reset")); }
    else { Out.print(F("ERR unknown command: ")); Out.println(argv[0]); }
  }
}

void begin() { len = 0; overflow = false; }

void update() {
  // Handle at most what has already arrived; never wait for more.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (overflow)   Out.println(F("ERR line too long"));
      else if (len)   { line[len] = '\0'; execute(line); }
      len = 0; overflow = false;
    } else if (len < LINE_MAX - 1) {
      line[len++] = c;
    } else {
      overflow = true;
    }
  }
  updateStream();
}

bool watching() { return watchOn; }

void updateStream() {
  if (!streamMs) return;
  uint32_t now = millis();
  if (now - lastStream < streamMs) return;
  lastStream = now;
  if (Out.room() < ST_ROOM) { stSkipped++; return; }   // never wait for USB
  Out.print(F("ST ")); printStatusFields();
}
}  // namespace console
