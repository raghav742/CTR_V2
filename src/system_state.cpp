#include "system_state.h"
#include "serial_io.h"
#include "hw_profile.h"
#include "status_led.h"
#include "motion.h"

namespace sys {
namespace {
  State    cur = State::Boot;
  bool     isHomed = false;
  char     reason[40] = "";

  // Allowed transitions: bit (1 << to) set in ALLOWED[from]
  constexpr uint16_t B(State s) { return 1u << (uint8_t)s; }
  const uint16_t ALLOWED[] = {
    /* Boot    */ B(State::Idle),
    /* Idle    */ B(State::Homing) | B(State::Remote),
    /* Homing  */ B(State::Ready)  | B(State::Idle),
    /* Ready   */ B(State::Homing) | B(State::Manual) | B(State::Demo) | B(State::Attract)
                | B(State::Visitor) | B(State::Remote) | B(State::Idle),
    /* Manual  */ B(State::Ready),
    /* Demo    */ B(State::Ready),
    /* Attract */ B(State::Ready),
    /* Visitor */ B(State::Ready),
    /* Remote  */ B(State::Ready)  | B(State::Idle),
    /* Fault   */ B(State::Idle)   | B(State::Ready),
  };

  void showLed(State s) {
    using status_led::Color; using status_led::Pattern;
    switch (s) {
      case State::Boot:    status_led::set(Color::Cyan); break;
      case State::Idle:    status_led::set(Color::Green, Pattern::SlowBlink); break;
      case State::Homing:  status_led::set(Color::Cyan,  Pattern::SlowBlink); break;
      case State::Ready:   status_led::set(Color::Green); break;
      case State::Manual:  status_led::set(Color::Blue); break;
      case State::Demo:
      case State::Attract:
      case State::Visitor: status_led::set(Color::Blue, Pattern::SlowBlink); break;
      case State::Remote:  status_led::set(Color::Cyan); break;
      case State::Fault:   status_led::set(Color::Red,  Pattern::FastBlink); break;  // cyan on V1
    }
  }

  void enter(State to, const char* why) {
    State from = cur;
    cur = to;
    showLed(to);
    Out.print(F("EV state ")); Out.print(name(from));
    Out.print(F(" -> "));      Out.print(name(to));
    if (why && *why) { Out.print(F(" (")); Out.print(why); Out.print(')'); }
    Out.println();
  }
}

const __FlashStringHelper* name(State s) {
  switch (s) {
    case State::Boot:    return F("BOOT");
    case State::Idle:    return F("IDLE");
    case State::Homing:  return F("HOMING");
    case State::Ready:   return F("READY");
    case State::Manual:  return F("MANUAL");
    case State::Demo:    return F("DEMO");
    case State::Attract: return F("ATTRACT");
    case State::Visitor: return F("VISITOR");
    case State::Remote:  return F("REMOTE");
    case State::Fault:   return F("FAULT");
  }
  return F("?");
}

void  begin() { cur = State::Boot; showLed(cur); }
State get()   { return cur; }
bool  homed() { return isHomed; }
void  setHomed(bool h) { isHomed = h; }
const char* faultReason() { return reason; }

bool request(State to) {
  if (to == cur) return true;
  if (to == State::Fault) { fault("requested"); return true; }
  if (!(ALLOWED[(uint8_t)cur] & B(to))) {
    Out.print(F("ERR not allowed: ")); Out.print(name(cur));
    Out.print(F(" -> ")); Out.println(name(to));
    return false;
  }
  if ((to == State::Ready) && !isHomed) {
    Out.println(F("ERR not homed")); return false;
  }
  enter(to, nullptr);
  return true;
}

void fault(const char* why) {
  motion::stopAll(F("fault"));
  strncpy(reason, why ? why : "", sizeof(reason) - 1);
  reason[sizeof(reason) - 1] = '\0';
  if (cur != State::Fault) enter(State::Fault, reason);
}

bool acknowledge() {
  if (cur != State::Fault) { Out.println(F("ERR no fault active")); return false; }
  reason[0] = '\0';
  enter(isHomed ? State::Ready : State::Idle, "acknowledged");
  return true;
}

bool motionAllowed() {
  switch (cur) {
    case State::Idle: case State::Ready: case State::Manual:
    case State::Demo: case State::Attract: case State::Visitor:
    case State::Remote: case State::Homing:
      return true;
    default:
      return false;   // Boot, Fault
  }
}

void update() {
  if (cur == State::Boot) { enter(State::Idle, "startup complete"); return; }

  // (driver health and limit switches are watched by the safety manager)
}

void report() {
  Out.print(F("OK state=")); Out.print(name(cur));
  Out.print(F(" homed="));   Out.print(isHomed ? 1 : 0);
  if (cur == State::Fault) { Out.print(F(" fault=\"")); Out.print(reason); Out.print('"'); }
  Out.println();
}
}  // namespace sys
