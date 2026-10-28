#include "steppers.h"
#include <util/atomic.h>
#include "hw_profile.h"
#include "motion_config.h"

// ============================================================
//  Data shared with the interrupt. Kept as separate small arrays
//  (not a struct) and 16-bit where possible: the ATmega2560 is an
//  8-bit CPU, so every byte saved is time saved in the interrupt.
// ============================================================
namespace steppers {
namespace {
  constexpr uint8_t N = hw::NUM_AXES;
  static_assert(N == 6, "the unrolled interrupt below handles exactly 6 axes");

  // Written by main loop, read by interrupt
  volatile uint16_t sInc[N];          // added to acc every tick; overflow = one step
  volatile int8_t   sDir[N];          // +1 / -1
  volatile uint8_t  sActive = 0;      // bit i = axis i moving
  // Owned by interrupt (main loop reads them with interrupts off)
  volatile int32_t  sPos[N];          // microsteps
  volatile uint32_t sRem[N];          // microsteps left in this move
  uint16_t          sAcc[N];          // interrupt-only
  uint8_t           sPulsed = 0;      // interrupt-only: STEP pins left HIGH last tick
  volatile uint16_t isrMaxTicks = 0;  // Timer1 counts (0.5 us each)

  // STEP pin output registers, looked up once at startup
  volatile uint8_t* sPort[N];
  uint8_t           sMask[N];

  constexpr uint16_t TIMER_PRESCALER = 8;                       // 16 MHz / 8 = 2 MHz
  constexpr uint16_t OCR_VALUE = (F_CPU / TIMER_PRESCALER) / motion_cfg::STEP_TICK_HZ - 1;
  constexpr float    INC_PER_STEP_PER_SEC = 65536.0f / motion_cfg::STEP_TICK_HZ;
}

void begin() {
  for (uint8_t i = 0; i < N; i++) {
    uint8_t pin = hw::AXES[i].pinStep;
    sPort[i] = portOutputRegister(digitalPinToPort(pin));
    sMask[i] = digitalPinToBitMask(pin);
    sInc[i] = 0; sAcc[i] = 0; sPos[i] = 0; sRem[i] = 0; sDir[i] = 1;
  }
  sActive = 0; sPulsed = 0;
  // Timer1: CTC mode, prescaler 8, interrupt at STEP_TICK_HZ
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS11);
  TCNT1  = 0;
  OCR1A  = OCR_VALUE;
  TIMSK1 |= _BV(OCIE1A);
  interrupts();
}

void start(uint8_t i, int32_t target) {
  if (i >= N) return;
  int32_t p = position(i);
  if (target == p) return;
  bool forward = target > p;
  // DIR first, then arm the axis; the first step comes on a later tick
  bool level = forward != hw::AXES[i].dirInvert;
  digitalWrite(hw::AXES[i].pinDir, level ? HIGH : LOW);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    sDir[i] = forward ? 1 : -1;
    sRem[i] = forward ? (uint32_t)(target - p) : (uint32_t)(p - target);
    sAcc[i] = 0;
    sActive |= _BV(i);
  }
}

void setRate(uint8_t i, float sps) {
  if (i >= N) return;
  if (sps < 0) sps = 0;
  float inc = sps * INC_PER_STEP_PER_SEC;
  if (inc > 65535.0f) inc = 65535.0f;          // = STEP_TICK_HZ steps/s, the maximum
  uint16_t v = (uint16_t)inc;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { sInc[i] = v; }
}

void setIncrement(uint8_t i, uint16_t inc) {
  if (i >= N) return;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { sInc[i] = inc; }
}

void stop(uint8_t i) {
  if (i >= N) return;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { sActive &= ~_BV(i); sInc[i] = 0; sRem[i] = 0; }
}

void stopAll() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    sActive = 0;
    for (uint8_t i = 0; i < N; i++) { sInc[i] = 0; sRem[i] = 0; }
  }
}

bool running(uint8_t i) { return i < N && (sActive & _BV(i)); }

int32_t position(uint8_t i) {
  int32_t p;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { p = sPos[i]; }
  return p;
}

int32_t remaining(uint8_t i) {
  uint32_t r;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { r = sRem[i]; }
  return (int32_t)r;
}

void setPosition(uint8_t i, int32_t p) {
  if (i >= N || running(i)) return;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { sPos[i] = p; }
}

float isrMaxUs() {
  uint16_t t;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = isrMaxTicks; }
  return t * (float)TIMER_PRESCALER / (F_CPU / 1000000.0f);
}

void resetIsrStats() { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { isrMaxTicks = 0; } }

// ------------------------------------------------------------
//  One axis, unrolled at compile time (I is a constant, so all
//  array addresses are fixed: no index arithmetic at run time).
// ------------------------------------------------------------
template <uint8_t I>
static inline __attribute__((always_inline)) void tickAxis(uint8_t active) {
  if (!(active & _BV(I))) return;
  uint16_t before = sAcc[I];
  uint16_t after  = before + sInc[I];
  sAcc[I] = after;
  if (after < before) {                       // accumulator overflow -> one step
    *sPort[I] |= sMask[I];
    sPulsed |= _BV(I);
    sPos[I] += sDir[I];
    if (--sRem[I] == 0) sActive &= ~_BV(I);   // move complete
  }
}

template <uint8_t I>
static inline __attribute__((always_inline)) void endPulse(uint8_t pulsed) {
  if (pulsed & _BV(I)) *sPort[I] &= ~sMask[I];
}

void isrBody() {
  // End last tick's pulses (pulse width = one tick = 50 us; TMC2209 needs > 0.1 us)
  uint8_t pulsed = sPulsed;
  if (pulsed) {
    endPulse<0>(pulsed); endPulse<1>(pulsed); endPulse<2>(pulsed);
    endPulse<3>(pulsed); endPulse<4>(pulsed); endPulse<5>(pulsed);
    sPulsed = 0;
  }
  uint8_t active = sActive;
  if (active) {
    tickAxis<0>(active); tickAxis<1>(active); tickAxis<2>(active);
    tickAxis<3>(active); tickAxis<4>(active); tickAxis<5>(active);
  }
  // Timer1 restarts at 0 every tick, so TCNT1 = time since this tick began
  uint16_t t = TCNT1;
  if (t > isrMaxTicks) isrMaxTicks = t;
}
}  // namespace steppers

// The step interrupt: 20,000 times per second.
ISR(TIMER1_COMPA_vect) { steppers::isrBody(); }
