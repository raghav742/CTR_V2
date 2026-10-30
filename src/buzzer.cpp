#include "buzzer.h"
#include "hw_profile.h"
#include "system_state.h"

namespace buzzer {
namespace {
  struct Note { uint16_t freq; uint16_t dur; };   // freq 0 = pause
  // Melodies from Casper's V1 firmware (Interface.ino)
  const Note MEL_CLICK[] PROGMEM = { {1800, 30} };
  const Note MEL_BEEP[]  PROGMEM = { {1200, 100} };
  const Note MEL_OK[]    PROGMEM = { {1047, 80}, {1319, 80}, {1568, 150} };
  const Note MEL_BOOT[]  PROGMEM = { {523, 80}, {659, 80}, {784, 80}, {1047, 180} };
  const Note MEL_ERROR[] PROGMEM = { {400, 300}, {0, 80}, {300, 400} };
  const Note MEL_HOME[]  PROGMEM = { {784, 80}, {659, 80}, {523, 150} };

  const Note* mel = nullptr;
  uint8_t  len = 0, idx = 0;
  uint32_t noteStart = 0;
  bool     isMuted = false;
  sys::State prevState = sys::State::Boot;

  Note noteAt(uint8_t i) { Note n; memcpy_P(&n, &mel[i], sizeof(Note)); return n; }
  void sound(uint16_t f) { if (f && !isMuted) tone(hw::PIN_BUZZER, f); else noTone(hw::PIN_BUZZER); }
}

void begin() { pinMode(hw::PIN_BUZZER, OUTPUT); noTone(hw::PIN_BUZZER); }

void play(Sound s) {
  switch (s) {
    case Sound::Click: mel = MEL_CLICK; len = 1; break;
    case Sound::Beep:  mel = MEL_BEEP;  len = 1; break;
    case Sound::Ok:    mel = MEL_OK;    len = 3; break;
    case Sound::Boot:  mel = MEL_BOOT;  len = 4; break;
    case Sound::Error: mel = MEL_ERROR; len = 3; break;
    case Sound::Home:  mel = MEL_HOME;  len = 3; break;
  }
  idx = 0;
  noteStart = millis();
  sound(noteAt(0).freq);
}

void update() {
  // Sounds for state changes (so other modules do not need to know the buzzer)
  sys::State st = sys::get();
  if (st != prevState) {
    if (st == sys::State::Fault) play(Sound::Error);
    else if (prevState == sys::State::Homing && st == sys::State::Ready) play(Sound::Home);
    prevState = st;
  }

  if (!mel) return;
  if (millis() - noteStart < noteAt(idx).dur) return;
  if (++idx >= len) { mel = nullptr; noTone(hw::PIN_BUZZER); return; }
  noteStart = millis();
  sound(noteAt(idx).freq);
}

void setMuted(bool m) { isMuted = m; if (m) noTone(hw::PIN_BUZZER); }
bool muted() { return isMuted; }
}  // namespace buzzer
