#include "attract.h"
#include "serial_io.h"
#include "motion_config.h"
#include "demo.h"
#include "motion.h"
#include "limits.h"
#include "console.h"
#include "ui.h"
#include "system_state.h"

namespace attract {
namespace {
  bool     on = true;
  bool     ours = false;            // current demo was started here
  uint8_t  next = 0;
  uint32_t lastBusy = 0;
}

void begin() { lastBusy = millis(); }
bool enabled() { return on; }
bool running() { return ours && demo::active(); }

void setEnabled(bool e) {
  on = e;
  if (!e && running()) demo::stop();
}

void update() {
  uint32_t now = millis();
  if (ours && !demo::active()) ours = false;
  if (motion::anyBusy() || demo::active()) { lastBusy = now; return; }
  if (!on || sys::get() != sys::State::Ready || !limits::active()) return;

  // idle = no motion, no encoder input, no console command for ATTRACT_IDLE_MS
  uint32_t last = lastBusy;
  if ((int32_t)(ui::lastInputMs() - last) > 0) last = ui::lastInputMs();
  if ((int32_t)(console::lastCommandMs() - last) > 0) last = console::lastCommandMs();
  if (now - last < motion_cfg::ATTRACT_IDLE_MS) return;

  Out.println(F("EV attract start"));
  if (demo::start(next, true)) ours = true;
  next = (next + 1) % demo::count();
  lastBusy = now;
}
}  // namespace attract
