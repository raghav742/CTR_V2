#pragma once
// ============================================================
//  Motion tuning (not hardware): speeds and accelerations.
//  Change here while testing; same for V1 and V2 unless retuned.
// ============================================================

namespace motion_cfg {
// Step timer
constexpr uint32_t STEP_TICK_HZ = 20000;    // Timer1 interrupt rate = max steps/s per axis

// Translation axes (mm/s, mm/s^2)
constexpr float TRA_SPEED_DEFAULT = 10.0f;
constexpr float TRA_SPEED_MAX     = 20.0f;   // requirement FRT16/26/36
constexpr float TRA_ACCEL         = 40.0f;   // 20 mm/s reached in 0.5 s
constexpr float TRA_SPEED_MIN     = 0.5f;    // start/stop speed

// Rotation axes (rpm, rpm/s)
constexpr float ROT_SPEED_DEFAULT = 30.0f;
constexpr float ROT_SPEED_MAX     = 120.0f;
constexpr float ROT_ACCEL         = 120.0f;  // rpm per second
constexpr float ROT_SPEED_MIN     = 2.0f;

// Driver chopper mode at startup (decision D4, measured 2026-10-28):
// StealthChop loses steps on translation, even at 5 mm/s; SpreadCycle loses none
// at 3-20 mm/s (repeatability against home switch within 0.1 mm).
//   0 = StealthChop (quiet), 1 = SpreadCycle (torque), 2 = hybrid (quiet below threshold)
constexpr uint8_t TRA_CHOPPER     = 1;
constexpr float   TRA_HYBRID_MMPS = 5.0f;   // only used when TRA_CHOPPER = 2
constexpr uint8_t ROT_CHOPPER     = 0;      // rotation runs fine in StealthChop

// Until homing exists (phase 2) positions are only counted steps, so a single
// translation move is capped to keep a typo from driving a carriage into the end.
constexpr float TRA_MAX_MOVE_UNHOMED = 50.0f;   // mm

// Ramp recalculation interval in the main loop
constexpr uint16_t RAMP_UPDATE_US = 1000;
}  // namespace motion_cfg
