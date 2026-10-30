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

// Homing (step 2.1)
// Two-speed approach (like CNC machines): fast until the switch clicks, back off
// HOME_REAPPROACH_MM, then slowly onto the switch again. Only the slow approach
// sets the position, so the fast one can be quick without losing accuracy.
constexpr float HOME_FAST_SPEED    = 10.0f;   // mm/s, first approach (V1: lower = less slip)
constexpr float HOME_SPEED         = 5.0f;    // mm/s, final approach and clearing the switch
constexpr float HOME_REAPPROACH_MM = 2.0f;    // back-off before the slow approach
constexpr float HOME_SEARCH_MM     = 1000.0f; // practically unlimited: only catches a broken switch
                                              // or cable (no carriage can travel 1 m)
constexpr float HOME_CLEAR_MM   = 10.0f;   // max forward move to release a pressed switch
constexpr float HOME_BACKOFF_MM = 1.0f;    // extra distance after the switch releases
// Lowest position a homed translation axis may be sent to. At 0 the switch is
// pressed (zero = the press point); it releases a little further forward
// (printed by homing as "released at"). Must be above that release point.
constexpr float TRA_SOFT_MIN_MM = 1.0f;

// Calibration of the front limits (step 2.2) and limit margin (step 2.3)
constexpr float CALIB_CONTACT_FAST  = 8.0f;   // mm/s, first approach of one carriage onto the next
constexpr float CALIB_CONTACT_SPEED = 3.0f;   // mm/s, final approach (sets the gap)
constexpr float CALIB_MOVE_SPEED    = 10.0f;  // mm/s, parking and return moves (V1: lower = less slip)
constexpr float CALIB_MAX_DEVIATION_MM = 5.0f;
constexpr float PARK_SPEED_PCT      = 50.0f;
constexpr float DEMO_SPEED_PCT      = 70.0f;  // demos (step 3.4): % of the maximum speed (V1: slip)  // "park" (step 2.4): % of the maximum speed // reject a calib value this far from the profile value
constexpr float CALIB_REST_MM       = 2.0f;   // where calib leaves each carriage at the end
constexpr float CALIB_PARK_MM       = 50.0f;  // before the 3rd carriage (T3) is measured, all carriages
                                              // go here together, so T3 touches T2 at ~50 + gap
                                              // (~400 mm) instead of at its full extension (~615 mm,
                                              // where it disturbed switches 39 and 42 on 2026-10-30)
constexpr float CALIB_SEPARATE_MM   = 5.0f;   // after a measurement, back off this far before
                                              // the next carriage approaches (absorbs pushes)
constexpr float LIMIT_MARGIN_MM     = 2.0f;   // stay this far from the far switch / the carriage in front
// Span rule: a carriage may not get further ahead of the carriage behind it than
// at home (the tube between them hangs unsupported and buckles). In positions:
// front <= back + LIMIT_SPAN_TOL_MM. The tolerance covers the small differences
// homing leaves (each carriage rests 1.3-1.4 mm off its switch).
constexpr float LIMIT_SPAN_TOL_MM   = 1.0f;
// The span rule is needed for the long automatic moves (homing and calib move the
// carriages together by design). Manual moves are short and may move each tube
// individually (Raghav, 2026-10-30): false = span rule not applied to normal moves.
constexpr bool  SPAN_RULE_FOR_MOVES = false;

// Driver health check (safety): one driver per interval, round-robin, so each
// driver is checked every 6 x interval. One check = 2 UART reads = ~9 ms of
// blocked loop (measured 2026-10-28, TMCStepper waits inside each read).
// Decision D7: 1 s (was 200 ms) -> the 9 ms pause happens once a second.
// The TMC2209 also shuts itself down on over-temperature (hardware).
constexpr uint16_t DRV_CHECK_MS = 1000;
}  // namespace motion_cfg
