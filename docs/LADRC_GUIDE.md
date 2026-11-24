# St4rpony Synchronized INDI + 4-Parameter Flight Controller Guide

## 1. Overview
St4rpony uses **Synchronized Incremental Nonlinear Dynamic Inversion (INDI) + LADRC** as its sole flight-control law across multirotors, hybrids, and fixed-wing aircraft, while preserving Betaflight's STM32 hardware drivers, bidirectional DShot RPM filtering, dynamic notch filtering, and unified board targets.

In default **4-Parameter Auto INDI** mode (`indi_manual_override = OFF`), the pilot specifies four physical craft parameters per profile and the firmware automatically derives control bandwidths (`wc`), control effectiveness (`b0`), tracking differentiator cutoffs (`td_hz`), synchronized INDI low-pass cutoffs (`f_indi`), DSP filter floors/notches, outer-loop angle gain (`0.25 * wc_roll`), and PID loop rates (`4000 Hz` for `< 10"` props, `1600 Hz` for `>= 10"` props).

## 2. Synchronized INDI Law & Upstream Phase Sync
For each axis (`FD_ROLL`, `FD_PITCH`, `FD_YAW`), `ladrcStepAxisEx()` executes:
1. **Setpoint Tracking Differentiator (ZOH 2nd-order critically damped)**:
   - `tdOmega = 2 * pi * td_hz * 1.553773974` (exact `-3 dB` cutoff alignment), bypassed (`tdWeight = 0`) when closed-loop outer attitude controllers or crash recovery are active.
2. **Twin Phase-Matched 2nd-Order Butterworth SVF Biquad LPFs (`f_indi`)**:
   - Gyro rate `y` passes through `gyroFilter` via `svfLowpassFilterApplyWithBandpass()`, producing filtered rate `omega_filt` and exact cancellation-free TPT-SVF bandpass angular acceleration `omega_dot_meas = f * (v1[k] + v1[k-1]) / dt`.
   - Actuator feedback `u_feedback` (either KV-free DShot eRPM or fallback slew+PT1 actuator model) passes through an optional upstream phase-sync PT1 (`feedbackSyncPt1`, matching upstream MEMS AAF + gyro LPF1/LPF2 + notch group delay minus actuator pipe delay) followed by an identical `feedbackFilter` at `f_indi`.
3. **Incremental Control Law**:
   - `u_P = wc_eff * (r - omega_filt) / b0`
   - `u_D = (-omega_dot_meas * ground_scale) / b0`
   - `u_F = r_dot / b0`
   - `u_cmd = clamp(u_meas_filt + u_P + u_D + u_F, -sum_limit, +sum_limit)`
   - All floating-point paths use `-ffast-math`-safe IEEE-754 bit inspection (`floatIsFiniteBits`, `constrainf`) to eliminate NaN/Inf propagation.
4. **Live Reconfiguration & Safety**:
   - Armed parameter updates call `ladrcReconfigureLive()` (updating gains and filter coefficients bumplessly without resetting observer/filter state).
   - Disarmed/CLI/MSP changes call `indiRefreshRuntimeRatesAndFilters()` to resynchronize gyro loop time, DShot telemetry, DSP filters, LADRC, mixer, servos, and RTOS scheduler tasks.
   - Pre-liftoff ground mode (`!airborneState`) scales `wc`/`u_D` by `ground_wc_pct` and clamps `u_meas_filt` to `[-0.12, 0.12]`; liftoff triggers at `throttle >= 0.20` or sustained `throttle >= 0.08 && gyroPeak >= 50 dps` for `50 ms`.

## 3. 4-Parameter Physical Scaling Laws
With clamped inputs `D = clamp(prop_size_inches, 3.0, 30.0)`, `m = clamp(all_up_weight_kg, 0.20, 50.0)`, `A = clamp(agility, 0.50, 1.50)`, `T_auth = clamp(authority_trim, 0.50, 1.50)`:
- **Synchronized INDI cutoff**: `f_indi = clamp(450 / D, 18, 100) Hz` (capped at `25 Hz` on fixed-wing).
- **RPM filter floor**: `f_rpm_min = clamp(500 / D, 20, 100) Hz`.
- **Dynamic notch range**: `min = clamp(550 / D, 25, 220) Hz`, `max = clamp(2500 / D, 120, 600) Hz` (auto-disabled below `2 kHz` when DShot RPM filter is active).
- **Bandwidths**: `wc_roll = clamp(18 * sqrt(5 / D) * A, 4, 30) rad/s`, `wc_pitch = 0.95 * wc_roll`, `wc_yaw = 0.70 * wc_roll`, `wo = 4 * wc`.
- **TD cutoffs**: `td_rp = clamp(60 * sqrt(5 / D) * A, 15, 100) Hz`, `td_yaw = 0.75 * td_rp`.
- **Physical `b0` law**:
  - `m_ref = 0.65 * (D / 5)^2.2`
  - `load_ratio = clamp(m / m_ref, 0.35, 4.0)`
  - `b0_calc = clamp(42 * (5 / D)^0.85 / sqrt(load_ratio), 5, 150)`
  - `b0_roll = b0_calc * T_auth`, `b0_pitch = 0.92 * b0_roll`, `b0_yaw = 0.30 * b0_roll`.
- **Actuator lag**: `tau_act = fixedWing ? 0.030 s : clamp(0.015 * (D / 5)^1.3, 0.010, 0.180) s`.

## 4. KV-Free DShot eRPM Feedback, Priority Mixing & Fixed-Wing
- **DShot Freshness Watchdog**: `updateDshotTelemetry(currentTimeUs)` timestamps valid eRPM frames; `isDshotMotorTelemetryFresh(i, currentTimeUs)` enforces a `25 ms` freshness window and clears stale GCR `rawValue` entries.
- **KV-Free Tangent Normalization**: Using ESC idle duty `d_idle`, linearized duty `c_i`, `d_i = d_idle + (1 - d_idle) * c_i`, `q_i = erpm_i^2`, and thrust-linearization slope `s_tl = dc/dm`, realized motor deviations `deltaM[i] = mixSign * (meanD / (2 * (1 - d_idle) * s_tl * meanQ)) * (q_i - meanQ)` are projected back to `[Roll, Pitch, Yaw]` via 4x4 Gauss-Jordan regularized pseudo-inverse `(M^T M + 1e-5 I)^-1 M^T m`.
- **Throttle-Aware Priority Mixing (`MIXER_LEGACY`)**: `indiAllocatePriorityMixWithThrottle()` preserves strict `Roll > Pitch > Yaw` authority inside `[0, 1]` motor limits without arbitrary low-throttle attenuation when headroom exists, caps airmode low-throttle yaw collective lift to `+0.10`, and prevents yaw saturation from pulling down climb throttle above `0.50`.
- **Fixed-Wing & Hybrid Axes**: Axes with zero motor column norm (or all axes when `isFixedWing()`) use realized servo travel (`servoGetRealizedAxisNorm()`), BEC-independent `b0` airspeed scheduling (`airspeedCms > 1.0`), acro yaw washout (`exp(-2 * dt)`), and coordinated turn feed-forward in level modes.

## 5. CLI Parameters
- 4-Parameter Auto INDI:
  - `prop_size_inches` (`3.0`–`30.0`, default `5.0`)
  - `all_up_weight_kg` (`0.20`–`50.00`, default `0.65`)
  - `agility` (`0.50`–`1.50`, default `1.00`)
  - `authority_trim` (`0.50`–`1.50`, default `1.00`)
  - `indi_manual_override` (`OFF`/`ON`, default `OFF`)
- Safety & Advanced:
  - `ladrc_anti_windup` (`ON`/`OFF`, default `ON`)
  - `ladrc_sat_z3_inhibit` (`ON`/`OFF`, default `ON`)
  - `ladrc_ground_wc_pct` (`0`–`100`, default `25`)
  - `ladrc_vbat_b0_comp` (`ON`/`OFF`, default `ON`)
  - `ladrc_airspeed_sched` (`ON`/`OFF`, default `OFF`)
  - `ladrc_airspeed_ref_cms` (`300`–`10000`, default `1500`)
  - `ladrc_disturbance_limit` (`0`–`1000`, default `500`)
  - Per-axis (`roll`, `pitch`, `yaw`): `ladrc_<axis>_order`, `ladrc_<axis>_wc`, `ladrc_<axis>_wo`, `ladrc_<axis>_b0`, `ladrc_<axis>_td_hz`.

## 6. Building & Testing
```bash
make configs
make arm_sdk_install
make -j$(nproc) CONFIG=SPEEDYBEEF405V3
make -j$(nproc) CONFIG=SPEEDYBEEF405V4
make -j$(nproc) CONFIG=SPEEDYBEEF405WING
make -j$(nproc) CONFIG=MAMBAF405US_I2C
make -j$(nproc) TARGET=SITL
```
Host unit tests:
```bash
make CC=gcc CXX=g++ -C src/test test_ladrc_unittest test_pid_unittest test_blackbox_unittest test_osd_unittest test_cli_unittest test_motor_output_unittest test_companion_link_unittest
```
