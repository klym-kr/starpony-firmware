# St4rpony LADRC Flight Controller Guide

## 1. Overview
St4rpony uses **Linear Active Disturbance Rejection Control (LADRC)** as its sole flight-control law (parameterized by bandwidth `wc`, `wo`, `b0` per axis), replacing classical PID + heuristic compensators (I-term relax, anti-gravity, D_max), while preserving Betaflight's battle-tested STM32 hardware drivers, DShot/RPM filtering, dynamic notch, and unified board defaults.

## 2. Discrete Pole-Placed Current-Observer Math
Each axis runs a split **measurement-update -> mixer -> time-update** Linear Extended State Observer (LESO) with poles placed exactly at $z = \lambda = e^{-\omega_o T_s}$:
- **Order 1 (2-state LESO, default for Yaw)**: $\dot{y} = f + b_0 u$
- **Order 2 (3-state LESO, default for Roll/Pitch)**: $\ddot{y} = f + b_0 u$ (models ESC/motor/prop phase lag and provides observer-smoothed acceleration damping $\hat{z}_2$ without differentiating raw gyro noise).

After `mixTable()` clamps and normalizes motor outputs, `ladrcPostMixerUpdate()` feeds back `u_real = u_cmd * mixerGetAuthorityScale()` into the LESO prediction step to prevent disturbance windup during motor saturation.

## 3. CLI & MSPv2 Parameters
- `ladrc_anti_windup` (`ON`/`OFF`, default `ON`)
- `ladrc_vbat_b0_comp` (`ON`/`OFF`, default `ON`)
- `ladrc_airspeed_sched` (`ON`/`OFF`, default `OFF`, Phase 2 fixed-wing)
- `ladrc_airspeed_ref_cms` (default `1500` = `15.0 m/s`)
- `ladrc_disturbance_limit` (default `400` in `pidSum` units, max `1000`)
- Per-axis (`roll`, `pitch`, `yaw`):
  - `ladrc_<axis>_order` (`1` or `2`)
  - `ladrc_<axis>_wc` (controller bandwidth in `0.1 rad/s`)
  - `ladrc_<axis>_wo` (observer bandwidth in `0.1 rad/s`, typically `3x–5x wc`)
  - `ladrc_<axis>_b0` (nominal control gain in `0.1` normalized units)
  - `ladrc_<axis>_td_hz` (setpoint tracking differentiator cutoff Hz, `0` = off)

## 4. Building Targets
```bash
make configs
make arm_sdk_install
make -j$(nproc) CONFIG=SPEEDYBEEF405V3
make -j$(nproc) CONFIG=SPEEDYBEEF405V4
make -j$(nproc) CONFIG=SPEEDYBEEF405V3
```
Host unit tests:
```bash
make -C src/test CC=gcc CXX=g++ test_pid_unittest test_ladrc_unittest test_companion_link_unittest
```
