# St4rpony Companion PC Link & MSPv2 Protocol Specification

Status: **Synchronized INDI + Offboard Actuation & Telemetry** (`RATE_THRUST` / `ATTITUDE_THRUST` wired into `rc.c`, `pid.c`, `mixer.c`, `core.c`; 4-Param INDI config/state exposed over MSPv2 `0x3500..0x3512`).

## 1. Architecture Philosophy
- **STM32 Flight Controller (`st4rpony-firmware`)**: Hard real-time inner loops only — gyro anti-aliasing & notch filtering, attitude estimation, Synchronized INDI/LADRC rate/angle loops, KV-free DShot eRPM / servo actuator feedback, priority mixing, RC RX & failsafes.
- **Companion PC (Raspberry Pi 4/5, Orange Pi 5)**: High-level autonomy — VIO/SLAM, obstacle avoidance, mission planning, trajectory generation, telemetry logging.

## 2. Physical Connection (Recommended)
- **Primary In-Flight Link**: Dedicated 3.3V TTL Hardware UART (`TX` -> `RX`, `RX` -> `TX`, shared `GND`) with Circular DMA RX + IDLE-line interrupt and DMA TX.
  - On `SPEEDYBEEF405V3`, `UART3` (`PC10`/`PC11`) or `UART6` (`PC6`/`PC7`) are ideal when free; `UART4` is wired to the onboard Bluetooth module.
  - Recommended baud rate: `921600` bps (default) up to `2000000` bps.
- **Secondary Bench Link**: USB-CDC Virtual COM port (12 Mbps Full Speed) for desktop HITL/SITL.

## 3. MSPv2 Extension Layout (`0x3500..0x351F`)
St4rpony uses **MSPv2** (`$X<` request / `$X>` response / `$X!` error, CRC8-DVB-S2) defined in `src/main/msp/msp_protocol_v2_st4rpony.h`.

### 3.1 INDI / LADRC Configuration & Telemetry (`0x3500..0x3504`)
- **`MSP2_ST4R_GET_LADRC_CONFIG` (`0x3500`)**: Optional `u8 profileIndex`. Reply (44 bytes): `u8 profileIndex`, `u8 anti_windup`, `u8 vbat_b0_comp`, `u8 airspeed_sched`, `u16 airspeed_ref_cms`, `u16 disturbance_limit`, 3x axis tuples (`u8 order`, `u16 wc_deci`, `u16 wo_deci`, `u16 b0_deci`, `u16 td_hz`), plus `u16 prop_size_deci_in`, `u16 auw_centi_kg`, `u16 agility_pct`, `u16 authority_trim_pct`, `u8 manual_override`.
- **`MSP2_ST4R_SET_LADRC_CONFIG` (`0x3501`)**: Accepts the 35-byte base payload or 44-byte payload with 4-Param physical fields. When `manual_override == 0`, firmware recomputes axis parameters via `indiApplyPhysicalParams()`. When disarmed on the active profile, calls `indiRefreshRuntimeRatesAndFilters()`; when armed, calls bumpless `ladrcReconfigureLive()`. (`MSP_SET_FILTER_CONFIG` is rejected with `MSP_RESULT_ERROR` while armed.)
- **`MSP2_ST4R_GET_LADRC_STATE` (`0x3502`)**: Reply (74 bytes): `u8 active (1)`, `u8 profileIndex`, `u16 targetPidLooptimeUs`, `u16 b0ScalePermille`, `u16 mixerAuthorityPermille`, 3x axis floats (`z1_dps`, `z2`, `distTermSum`, `deltaU_sum`, `uCmd_sum`), `u8 feedbackMode` (`0=ACTUATOR_MODEL`, `1=DSHOT_RPM`), `u8 dshotActiveMotors`, `u16 fIndiDeciHz`, `u16 fRpmMinDeciHz`.
- **`MSP2_ST4R_GET_PARAM_DEBUG` (`0x3503`)**: EEPROM health, loop rates, active filter summary, and per-axis runtime state (`order`, `dt_us`, `wc`, `wo`, `b0Nom`, `b0Eff`, `lc1..lc3`, `distLimitNorm`).
- **`MSP2_ST4R_SDCARD_FORMAT` (`0x3504`)**: Formats SD card when disarmed.

### 3.2 `MSP2_ST4R_GET_COMPANION_STATUS` (`0x3510`)
Reply (29 bytes, little-endian):
- `u8 enabled`
- `u8 link_alive`
- `u8 offboard_active`
- `u8 active_mode` (`0=NONE`, `1=RATE_THRUST`, `2=ATTITUDE_THRUST`)
- `u8 failsafe_policy` (`0=PILOT_TAKEOVER`, `1=LEVEL_HOLD`, `2=RTH`)
- `u8 allow_offboard_rate`
- `u8 allow_offboard_att`
- `u16 default_ttl_ms`
- `u16 telemetry_hz`
- `u16 last_seq`
- `u32 rx_packet_count`
- `u32 timeout_count`
- `i64 last_clock_offset_us`

### 3.3 `MSP2_ST4R_SET_COMPANION_CMD` (`0x3511`)
Request (21 bytes, little-endian):
- `u16 seq` — monotonic packet sequence number
- `u16 ttl_ms` — time-to-live watchdog in ms (`0` uses `companion_ttl_ms`, clamped to `<= 2000 ms`)
- `u8 mode` — `0=NONE`, `1=RATE_THRUST`, `2=ATTITUDE_THRUST`
- `f32 target[4]` — `[roll/p, pitch/q, yaw/r, throttle (0.0..1.0)]` (NaN/Inf targets rejected by `companionLinkAcceptOffboardCmd`)

### 3.4 `MSP2_ST4R_TIME_SYNC` (`0x3512`)
Request (8 bytes): `u64 client_tx_us`
Reply (16 bytes): `u64 client_tx_us`, `u64 fc_rx_us` (`micros()`).

## 4. Safety & Authority Handover Rules
1. **Arming Gate**: `companionLinkIsOffboardActive()` is strictly `false` unless the FC is armed (`ARMING_FLAG(ARMED)`).
2. **TTL Watchdog**: `companionLinkUpdate()` checks `cmpTimeUs(now, receivedAtUs) > ttl_ms * 1000` on every RC/control loop tick. If the companion freezes or disconnects, offboard authority clears immediately and increments `timeout_count`.
3. **Pilot RC Override**: Flipping the RC offboard switch or moving sticks beyond deadband returns control to the pilot Synchronized INDI loop within 1 loop cycle (< 1 ms).
4. **Closed-Loop Attitude TD Bypass**: In `ATTITUDE_THRUST` mode, `pidController()` sets `tdWeight = 0.0` so the inner INDI loop does not double-filter outer-loop attitude rate commands.
