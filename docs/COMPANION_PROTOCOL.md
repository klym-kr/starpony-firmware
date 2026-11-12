# St4rpony Companion PC Link & Protocol Specification

Status: **Phase 2 Offboard Actuation, Validation & Safety Policies Implemented** (`RATE_THRUST` / `ATTITUDE_THRUST` wired into `rc.c`, `pid.c`, `mixer.c`, `core.c`).

## 1. Architecture Philosophy
- **STM32 Flight Controller (`st4rpony-firmware`)**: Hard real-time inner loops only — gyro filtering, attitude estimation, LADRC rate/angle loops, DShot/servo mixing, RC RX & failsafes.
- **Companion PC (Raspberry Pi 4/5, Orange Pi 5)**: High-level autonomy — VIO/SLAM, obstacle avoidance, mission planning, trajectory generation, telemetry logging.

## 2. Physical Connection (Recommended)
- **Primary In-Flight Link**: Dedicated 3.3V TTL Hardware UART (`TX` -> `RX`, `RX` -> `TX`, shared `GND`) with Circular DMA RX + IDLE-line interrupt and DMA TX.
  - On `SPEEDYBEEF405V3`, `UART3` (`PC10`/`PC11`) or `UART6` (`PC6`/`PC7`) are ideal when free; `UART4` is wired to the onboard Bluetooth module.
  - Recommended baud rate: `921600` bps (default) up to `2000000` bps.
- **Secondary Bench Link**: USB-CDC Virtual COM port (12 Mbps Full Speed) for desktop HITL/SITL.

## 3. Protocol Layout
St4rpony uses **MSPv2** (`$X<` request / `$X>` response / `$X!` error, CRC8-DVB-S2) for unified Configurator + Companion interoperability, with dedicated Extension IDs in `0x3500..0x351F` (defined in `src/main/msp/msp_protocol_v2_st4rpony.h`).

### 3.1 `MSP2_ST4R_GET_COMPANION_STATUS` (`0x3510`)
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

### 3.2 `MSP2_ST4R_SET_COMPANION_CMD` (`0x3511`)
Request (21 bytes, little-endian):
- `u16 seq` — monotonic packet sequence number
- `u16 ttl_ms` — time-to-live watchdog in ms (`0` uses `companion_ttl_ms`, clamped to `<= 2000 ms`)
- `u8 mode` — `0=NONE`, `1=RATE_THRUST`, `2=ATTITUDE_THRUST`
- `f32 target[4]` — `[roll/p, pitch/q, yaw/r, throttle (0.0..1.0)]`

### 3.3 `MSP2_ST4R_TIME_SYNC` (`0x3512`)
Request (8 bytes): `u64 client_tx_us`
Reply (16 bytes): `u64 client_tx_us`, `u64 fc_rx_us` (`micros()`).

## 4. Safety & Authority Handover Rules
1. **Arming Gate**: `companionLinkIsOffboardActive()` is strictly `false` unless the FC is armed (`ARMING_FLAG(ARMED)`).
2. **TTL Watchdog**: `companionLinkUpdate()` checks `cmpTimeUs(now, receivedAtUs) > ttl_ms * 1000` on every RC/control loop tick. If the companion freezes or disconnects, offboard authority clears immediately and increments `timeout_count`.
3. **Pilot RC Override**: Flipping the RC offboard switch or moving sticks beyond deadband returns control to the pilot LADRC loop within 1 loop cycle (< 1 ms).
