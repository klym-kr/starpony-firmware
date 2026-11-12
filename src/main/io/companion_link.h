/*
 * This file is part of St4rpony (derived from Betaflight).
 *
 * St4rpony is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * St4rpony is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "pg/pg.h"

typedef enum {
    COMPANION_MODE_NONE = 0,
    COMPANION_MODE_RATE_THRUST = 1,
    COMPANION_MODE_ATTITUDE_THRUST = 2,
} companionControlMode_e;

typedef enum {
    COMPANION_FAILSAFE_PILOT_TAKEOVER = 0,
    COMPANION_FAILSAFE_LEVEL_HOLD = 1,
    COMPANION_FAILSAFE_RTH = 2,
} companionFailsafePolicy_e;

typedef struct companionConfig_s {
    uint8_t enabled;             // Master enable for companion link acceptance
    uint16_t default_ttl_ms;     // Default offboard setpoint timeout watchdog
    uint16_t telemetry_hz;       // Target FC->Companion fast state push frequency
    uint8_t failsafe_policy;     // companionFailsafePolicy_e
    uint8_t allow_offboard_rate; // Permit RATE_THRUST offboard setpoints
    uint8_t allow_offboard_att;  // Permit ATTITUDE_THRUST offboard setpoints
} companionConfig_t;

PG_DECLARE(companionConfig_t, companionConfig);

typedef struct companionSetpoint_s {
    uint16_t seq;
    uint16_t ttlMs;
    uint8_t mode;          // companionControlMode_e
    float target[4];       // [roll/p, pitch/q, yaw/r, throttle 0..1]
    timeUs_t receivedAtUs;
} companionSetpoint_t;

typedef struct companionStatus_s {
    bool linkAlive;
    bool offboardActive;
    uint8_t activeMode;
    uint16_t lastSeq;
    uint32_t rxPacketCount;
    uint32_t timeoutCount;
    timeUs_t lastPacketUs;
    int64_t lastClockOffsetUs;
} companionStatus_t;

void companionLinkInit(void);
void companionLinkUpdate(timeUs_t currentTimeUs);
bool companionLinkAcceptOffboardCmd(uint16_t seq, uint16_t ttlMs, uint8_t mode,
                                    const float target[4], timeUs_t currentTimeUs);
void companionLinkSetPilotOverride(bool overrideSwitchOrFailsafeActive, bool stickOverrideActive);
bool companionLinkIsOffboardActive(void);
bool companionLinkIsLevelHoldActive(void);
bool companionLinkShouldTriggerRth(void);
float companionLinkGetFailsafeThrottle(void);
const companionSetpoint_t *companionLinkGetSetpoint(void);
const companionStatus_t *companionLinkGetStatus(void);
void companionLinkRecordTimeSync(int64_t offsetUs);
