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

#include <math.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"
#include "common/maths.h"
#include "common/time.h"
#include "fc/runtime_config.h"
#include "io/companion_link.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#define COMPANION_MAX_RATE_DPS     2000.0f
#define COMPANION_MAX_ATTITUDE_DEG   85.0f

PG_REGISTER_WITH_RESET_TEMPLATE(companionConfig_t, companionConfig, PG_COMPANION_CONFIG, 1);

PG_RESET_TEMPLATE(companionConfig_t, companionConfig,
    .enabled = 0,
    .default_ttl_ms = 150,
    .telemetry_hz = 200,
    .failsafe_policy = COMPANION_FAILSAFE_PILOT_TAKEOVER,
    .allow_offboard_rate = 1,
    .allow_offboard_att = 1,
);

static companionSetpoint_t activeSetpoint;
static companionStatus_t companionStatus;
static bool pilotOverride;
static bool stickOverrideLatched;
static bool failsafeHoldActive;
static bool failsafeRthActive;
static float failsafeHoldThrottle;

void companionLinkInit(void)
{
    memset(&activeSetpoint, 0, sizeof(activeSetpoint));
    memset(&companionStatus, 0, sizeof(companionStatus));
    pilotOverride = false;
    stickOverrideLatched = false;
    failsafeHoldActive = false;
    failsafeRthActive = false;
    failsafeHoldThrottle = 0.0f;
}

bool companionLinkAcceptOffboardCmd(uint16_t seq, uint16_t ttlMs, uint8_t mode,
                                    const float target[4], timeUs_t currentTimeUs)
{
    const companionConfig_t *cfg = companionConfig();
    if (!cfg->enabled || !target) {
        return false;
    }

    for (int i = 0; i < 4; i++) {
        if (!isfinite(target[i])) {
            return false;
        }
    }

    if (mode == COMPANION_MODE_RATE_THRUST && !cfg->allow_offboard_rate) {
        return false;
    }
    if (mode == COMPANION_MODE_ATTITUDE_THRUST && !cfg->allow_offboard_att) {
        return false;
    }
    if (mode > COMPANION_MODE_ATTITUDE_THRUST) {
        return false;
    }

    const uint16_t effectiveTtl = (ttlMs > 0) ? MIN(ttlMs, (uint16_t)2000) : cfg->default_ttl_ms;

    activeSetpoint.seq = seq;
    activeSetpoint.ttlMs = effectiveTtl;
    activeSetpoint.mode = mode;
    if (mode == COMPANION_MODE_ATTITUDE_THRUST) {
        activeSetpoint.target[0] = constrainf(target[0], -COMPANION_MAX_ATTITUDE_DEG, COMPANION_MAX_ATTITUDE_DEG);
        activeSetpoint.target[1] = constrainf(target[1], -COMPANION_MAX_ATTITUDE_DEG, COMPANION_MAX_ATTITUDE_DEG);
        activeSetpoint.target[2] = constrainf(target[2], -COMPANION_MAX_RATE_DPS, COMPANION_MAX_RATE_DPS);
    } else if (mode == COMPANION_MODE_RATE_THRUST) {
        activeSetpoint.target[0] = constrainf(target[0], -COMPANION_MAX_RATE_DPS, COMPANION_MAX_RATE_DPS);
        activeSetpoint.target[1] = constrainf(target[1], -COMPANION_MAX_RATE_DPS, COMPANION_MAX_RATE_DPS);
        activeSetpoint.target[2] = constrainf(target[2], -COMPANION_MAX_RATE_DPS, COMPANION_MAX_RATE_DPS);
    } else {
        activeSetpoint.target[0] = 0.0f;
        activeSetpoint.target[1] = 0.0f;
        activeSetpoint.target[2] = 0.0f;
    }
    activeSetpoint.target[3] = constrainf(target[3], 0.0f, 1.0f);
    activeSetpoint.receivedAtUs = currentTimeUs;

    companionStatus.linkAlive = true;
    companionStatus.lastSeq = seq;
    companionStatus.rxPacketCount++;
    companionStatus.lastPacketUs = currentTimeUs;

    if (mode == COMPANION_MODE_NONE) {
        stickOverrideLatched = false;
        failsafeHoldActive = false;
        failsafeRthActive = false;
        companionStatus.offboardActive = false;
        companionStatus.activeMode = COMPANION_MODE_NONE;
    } else {
        failsafeHoldActive = false;
        failsafeRthActive = false;
        const bool canEngage = !pilotOverride && !stickOverrideLatched;
        companionStatus.offboardActive = canEngage;
        companionStatus.activeMode = canEngage ? mode : COMPANION_MODE_NONE;
    }

    return true;
}

void companionLinkSetPilotOverride(bool overrideSwitchOrFailsafeActive, bool stickOverrideActive)
{
    if (!ARMING_FLAG(ARMED)) {
        stickOverrideLatched = false;
        failsafeHoldActive = false;
        failsafeRthActive = false;
    } else {
        if (overrideSwitchOrFailsafeActive) {
            // Explicit RC switch/failsafe deactivation clears hold/RTH and resets stick latch once released
            stickOverrideLatched = false;
            failsafeHoldActive = false;
            failsafeRthActive = false;
        } else if (stickOverrideActive && (companionStatus.offboardActive || failsafeHoldActive || failsafeRthActive)) {
            stickOverrideLatched = true;
            failsafeHoldActive = false;
            failsafeRthActive = false;
        }
    }

    pilotOverride = overrideSwitchOrFailsafeActive || stickOverrideActive || stickOverrideLatched;
    if (pilotOverride) {
        companionStatus.offboardActive = false;
        companionStatus.activeMode = COMPANION_MODE_NONE;
    }
}

void companionLinkUpdate(timeUs_t currentTimeUs)
{
    const companionConfig_t *cfg = companionConfig();
    if (!cfg->enabled) {
        companionStatus.linkAlive = false;
        companionStatus.offboardActive = false;
        companionStatus.activeMode = COMPANION_MODE_NONE;
        stickOverrideLatched = false;
        failsafeHoldActive = false;
        failsafeRthActive = false;
        return;
    }

    if (!ARMING_FLAG(ARMED)) {
        stickOverrideLatched = false;
        failsafeHoldActive = false;
        failsafeRthActive = false;
    }

    if (companionStatus.linkAlive) {
        const timeDelta_t ageUs = cmpTimeUs(currentTimeUs, activeSetpoint.receivedAtUs);
        const timeDelta_t ttlUs = (timeDelta_t)activeSetpoint.ttlMs * 1000;
        if (ttlUs <= 0 || ageUs > ttlUs) {
            if (companionStatus.offboardActive) {
                companionStatus.timeoutCount++;
                if (ARMING_FLAG(ARMED) && !pilotOverride && !stickOverrideLatched) {
                    failsafeHoldThrottle = constrainf(activeSetpoint.target[3], 0.0f, 0.85f);
                    if (cfg->failsafe_policy == COMPANION_FAILSAFE_LEVEL_HOLD) {
                        failsafeHoldActive = true;
                        failsafeRthActive = false;
                    } else if (cfg->failsafe_policy == COMPANION_FAILSAFE_RTH) {
                        failsafeHoldActive = false;
                        failsafeRthActive = true;
                    }
                }
            }
            companionStatus.linkAlive = false;
            companionStatus.offboardActive = false;
            companionStatus.activeMode = COMPANION_MODE_NONE;
            if (!ARMING_FLAG(ARMED)) {
                stickOverrideLatched = false;
            }
        }
    } else if (!ARMING_FLAG(ARMED)) {
        stickOverrideLatched = false;
    }

    if (pilotOverride || stickOverrideLatched) {
        companionStatus.offboardActive = false;
        companionStatus.activeMode = COMPANION_MODE_NONE;
    }

    DEBUG_SET(DEBUG_COMPANION, 0, companionStatus.linkAlive ? 1 : 0);
    DEBUG_SET(DEBUG_COMPANION, 1, companionLinkIsOffboardActive() ? companionStatus.activeMode : 0);
    DEBUG_SET(DEBUG_COMPANION, 2, (int16_t)(companionStatus.rxPacketCount & 0x7FFF));
    DEBUG_SET(DEBUG_COMPANION, 3, (int16_t)(companionStatus.timeoutCount & 0x7FFF));
}

bool companionLinkIsOffboardActive(void)
{
    return companionStatus.offboardActive && !pilotOverride && !stickOverrideLatched && ARMING_FLAG(ARMED);
}

bool companionLinkIsLevelHoldActive(void)
{
    return failsafeHoldActive && !pilotOverride && !stickOverrideLatched && ARMING_FLAG(ARMED);
}

bool companionLinkShouldTriggerRth(void)
{
    return failsafeRthActive && !pilotOverride && !stickOverrideLatched && ARMING_FLAG(ARMED);
}

float companionLinkGetFailsafeThrottle(void)
{
    return failsafeHoldThrottle;
}

const companionSetpoint_t *companionLinkGetSetpoint(void)
{
    return &activeSetpoint;
}

const companionStatus_t *companionLinkGetStatus(void)
{
    return &companionStatus;
}

void companionLinkRecordTimeSync(int64_t offsetUs)
{
    companionStatus.lastClockOffsetUs = offsetUs;
}
