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

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "flight/ladrc.h"
#include "flight/pid.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(ladrcProfile_t, PID_PROFILE_COUNT, ladrcProfiles, PG_LADRC_PROFILE, 2);

void resetLadrcProfile(ladrcProfile_t *profile)
{
    memset(profile, 0, sizeof(*profile));
    profile->anti_windup = 1;
    profile->vbat_b0_comp = 1;
    profile->airspeed_sched = 0;
    profile->airspeed_ref_cms = 1500;
    profile->disturbance_limit = 500;
    profile->sat_z3_inhibit = 1;      // Inhibit z3 growth while mixer saturates/clips
    profile->ground_wc_pct = 25;      // 25% stiffness on ground before liftoff to prevent tip-over

    // Default 10" cargo payload-transfer baseline (safe across 7"-10"; 16"/20"+ select preset)
    // Roll & Pitch: 2nd-order LADRC (captures heavy prop/motor phase lag via 3-state LESO)
    profile->axis[FD_ROLL] = (ladrcAxisProfile_t){
        .order = LADRC_ORDER_2,
        .wc_deci = 105,  // 10.5 rad/s
        .wo_deci = 420,  // 42.0 rad/s (4x wc)
        .b0_deci = 1900, // 190.0 kdeg/s^3 per unit norm
        .td_hz = 45,
    };
    profile->axis[FD_PITCH] = (ladrcAxisProfile_t){
        .order = LADRC_ORDER_2,
        .wc_deci = 100,  // 10.0 rad/s
        .wo_deci = 400,  // 40.0 rad/s (4x wc)
        .b0_deci = 1750, // 175.0 kdeg/s^3 per unit norm
        .td_hz = 45,
    };
    // Yaw: 1st-order LADRC (lower noise sensitivity & low phase lag for torque/drag axis)
    profile->axis[FD_YAW] = (ladrcAxisProfile_t){
        .order = LADRC_ORDER_1,
        .wc_deci = 75,   // 7.5 rad/s
        .wo_deci = 300,  // 30.0 rad/s (4x wc)
        .b0_deci = 80,   // 8.0 kdeg/s^2 per unit norm
        .td_hz = 35,
    };
}

void pgResetFn_ladrcProfiles(ladrcProfile_t *profiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetLadrcProfile(&profiles[i]);
    }
}

static FAST_DATA_ZERO_INIT ladrcRuntimeAxis_t ladrcRuntime[XYZ_AXIS_COUNT];
static FAST_DATA_ZERO_INIT uint8_t activeLadrcProfileIndex;
static FAST_DATA float externalB0Scale = 1.0f;
static FAST_DATA bool mixerSaturatedFlag = false;
static FAST_DATA bool airborneState = false;
static FAST_DATA float liftoffGyroTimerS = 0.0f;
static FAST_DATA float airborneBlend = 0.0f;

void ladrcSetMixerSaturated(bool saturated)
{
    mixerSaturatedFlag = saturated;
}

bool ladrcIsMixerSaturated(void)
{
    return mixerSaturatedFlag;
}

void ladrcUpdateLiftoffState(float commandedThrottle, float gyroPeakDps, float dT)
{
    const float safeDt = (dT > 1e-6f) ? dT : 0.000125f;
    // Liftoff detected if commanded throttle >= 20% (0.20) or sustained rotational motion > 20 dps for 25ms
    if (!airborneState) {
        if (commandedThrottle >= 0.20f) {
            airborneState = true;
        } else if (gyroPeakDps >= 20.0f) {
            liftoffGyroTimerS += safeDt;
            if (liftoffGyroTimerS >= 0.025f) {
                airborneState = true;
            }
        } else {
            liftoffGyroTimerS = 0.0f;
        }
    }

    if (airborneState) {
        if (airborneBlend < 1.0f) {
            airborneBlend = MIN(1.0f, airborneBlend + safeDt * 5.0f); // 200ms ramp
        }
    } else {
        airborneBlend = 0.0f;
    }
}

void ladrcSetAirborne(bool airborne)
{
    airborneState = airborne;
    liftoffGyroTimerS = 0.0f;
    airborneBlend = airborne ? 1.0f : 0.0f;
}

bool ladrcIsAirborne(void)
{
    return airborneState;
}

void ladrcResetAxisState(int axis)
{
    if (axis < 0 || axis >= XYZ_AXIS_COUNT) {
        return;
    }
    ladrcRuntimeAxis_t *s = &ladrcRuntime[axis];
    s->tdR1 = 0.0f;
    s->tdR2 = 0.0f;
    s->z1 = 0.0f;
    s->z2 = 0.0f;
    s->z3 = 0.0f;
    s->yLastNorm = 0.0f;
    s->rLastNorm = 0.0f;
    s->rDotLast = 0.0f;
    s->z1Last = 0.0f;
    s->z2Last = 0.0f;
    s->z3Last = 0.0f;
    s->uCmdNorm = 0.0f;
    s->uRealNorm = 0.0f;
    s->pTermSum = 0.0f;
    s->dTermSum = 0.0f;
    s->fTermSum = 0.0f;
    s->distTermSum = 0.0f;
    s->pendingTimeUpdate = false;
    s->resetMode = LADRC_RESET_NONE;
}

void ladrcResetState(void)
{
    airborneState = true; // Default airborne for standalone testing / bench
    liftoffGyroTimerS = 0.0f;
    airborneBlend = 1.0f;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        ladrcResetAxisState(axis);
    }
}

void ladrcResetAxisDisturbance(int axis)
{
    if (axis < 0 || axis >= XYZ_AXIS_COUNT) {
        return;
    }
    ladrcRuntimeAxis_t *s = &ladrcRuntime[axis];
    if (s->order == LADRC_ORDER_1) {
        s->z2 = 0.0f;
    } else {
        s->z3 = 0.0f;
    }
    s->distTermSum = 0.0f;
    if (s->resetMode != LADRC_RESET_OBSERVER_FULL) {
        s->resetMode = LADRC_RESET_DISTURBANCE;
    }
}

void ladrcResetDisturbance(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        ladrcResetAxisDisturbance(axis);
    }
}

void ladrcSetExternalB0Scale(float scale)
{
    externalB0Scale = constrainf(scale, 0.1f, 10.0f);
}

float ladrcGetExternalB0Scale(void)
{
    return externalB0Scale;
}

static void configureAxisGains(ladrcRuntimeAxis_t *s, const ladrcAxisProfile_t *cfg, float distLimitNorm, float dT)
{
    const float safeDt = (dT > 1e-6f) ? dT : 0.000125f;
    const uint8_t order = (cfg->order == LADRC_ORDER_1) ? LADRC_ORDER_1 : LADRC_ORDER_2;
    const float wc = MAX(0.5f, cfg->wc_deci * 0.1f);
    const float wo = MAX(1.0f, cfg->wo_deci * 0.1f);
    const float b0 = MAX(0.1f, cfg->b0_deci * 0.1f);

    s->order = order;
    s->dt = safeDt;
    s->wc = wc;
    s->wo = wo;
    s->b0Nom = b0;
    s->b0Eff = b0;
    s->distLimitNorm = distLimitNorm;
    s->tdOmega = (cfg->td_hz > 0) ? (2.0f * M_PIf * (float)cfg->td_hz) : 0.0f;
    s->tdAlpha = (s->tdOmega > 0.0f) ? (1.0f - expf(-s->tdOmega * safeDt)) : 0.0f;

    // Exact ZOH discrete current-observer pole placement at z = lambda = exp(-wo * dt).
    // Guarantees observer stability regardless of loop rate or high wo*dt.
    const float lambda = expf(-wo * safeDt);
    const float oneMinusLam = 1.0f - lambda;

    if (order == LADRC_ORDER_1) {
        s->lc1 = 1.0f - lambda * lambda;
        s->lc2 = (oneMinusLam * oneMinusLam) / safeDt;
        s->lc3 = 0.0f;
    } else {
        s->lc1 = 1.0f - lambda * lambda * lambda;
        s->lc2 = 1.5f * oneMinusLam * oneMinusLam * (1.0f + lambda) / safeDt;
        s->lc3 = (oneMinusLam * oneMinusLam * oneMinusLam) / (safeDt * safeDt);
    }
}

void ladrcInit(uint8_t profileIndex, float dT)
{
    if (profileIndex >= PID_PROFILE_COUNT) {
        profileIndex = 0;
    }
    activeLadrcProfileIndex = profileIndex;
    const ladrcProfile_t *profile = ladrcProfiles(profileIndex);
    const float distLimitNorm = constrainf(profile->disturbance_limit / PID_MIXER_SCALING, 0.0f, 1.0f);

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        configureAxisGains(&ladrcRuntime[axis], &profile->axis[axis], distLimitNorm, dT);
    }
    ladrcResetState();
}

float ladrcComputeB0Scale(const ladrcProfile_t *profile, float vbatRatio, float airspeedCms, bool fixedWing)
{
    float scale = 1.0f;

    // Multirotor thrust scales ~V^2 => hover throttle u_hover ~ 1/V, so local thrust derivative (db0/du) ~ V^1.
    // Fixed-wing control surface servos are driven from a regulated BEC rail, independent of main pack Vbat.
    if (!fixedWing && profile->vbat_b0_comp && vbatRatio > 0.1f) {
        scale *= constrainf(vbatRatio, 0.70f, 1.25f);
    }

    if (fixedWing && profile->airspeed_sched && profile->airspeed_ref_cms > 100 && airspeedCms > 100.0f) {
        const float ratio = airspeedCms / (float)profile->airspeed_ref_cms;
        scale *= constrainf(ratio * ratio, 0.25f, 4.0f);
    }

    return scale;
}

float FAST_CODE ladrcStepAxisEx(int axis, float setpointDps, float gyroDps, float pidSumLimit, float b0Scale,
                                float tdWeight, ladrcResetMode_e resetMode,
                                float *pTermOut, float *dTermOut, float *fTermOut, float *distTermOut)
{
    if (axis < 0 || axis >= XYZ_AXIS_COUNT) {
        return 0.0f;
    }

    ladrcRuntimeAxis_t *s = &ladrcRuntime[axis];
    const float y = gyroDps * LADRC_RATE_SCALE_TO_NORM;
    const float rRaw = setpointDps * LADRC_RATE_SCALE_TO_NORM;
    const float b0 = MAX(0.05f, s->b0Nom * constrainf(b0Scale, 0.1f, 10.0f));
    s->b0Eff = b0;

    // 1. Optional unconditionally-stable critically damped 2nd-order tracking differentiator.
    // Keep internal TD state tracking rRaw continuously, and blend/bypass via tdWeight
    // so closed-loop outer attitude controllers do not suffer double-filtering phase lag.
    const float safeTdWeight = constrainf(tdWeight, 0.0f, 1.0f);
    float r = rRaw;
    float rDot = 0.0f;
    if (s->tdOmega > 0.0f) {
        s->tdR1 += s->tdAlpha * (rRaw - s->tdR1);
        s->tdR2 += s->tdAlpha * (s->tdR1 - s->tdR2);
        if (safeTdWeight >= 1.0f) {
            r = s->tdR2;
            rDot = s->tdOmega * (s->tdR1 - s->tdR2);
        } else if (safeTdWeight > 0.0f) {
            const float rFiltered = s->tdR2;
            const float rDotFiltered = s->tdOmega * (s->tdR1 - s->tdR2);
            r = rRaw + safeTdWeight * (rFiltered - rRaw);
            rDot = safeTdWeight * rDotFiltered;
        }
    } else {
        s->tdR1 = rRaw;
        s->tdR2 = rRaw;
    }

    // 2. Current-observer measurement update using latest filtered gyro rate y_k
    const ladrcProfile_t *profile = ladrcProfiles(activeLadrcProfileIndex);
    const float innov = y - s->z1;
    s->z1 += s->lc1 * innov;
    s->z2 += s->lc2 * innov;

    // ADRC-033: Disturbance growth inhibit during actuator saturation or grounded idle
    // If the mixer is pinned or we have not lifted off, only allow disturbance to decrease in magnitude towards 0.
    const bool inhibitGrowth = (profile->sat_z3_inhibit && mixerSaturatedFlag) || !airborneState;

    if (s->order == LADRC_ORDER_2) {
        const float z3Updated = s->z3 + s->lc3 * innov;
        if (inhibitGrowth) {
            if (s->z3 > 0.0f) {
                s->z3 = constrainf(z3Updated, 0.0f, s->z3);
            } else if (s->z3 < 0.0f) {
                s->z3 = constrainf(z3Updated, s->z3, 0.0f);
            } else {
                s->z3 = 0.0f;
            }
        } else {
            s->z3 = z3Updated;
        }
    } else { // LADRC_ORDER_1 (z2 is disturbance)
        if (inhibitGrowth) {
            const float z2Prev = s->z2 - s->lc2 * innov;
            if (z2Prev > 0.0f) {
                s->z2 = constrainf(s->z2, 0.0f, z2Prev);
            } else if (z2Prev < 0.0f) {
                s->z2 = constrainf(s->z2, z2Prev, 0.0f);
            } else {
                s->z2 = 0.0f;
            }
        }
    }

    // Clamp observer disturbance/acceleration states to prevent windup when constrained (ground idle / crash / spikes)
    const float maxDistState = s->distLimitNorm * b0;
    if (s->order == LADRC_ORDER_1) {
        s->z2 = constrainf(s->z2, -maxDistState, maxDistState);
    } else {
        const float maxAccelState = MAX(50.0f, 4.0f * b0 / MAX(1.0f, s->wc));
        s->z2 = constrainf(s->z2, -maxAccelState, maxAccelState);
        s->z3 = constrainf(s->z3, -maxDistState, maxDistState);
    }

    if (resetMode == LADRC_RESET_OBSERVER_FULL) {
        s->z2 = 0.0f;
        s->z3 = 0.0f;
        s->resetMode = LADRC_RESET_OBSERVER_FULL;
    } else if (resetMode == LADRC_RESET_DISTURBANCE) {
        if (s->order == LADRC_ORDER_1) {
            s->z2 = 0.0f;
        } else {
            s->z3 = 0.0f;
        }
        s->resetMode = LADRC_RESET_DISTURBANCE;
    } else {
        s->resetMode = LADRC_RESET_NONE;
    }

    // Snapshot tick-k measurement-updated state before ladrcPostMixerUpdate() predicts k+1|k
    s->yLastNorm = y;
    s->rLastNorm = r;
    s->rDotLast = rDot;
    s->z1Last = s->z1;
    s->z2Last = s->z2;
    s->z3Last = s->z3;

    // 3. Linear State Error Feedback (LSEF) + disturbance cancellation
    const float sumLimitNorm = constrainf(pidSumLimit / PID_MIXER_SCALING, 0.05f, 1.0f);
    float uP = 0.0f;
    float uD = 0.0f;
    float uF = 0.0f;
    float uDist = 0.0f;

    // Apply ground-wc attenuation while grounded to prevent tip-over before liftoff
    float wcEff = s->wc;
    if (profile->ground_wc_pct > 0 && profile->ground_wc_pct < 100) {
        const float groundWcRatio = (float)profile->ground_wc_pct * 0.01f;
        wcEff = s->wc * (groundWcRatio + (1.0f - groundWcRatio) * airborneBlend);
    }

    if (s->order == LADRC_ORDER_1) {
        uP = (wcEff * (r - s->z1)) / b0;
        uF = rDot / b0;
        uDist = -s->z2 / b0;
    } else {
        uP = (wcEff * wcEff * (r - s->z1)) / b0;
        uD = (-2.0f * wcEff * s->z2) / b0;
        uF = (2.0f * wcEff * rDot) / b0;
        uDist = -s->z3 / b0;
    }

    const float uTotal = constrainf(uP + uD + uF + uDist, -sumLimitNorm, sumLimitNorm);
    s->uCmdNorm = uTotal;
    s->uRealNorm = uTotal;
    s->pendingTimeUpdate = true;

    s->pTermSum = uP * PID_MIXER_SCALING;
    s->dTermSum = uD * PID_MIXER_SCALING;
    s->fTermSum = uF * PID_MIXER_SCALING;
    s->distTermSum = uDist * PID_MIXER_SCALING;

    if (pTermOut) {
        *pTermOut = s->pTermSum;
    }
    if (dTermOut) {
        *dTermOut = s->dTermSum;
    }
    if (fTermOut) {
        *fTermOut = s->fTermSum;
    }
    if (distTermOut) {
        *distTermOut = s->distTermSum;
    }

    if (debugMode == DEBUG_LADRC && axis == FD_ROLL) {
        DEBUG_SET(DEBUG_LADRC, 0, lrintf(s->z1 * LADRC_RATE_SCALE_FROM_NORM));
        DEBUG_SET(DEBUG_LADRC, 1, lrintf(s->distTermSum));
        DEBUG_SET(DEBUG_LADRC, 2, lrintf(s->pTermSum + s->dTermSum));
        DEBUG_SET(DEBUG_LADRC, 3, lrintf(s->uCmdNorm * PID_MIXER_SCALING));
        DEBUG_SET(DEBUG_LADRC, 6, lrintf((s->b0Eff / MAX(0.05f, s->b0Nom)) * 1000.0f));
    }

    return s->uCmdNorm * PID_MIXER_SCALING;
}

float FAST_CODE ladrcStepAxis(int axis, float setpointDps, float gyroDps, float pidSumLimit, float b0Scale,
                              float *pTermOut, float *dTermOut, float *fTermOut, float *distTermOut)
{
    return ladrcStepAxisEx(axis, setpointDps, gyroDps, pidSumLimit, b0Scale, 1.0f, LADRC_RESET_NONE,
                           pTermOut, dTermOut, fTermOut, distTermOut);
}

void FAST_CODE ladrcPostMixerUpdate(float mixerAuthorityScale, bool stabilised)
{
    if (!stabilised) {
        ladrcResetState();
        return;
    }

    const ladrcProfile_t *profile = ladrcProfiles(activeLadrcProfileIndex);
    const float clampedScale = profile->anti_windup ? constrainf(mixerAuthorityScale, 0.0f, 1.0f) : 1.0f;

    if (debugMode == DEBUG_LADRC) {
        DEBUG_SET(DEBUG_LADRC, 4, lrintf(ladrcRuntime[FD_PITCH].distTermSum));
        DEBUG_SET(DEBUG_LADRC, 5, lrintf(ladrcRuntime[FD_YAW].distTermSum));
        DEBUG_SET(DEBUG_LADRC, 7, lrintf(clampedScale * 1000.0f));
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        ladrcRuntimeAxis_t *s = &ladrcRuntime[axis];
        if (!s->pendingTimeUpdate) {
            continue;
        }
        s->pendingTimeUpdate = false;
        s->uRealNorm = s->uCmdNorm * clampedScale;

        if (s->resetMode == LADRC_RESET_OBSERVER_FULL) {
            s->z2 = 0.0f;
            s->z3 = 0.0f;
            continue;
        }

        const float dt = s->dt;
        const float b0u = s->b0Eff * s->uRealNorm;

        if (s->order == LADRC_ORDER_1) {
            s->z1 += dt * (s->z2 + b0u);
        } else {
            s->z1 += dt * s->z2 + 0.5f * dt * dt * (s->z3 + b0u);
            s->z2 += dt * (s->z3 + b0u);
        }
    }
}

void ladrcUpdateSingleStep(int axis, float setpointDps, float gyroDps, float pidSumLimit,
                           float b0Scale, float mixerAuthorityScale)
{
    ladrcStepAxis(axis, setpointDps, gyroDps, pidSumLimit, b0Scale, NULL, NULL, NULL, NULL);
    ladrcRuntimeAxis_t *s = &ladrcRuntime[axis];
    if (s->pendingTimeUpdate) {
        s->pendingTimeUpdate = false;
        const ladrcProfile_t *profile = ladrcProfiles(activeLadrcProfileIndex);
        const float clampedScale = profile->anti_windup ? constrainf(mixerAuthorityScale, 0.0f, 1.0f) : 1.0f;
        s->uRealNorm = s->uCmdNorm * clampedScale;
        if (s->resetMode == LADRC_RESET_OBSERVER_FULL) {
            s->z2 = 0.0f;
            s->z3 = 0.0f;
            return;
        }
        const float dt = s->dt;
        const float b0u = s->b0Eff * s->uRealNorm;
        if (s->order == LADRC_ORDER_1) {
            s->z1 += dt * (s->z2 + b0u);
        } else {
            s->z1 += dt * s->z2 + 0.5f * dt * dt * (s->z3 + b0u);
            s->z2 += dt * (s->z3 + b0u);
        }
    }
}

const ladrcRuntimeAxis_t *ladrcGetAxisState(int axis)
{
    if (axis < 0 || axis >= XYZ_AXIS_COUNT) {
        return NULL;
    }
    return &ladrcRuntime[axis];
}
