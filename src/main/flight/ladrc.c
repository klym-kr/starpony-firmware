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

PG_REGISTER_ARRAY_WITH_RESET_FN(ladrcProfile_t, PID_PROFILE_COUNT, ladrcProfiles, PG_LADRC_PROFILE, 3);

static float getSafePropInches(const ladrcProfile_t *profile)
{
    const uint16_t raw = (profile && profile->prop_size_deci_in > 0) ? profile->prop_size_deci_in : 50;
    return constrainf(raw * 0.1f, 3.0f, 30.0f);
}

static float getSafeAuwKg(const ladrcProfile_t *profile)
{
    const uint16_t raw = (profile && profile->auw_centi_kg > 0) ? profile->auw_centi_kg : 65;
    return constrainf(raw * 0.01f, 0.20f, 50.0f);
}

static float getSafeAgility(const ladrcProfile_t *profile)
{
    const uint16_t raw = (profile && profile->agility_pct > 0) ? profile->agility_pct : 100;
    return constrainf(raw * 0.01f, 0.50f, 1.50f);
}

static float getSafeAuthorityTrim(const ladrcProfile_t *profile)
{
    const uint16_t raw = (profile && profile->authority_trim_pct > 0) ? profile->authority_trim_pct : 100;
    return constrainf(raw * 0.01f, 0.50f, 1.50f);
}

float indiComputeFIndiHz(const ladrcProfile_t *profile)
{
    const float D = getSafePropInches(profile);
    return constrainf(450.0f / D, 18.0f, 100.0f);
}

float indiComputeRpmMinHz(const ladrcProfile_t *profile)
{
    const float D = getSafePropInches(profile);
    return constrainf(500.0f / D, 20.0f, 100.0f);
}

void indiComputeDynNotchRangeHz(const ladrcProfile_t *profile, float *minHzOut, float *maxHzOut)
{
    const float D = getSafePropInches(profile);
    if (minHzOut) {
        *minHzOut = constrainf(550.0f / D, 25.0f, 220.0f);
    }
    if (maxHzOut) {
        *maxHzOut = constrainf(2500.0f / D, 120.0f, 600.0f);
    }
}

float indiComputeTauActS(const ladrcProfile_t *profile, bool fixedWing)
{
    if (fixedWing) {
        return 0.030f;
    }
    const float D = getSafePropInches(profile);
    return constrainf(0.015f * powf(D / 5.0f, 1.3f), 0.010f, 0.180f);
}

void indiApplyPhysicalParams(ladrcProfile_t *profile)
{
    if (!profile) {
        return;
    }

    const float D = getSafePropInches(profile);
    const float m = getSafeAuwKg(profile);
    const float A = getSafeAgility(profile);
    const float tAuth = getSafeAuthorityTrim(profile);

    const float wcRoll = constrainf(18.0f * sqrtf(5.0f / D) * A, 4.0f, 30.0f);
    const float wcPitch = 0.95f * wcRoll;
    const float wcYaw = 0.70f * wcRoll;

    const float fTdRp = constrainf(60.0f * sqrtf(5.0f / D) * A, 15.0f, 100.0f);
    const float fTdYaw = 0.75f * fTdRp;

    const float mRef = 0.65f * powf(D / 5.0f, 2.2f);
    const float loadRatio = constrainf(m / mRef, 0.35f, 4.0f);
    const float b0Calc = constrainf(42.0f * powf(5.0f / D, 0.85f) / sqrtf(loadRatio), 5.0f, 150.0f);
    const float b0Roll = b0Calc * tAuth;
    const float b0Pitch = 0.92f * b0Roll;
    const float b0Yaw = 0.30f * b0Roll;

    profile->axis[FD_ROLL].wc_deci = (uint16_t)constrain(lrintf(wcRoll * 10.0f), 20, 2000);
    profile->axis[FD_PITCH].wc_deci = (uint16_t)constrain(lrintf(wcPitch * 10.0f), 20, 2000);
    profile->axis[FD_YAW].wc_deci = (uint16_t)constrain(lrintf(wcYaw * 10.0f), 20, 2000);

    profile->axis[FD_ROLL].wo_deci = (uint16_t)constrain(profile->axis[FD_ROLL].wc_deci * 4, 80, 10000);
    profile->axis[FD_PITCH].wo_deci = (uint16_t)constrain(profile->axis[FD_PITCH].wc_deci * 4, 80, 10000);
    profile->axis[FD_YAW].wo_deci = (uint16_t)constrain(profile->axis[FD_YAW].wc_deci * 4, 80, 10000);

    profile->axis[FD_ROLL].b0_deci = (uint16_t)constrain(lrintf(b0Roll * 10.0f), 20, 60000);
    profile->axis[FD_PITCH].b0_deci = (uint16_t)constrain(lrintf(b0Pitch * 10.0f), 20, 60000);
    profile->axis[FD_YAW].b0_deci = (uint16_t)constrain(lrintf(b0Yaw * 10.0f), 20, 60000);

    profile->axis[FD_ROLL].td_hz = (uint16_t)constrain(lrintf(fTdRp), 10, 250);
    profile->axis[FD_PITCH].td_hz = (uint16_t)constrain(lrintf(fTdRp), 10, 250);
    profile->axis[FD_YAW].td_hz = (uint16_t)constrain(lrintf(fTdYaw), 10, 250);
}

void resetLadrcProfile(ladrcProfile_t *profile)
{
    memset(profile, 0, sizeof(*profile));
    profile->anti_windup = 1;
    profile->vbat_b0_comp = 1;
    profile->airspeed_sched = 0;
    profile->airspeed_ref_cms = 1500;
    profile->disturbance_limit = 500;
    profile->sat_z3_inhibit = 1;      // Inhibit disturbance growth while mixer saturates/clips
    profile->ground_wc_pct = 25;      // 25% stiffness on ground before liftoff to prevent tip-over
    profile->prop_size_deci_in = 50;  // 5.0"
    profile->auw_centi_kg = 65;       // 0.65 kg
    profile->agility_pct = 100;       // 1.00
    profile->authority_trim_pct = 100;// 1.00
    profile->manual_override = 0;     // 4-Param Auto INDI

    profile->axis[FD_ROLL].order = LADRC_ORDER_2;
    profile->axis[FD_PITCH].order = LADRC_ORDER_2;
    profile->axis[FD_YAW].order = LADRC_ORDER_1;
    indiApplyPhysicalParams(profile);
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
static FAST_DATA bool airborneState = true;
static FAST_DATA float liftoffGyroTimerS = 0.0f;
static FAST_DATA float airborneBlend = 1.0f;
static FAST_DATA_ZERO_INIT bool ladrcFixedWing;
static FAST_DATA_ZERO_INIT bool dshotFeedbackValid;
static FAST_DATA_ZERO_INIT uint8_t dshotActiveMotorCount;
static FAST_DATA_ZERO_INIT bool dshotConfigured;
static FAST_DATA bool dshotBatteryPresent = true;
static FAST_DATA float indiLpfHz = 90.0f;
static FAST_DATA float indiRpmMinHz = 100.0f;
static FAST_DATA float upstreamSyncDelayS = 0.0f;

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
    // Liftoff detected if commanded throttle >= 20% (0.20) or sustained rotational motion >= 50 dps with throttle >= 8% for 50ms
    if (!airborneState) {
        if (commandedThrottle >= 0.20f) {
            airborneState = true;
        } else if (commandedThrottle >= 0.08f && gyroPeakDps >= 50.0f) {
            liftoffGyroTimerS += safeDt;
            if (liftoffGyroTimerS >= 0.050f) {
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

static void resetAxisFilters(ladrcRuntimeAxis_t *s)
{
    s->gyroFilter.ic1 = 0.0f;
    s->gyroFilter.ic2 = 0.0f;
    s->feedbackFilter.ic1 = 0.0f;
    s->feedbackFilter.ic2 = 0.0f;
    s->feedbackSyncPt1.state = 0.0f;
    s->actModelPt1.state = 0.0f;
    s->actModelSlew.state = 0.0f;
    s->gyroFiltPrevNorm = 0.0f;
    s->gyroBpPrev = 0.0f;
    s->uActualFeedback = 0.0f;
    s->uRpmFeedback = 0.0f;
    s->uModelOut = 0.0f;
    s->deltaUNorm = 0.0f;
    s->firstSample = true;
    s->feedbackUpdated = false;
    s->dshotRpmValid = false;
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
    resetAxisFilters(s);
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
    mixerSaturatedFlag = false;
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
    s->z3 = 0.0f;
    s->feedbackFilter.ic1 = 0.0f;
    s->feedbackFilter.ic2 = 0.0f;
    s->feedbackSyncPt1.state = 0.0f;
    s->actModelPt1.state = 0.0f;
    s->actModelSlew.state = 0.0f;
    s->uModelOut = 0.0f;
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

static float computeClampedFIndiHz(const ladrcProfile_t *profile, float safeDt)
{
    const float fIndiRaw = indiComputeFIndiHz(profile);
    const float maxCutoff = 0.45f / safeDt;
    float fIndiClamped = constrainf(fIndiRaw, 1.0f, maxCutoff);
    if (ladrcFixedWing) {
        fIndiClamped = MIN(fIndiClamped, 25.0f);
    }
    return fIndiClamped;
}

static void configureAxisActuatorModel(ladrcRuntimeAxis_t *s, float tauActS, float safeDt)
{
    s->tauActS = tauActS;
    const float pt1Gain = pt1FilterGainFromDelay(tauActS, safeDt);
    const float prevPt1State = s->actModelPt1.state;
    const float prevSlewState = s->actModelSlew.state;
    pt1FilterInit(&s->actModelPt1, pt1Gain);
    s->actModelPt1.state = prevPt1State;
    const float maxSlewPerStep = (ladrcFixedWing ? 8.0f : (8.0f / MAX(tauActS, 0.01f))) * safeDt;
    slewFilterInit(&s->actModelSlew, maxSlewPerStep, 2.0f);
    s->actModelSlew.state = prevSlewState;
}

static void configureAxisGains(ladrcRuntimeAxis_t *s, const ladrcAxisProfile_t *cfg,
                               float distLimitNorm, float fIndiClampedHz, float tauActS, float dT, bool resetFilters)
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
    s->fIndiHz = fIndiClampedHz;
    s->tdOmega = (cfg->td_hz > 0) ? (2.0f * M_PIf * (float)cfg->td_hz * 1.553773974f) : 0.0f;
    s->tdAlpha = (s->tdOmega > 0.0f) ? (1.0f - expf(-s->tdOmega * safeDt)) : 0.0f;

    // Exact ZOH discrete observer coefficients retained for diagnostic telemetry
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

    // Twin identical 2nd-order Butterworth SVF Biquad LPFs for gyro and actuator feedback
    const float syncGain = pt1FilterGainFromDelay(upstreamSyncDelayS, safeDt);
    if (resetFilters) {
        svfLowpassFilterInit(&s->gyroFilter, fIndiClampedHz, safeDt);
        svfLowpassFilterInit(&s->feedbackFilter, fIndiClampedHz, safeDt);
        pt1FilterInit(&s->feedbackSyncPt1, syncGain);
    } else {
        svfLowpassFilterUpdate(&s->gyroFilter, fIndiClampedHz, safeDt);
        svfLowpassFilterUpdate(&s->feedbackFilter, fIndiClampedHz, safeDt);
        pt1FilterUpdateCutoff(&s->feedbackSyncPt1, syncGain);
    }
    configureAxisActuatorModel(s, tauActS, safeDt);
}

void ladrcSetFixedWing(bool fixedWing)
{
    if (ladrcFixedWing == fixedWing) {
        return;
    }
    ladrcFixedWing = fixedWing;
    const ladrcProfile_t *profile = ladrcProfiles(activeLadrcProfileIndex);
    const float tauActS = indiComputeTauActS(profile, ladrcFixedWing);
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        ladrcRuntimeAxis_t *s = &ladrcRuntime[axis];
        if (s->dt > 1e-6f) {
            const float fIndiClamped = computeClampedFIndiHz(profile, s->dt);
            indiLpfHz = fIndiClamped;
            s->fIndiHz = fIndiClamped;
            svfLowpassFilterUpdate(&s->gyroFilter, fIndiClamped, s->dt);
            svfLowpassFilterUpdate(&s->feedbackFilter, fIndiClamped, s->dt);
            configureAxisActuatorModel(s, tauActS, s->dt);
        }
    }
}

void ladrcSetUpstreamSyncDelayS(float delayS)
{
    upstreamSyncDelayS = floatIsFiniteBits(delayS) ? constrainf(delayS, 0.0f, 0.025f) : 0.0f;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        ladrcRuntimeAxis_t *s = &ladrcRuntime[axis];
        if (s->dt > 1e-6f) {
            pt1FilterUpdateCutoff(&s->feedbackSyncPt1, pt1FilterGainFromDelay(upstreamSyncDelayS, s->dt));
        }
    }
}

void ladrcSetDshotFeedbackStatus(bool dshotRpmValid, uint8_t activeMotors)
{
    dshotFeedbackValid = dshotRpmValid;
    dshotActiveMotorCount = activeMotors;
}

void ladrcSetDshotConfiguredStatus(bool configured, bool batteryPresent)
{
    dshotConfigured = configured;
    dshotBatteryPresent = batteryPresent;
}

void FAST_CODE ladrcSetAxisActuatorFeedback(int axis, float uActual, float uRpm, bool dshotRpmValid)
{
    if (axis < 0 || axis >= XYZ_AXIS_COUNT) {
        return;
    }
    ladrcRuntimeAxis_t *s = &ladrcRuntime[axis];
    s->uActualFeedback = uActual;
    s->uRpmFeedback = uRpm;
    s->dshotRpmValid = dshotRpmValid;
    s->feedbackUpdated = true;
}

indiFeedbackMode_e indiGetFeedbackMode(void)
{
    return (dshotFeedbackValid && !ladrcFixedWing) ? INDI_FEEDBACK_DSHOT_RPM : INDI_FEEDBACK_ACTUATOR_MODEL;
}

uint8_t indiGetDshotActiveMotorCount(void)
{
    return dshotActiveMotorCount;
}

float indiGetLpfHz(void)
{
    return indiLpfHz;
}

float indiGetRpmMinHz(void)
{
    return indiRpmMinHz;
}

bool indiIsDshotStandbyNoBattery(void)
{
    return !ladrcFixedWing && !dshotFeedbackValid && dshotConfigured && !dshotBatteryPresent;
}

bool indiIsDshotWarningActive(void)
{
    return !ladrcFixedWing && !dshotFeedbackValid && !(dshotConfigured && !dshotBatteryPresent);
}

float indiGetAngleGain(void)
{
    if (ladrcRuntime[FD_ROLL].wc > 0.0f) {
        return 0.25f * ladrcRuntime[FD_ROLL].wc;
    }
    const ladrcProfile_t *profile = ladrcProfiles(activeLadrcProfileIndex);
    const float D = getSafePropInches(profile);
    const float A = getSafeAgility(profile);
    const float wcRoll = constrainf(18.0f * sqrtf(5.0f / D) * A, 4.0f, 30.0f);
    return 0.25f * wcRoll;
}

void ladrcInit(uint8_t profileIndex, float dT)
{
    if (profileIndex >= PID_PROFILE_COUNT) {
        profileIndex = 0;
    }
    activeLadrcProfileIndex = profileIndex;
    upstreamSyncDelayS = 0.0f;
    ladrcProfile_t *profile = ladrcProfilesMutable(profileIndex);
    if (!profile->manual_override) {
        indiApplyPhysicalParams(profile);
    }

    const float safeDt = (dT > 1e-6f) ? dT : 0.000125f;
    const float fIndiClamped = computeClampedFIndiHz(profile, safeDt);
    indiLpfHz = fIndiClamped;
    indiRpmMinHz = indiComputeRpmMinHz(profile);

    const float tauActS = indiComputeTauActS(profile, ladrcFixedWing);
    const float distLimitNorm = constrainf(profile->disturbance_limit / PID_MIXER_SCALING, 0.0f, 1.0f);

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        configureAxisGains(&ladrcRuntime[axis], &profile->axis[axis], distLimitNorm, fIndiClamped, tauActS, safeDt, true);
    }
    ladrcResetState();
}

void ladrcReconfigureLive(uint8_t profileIndex, float dT)
{
    if (profileIndex >= PID_PROFILE_COUNT) {
        profileIndex = 0;
    }
    activeLadrcProfileIndex = profileIndex;
    ladrcProfile_t *profile = ladrcProfilesMutable(profileIndex);
    if (!profile->manual_override) {
        indiApplyPhysicalParams(profile);
    }

    const float safeDt = (dT > 1e-6f) ? dT : 0.000125f;
    const float fIndiClamped = computeClampedFIndiHz(profile, safeDt);
    indiLpfHz = fIndiClamped;
    indiRpmMinHz = indiComputeRpmMinHz(profile);

    const float tauActS = indiComputeTauActS(profile, ladrcFixedWing);
    const float distLimitNorm = constrainf(profile->disturbance_limit / PID_MIXER_SCALING, 0.0f, 1.0f);

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        configureAxisGains(&ladrcRuntime[axis], &profile->axis[axis], distLimitNorm, fIndiClamped, tauActS, safeDt, false);
    }
}

float ladrcComputeB0Scale(const ladrcProfile_t *profile, float vbatRatio, float airspeedCms, bool fixedWing)
{
    float scale = 1.0f;

    // Multirotor thrust scales ~V^2 => hover throttle u_hover ~ 1/V, so local thrust derivative (db0/du) ~ V^1.
    // Fixed-wing control surface servos are driven from a regulated BEC rail, independent of main pack Vbat.
    if (!fixedWing && profile->vbat_b0_comp && vbatRatio > 0.1f) {
        scale *= constrainf(vbatRatio, 0.70f, 1.25f);
    }

    if (fixedWing && profile->airspeed_sched && profile->airspeed_ref_cms > 100 && airspeedCms > 1.0f) {
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
    const float safeGyroDps = floatIsFiniteBits(gyroDps) ? gyroDps : 0.0f;
    const float safeSetpointDps = floatIsFiniteBits(setpointDps) ? setpointDps : 0.0f;
    const float safeB0Scale = floatIsFiniteBits(b0Scale) ? b0Scale : 1.0f;

    const float y = safeGyroDps * LADRC_RATE_SCALE_TO_NORM;
    const float rRaw = safeSetpointDps * LADRC_RATE_SCALE_TO_NORM;
    const float b0 = MAX(0.05f, s->b0Nom * constrainf(safeB0Scale, 0.1f, 10.0f));
    s->b0Eff = b0;

    // 1. Unconditionally-stable critically damped 2nd-order tracking differentiator (ZOH)
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

    // 2. Twin identical 2nd-order Butterworth SVF Biquad LPFs on gyro and actuator feedback
    float v1Bp = 0.0f;
    const float omegaFilt = svfLowpassFilterApplyWithBandpass(&s->gyroFilter, y, &v1Bp);
    float omegaDotMeas = 0.0f;
    if (!s->firstSample) {
        omegaDotMeas = (s->gyroFilter.f * (v1Bp + s->gyroBpPrev)) / s->dt;
    } else {
        s->firstSample = false;
    }
    s->gyroBpPrev = v1Bp;
    s->gyroFiltPrevNorm = omegaFilt;

    const ladrcProfile_t *profile = ladrcProfiles(activeLadrcProfileIndex);
    const bool useRpm = (dshotFeedbackValid && !ladrcFixedWing && s->dshotRpmValid);
    const float uFeedbackRaw = useRpm ? s->uRpmFeedback : s->uModelOut;
    const float uFeedbackSynced = (upstreamSyncDelayS > 1e-5f)
        ? pt1FilterApply(&s->feedbackSyncPt1, uFeedbackRaw)
        : uFeedbackRaw;
    float uMeasFilt = svfLowpassFilterApply(&s->feedbackFilter, uFeedbackSynced);
    uMeasFilt = constrainf(uMeasFilt, -s->distLimitNorm, s->distLimitNorm);

    if (profile->sat_z3_inhibit && mixerSaturatedFlag) {
        const float satCap = MAX(fabsf(s->uRealNorm), 0.05f);
        uMeasFilt = constrainf(uMeasFilt, -satCap, satCap);
    }

    if (!airborneState) {
        uMeasFilt = constrainf(uMeasFilt, -0.12f, 0.12f);
    }

    if (ladrcFixedWing && axis == FD_YAW && safeTdWeight >= 0.99f && fabsf(safeSetpointDps) < 5.0f) {
        const float wash = expf(-2.0f * s->dt);
        s->feedbackFilter.ic2 *= wash;
        uMeasFilt *= wash;
    }

    if (resetMode == LADRC_RESET_OBSERVER_FULL) {
        omegaDotMeas = 0.0f;
        s->gyroBpPrev = 0.0f;
        uMeasFilt = 0.0f;
        s->feedbackFilter.ic1 = 0.0f;
        s->feedbackFilter.ic2 = 0.0f;
        s->feedbackSyncPt1.state = 0.0f;
        s->actModelPt1.state = 0.0f;
        s->actModelSlew.state = 0.0f;
        s->uModelOut = 0.0f;
        s->resetMode = LADRC_RESET_OBSERVER_FULL;
    } else if (resetMode == LADRC_RESET_DISTURBANCE) {
        uMeasFilt = 0.0f;
        s->feedbackFilter.ic1 = 0.0f;
        s->feedbackFilter.ic2 = 0.0f;
        s->feedbackSyncPt1.state = 0.0f;
        s->actModelPt1.state = 0.0f;
        s->actModelSlew.state = 0.0f;
        s->uModelOut = 0.0f;
        s->resetMode = LADRC_RESET_DISTURBANCE;
    } else {
        s->resetMode = LADRC_RESET_NONE;
    }

    // Store synchronized INDI states for MSP telemetry & diagnostics
    s->z1 = omegaFilt;
    s->z2 = omegaDotMeas;
    s->z3 = 0.0f;
    s->yLastNorm = y;
    s->rLastNorm = r;
    s->rDotLast = rDot;
    s->z1Last = s->z1;
    s->z2Last = s->z2;
    s->z3Last = 0.0f;

    // 3. Outer rate law + incremental INDI inner law
    const float sumLimitNorm = constrainf(pidSumLimit / PID_MIXER_SCALING, 0.05f, 1.0f);

    // Apply ground-wc attenuation while grounded to prevent tip-over before liftoff
    float groundScale = 1.0f;
    if (profile->ground_wc_pct > 0 && profile->ground_wc_pct < 100) {
        const float groundWcRatio = (float)profile->ground_wc_pct * 0.01f;
        groundScale = groundWcRatio + (1.0f - groundWcRatio) * airborneBlend;
    }
    const float wcEff = s->wc * groundScale;

    const float uP = (wcEff * (r - omegaFilt)) / b0;
    const float uD = (-omegaDotMeas * groundScale) / b0;
    const float uF = rDot / b0;
    const float deltaU = uP + uD + uF; // == ((wcEff * (r - omegaFilt) + rDot) - omegaDotMeas * groundScale) / b0
    const float uTotal = constrainf(uMeasFilt + deltaU, -sumLimitNorm, sumLimitNorm);

    if (!floatIsFiniteBits(uTotal)) {
        ladrcResetAxisState(axis);
        if (pTermOut) {
            *pTermOut = 0.0f;
        }
        if (dTermOut) {
            *dTermOut = 0.0f;
        }
        if (fTermOut) {
            *fTermOut = 0.0f;
        }
        if (distTermOut) {
            *distTermOut = 0.0f;
        }
        return 0.0f;
    }

    s->deltaUNorm = deltaU;
    s->uCmdNorm = uTotal;
    s->uRealNorm = uTotal;
    s->pendingTimeUpdate = true;

    s->pTermSum = uP * PID_MIXER_SCALING;
    s->dTermSum = uD * PID_MIXER_SCALING;
    s->fTermSum = uF * PID_MIXER_SCALING;
    s->distTermSum = uMeasFilt * PID_MIXER_SCALING;

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
        DEBUG_SET(DEBUG_LADRC, 2, lrintf(s->deltaUNorm * PID_MIXER_SCALING));
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

static void FAST_CODE finalizeAxisPostMixer(ladrcRuntimeAxis_t *s, float clampedScale)
{
    if (!s->pendingTimeUpdate) {
        return;
    }
    s->pendingTimeUpdate = false;
    const float uAct = s->feedbackUpdated ? s->uActualFeedback : (s->uCmdNorm * clampedScale);
    s->feedbackUpdated = false;
    if (!floatIsFiniteBits(uAct)) {
        resetAxisFilters(s);
        s->uRealNorm = 0.0f;
        return;
    }
    s->uRealNorm = uAct;

    if (s->resetMode == LADRC_RESET_OBSERVER_FULL) {
        s->actModelPt1.state = 0.0f;
        s->actModelSlew.state = 0.0f;
        s->uModelOut = 0.0f;
        return;
    }

    // Fallback actuator model: rate-limited slew + PT1 lag with tau_act
    const float slewStep = constrainf(uAct - s->actModelSlew.state, -s->actModelSlew.slewLimit, s->actModelSlew.slewLimit);
    s->actModelSlew.state += slewStep;
    s->uModelOut = pt1FilterApply(&s->actModelPt1, s->actModelSlew.state);
    if (!floatIsFiniteBits(s->uModelOut)) {
        s->actModelPt1.state = 0.0f;
        s->actModelSlew.state = 0.0f;
        s->uModelOut = 0.0f;
    }
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
        finalizeAxisPostMixer(&ladrcRuntime[axis], clampedScale);
    }
}

void ladrcUpdateSingleStep(int axis, float setpointDps, float gyroDps, float pidSumLimit,
                           float b0Scale, float mixerAuthorityScale)
{
    ladrcStepAxis(axis, setpointDps, gyroDps, pidSumLimit, b0Scale, NULL, NULL, NULL, NULL);
    if (axis < 0 || axis >= XYZ_AXIS_COUNT) {
        return;
    }
    const ladrcProfile_t *profile = ladrcProfiles(activeLadrcProfileIndex);
    const float clampedScale = profile->anti_windup ? constrainf(mixerAuthorityScale, 0.0f, 1.0f) : 1.0f;
    finalizeAxisPostMixer(&ladrcRuntime[axis], clampedScale);
}

static float computeAxisAllocationScale(const float *baseMix, const float *axisMix, uint8_t motorCount, float spanLimit)
{
    float scale = 1.0f;
    for (uint8_t i = 0; i < motorCount; i++) {
        for (uint8_t j = 0; j < motorCount; j++) {
            const float diffAxis = axisMix[i] - axisMix[j];
            if (diffAxis > 1e-6f) {
                const float headroom = spanLimit - (baseMix[i] - baseMix[j]);
                const float maxS = (headroom > 0.0f) ? (headroom / diffAxis) : 0.0f;
                if (maxS < scale) {
                    scale = maxS;
                }
            }
        }
    }
    return constrainf(scale, 0.0f, 1.0f);
}

static float computeMotorBoundsAxisScale(const float (*mixerRows)[4], const float *baseMix, const float *axisMix,
                                         uint8_t motorCount, float thr)
{
    float scale = 1.0f;
    for (uint8_t i = 0; i < motorCount; i++) {
        const float offset = thr * mixerRows[i][0] + baseMix[i];
        if (axisMix[i] > 1e-6f) {
            const float headroom = 1.0f - offset;
            const float maxS = (headroom > 0.0f) ? (headroom / axisMix[i]) : 0.0f;
            if (maxS < scale) {
                scale = maxS;
            }
        } else if (axisMix[i] < -1e-6f) {
            const float headroom = offset;
            const float maxS = (headroom > 0.0f) ? (headroom / (-axisMix[i])) : 0.0f;
            if (maxS < scale) {
                scale = maxS;
            }
        }
    }
    return constrainf(scale, 0.0f, 1.0f);
}

static void FAST_CODE indiAllocatePriorityMixInternal(const float (*mixerRows)[4], uint8_t motorCount,
                                                    float throttleIn, int8_t mixSign, bool airmodeEnabled,
                                                    bool restrictYawThrottleShift,
                                                    float rollCmd, float pitchCmd, float yawCmd,
                                                    float *motorMixOut, float *throttleOut,
                                                    float *mixMinOut, float *mixMaxOut, float *rawRangeOut)
{
    if (!mixerRows || !motorMixOut || motorCount == 0) {
        if (throttleOut) {
            *throttleOut = constrainf(throttleIn, 0.0f, 1.0f);
        }
        if (mixMinOut) {
            *mixMinOut = 0.0f;
        }
        if (mixMaxOut) {
            *mixMaxOut = 0.0f;
        }
        if (rawRangeOut) {
            *rawRangeOut = 0.0f;
        }
        return;
    }

    const uint8_t count = MIN(motorCount, (uint8_t)MAX_SUPPORTED_MOTORS);
    const float signF = (mixSign < 0) ? -1.0f : 1.0f;
    float rollMix[MAX_SUPPORTED_MOTORS];
    float pitchMix[MAX_SUPPORTED_MOTORS];
    float yawMix[MAX_SUPPORTED_MOTORS];
    float zeros[MAX_SUPPORTED_MOTORS];
    float rpMix[MAX_SUPPORTED_MOTORS];

    float rawMin = 0.0f;
    float rawMax = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        zeros[i] = 0.0f;
        rollMix[i] = rollCmd * mixerRows[i][1] * signF;
        pitchMix[i] = pitchCmd * mixerRows[i][2] * signF;
        yawMix[i] = yawCmd * mixerRows[i][3] * signF;
        const float raw = rollMix[i] + pitchMix[i] + yawMix[i];
        if (raw > rawMax) {
            rawMax = raw;
        } else if (raw < rawMin) {
            rawMin = raw;
        }
    }

    if (rawRangeOut) {
        *rawRangeOut = rawMax - rawMin;
    }

    // Strict priority: Roll > Pitch > Yaw inside [0, 1] motor span (max - min <= 1.0)
    const float rollSpanScale = computeAxisAllocationScale(zeros, rollMix, count, 1.0f);
    for (uint8_t i = 0; i < count; i++) {
        rollMix[i] *= rollSpanScale;
    }

    const float pitchSpanScale = computeAxisAllocationScale(rollMix, pitchMix, count, 1.0f);
    for (uint8_t i = 0; i < count; i++) {
        pitchMix[i] *= pitchSpanScale;
        rpMix[i] = rollMix[i] + pitchMix[i];
    }

    float thrLowRp = 0.0f;
    float thrHighRp = 1.0f;
    for (uint8_t i = 0; i < count; i++) {
        const float ti = mixerRows[i][0];
        if (ti > 1e-6f) {
            thrLowRp = MAX(thrLowRp, -rpMix[i] / ti);
            thrHighRp = MIN(thrHighRp, (1.0f - rpMix[i]) / ti);
        }
    }

    float thrRp = constrainf(throttleIn, 0.0f, 1.0f);
    if (airmodeEnabled) {
        if (thrLowRp <= thrHighRp) {
            thrRp = constrainf(thrRp, thrLowRp, thrHighRp);
        } else {
            thrRp = constrainf(thrRp, 0.0f, 1.0f);
        }
    } else {
        thrRp = constrainf(MIN(thrRp, thrHighRp), 0.0f, 1.0f);
        bool violates = false;
        for (uint8_t i = 0; i < count; i++) {
            const float out = thrRp * mixerRows[i][0] + rpMix[i];
            if (out < -1e-6f || out > 1.0f + 1e-6f) {
                violates = true;
                break;
            }
        }
        if (violates) {
            const float rScale = computeMotorBoundsAxisScale(mixerRows, zeros, rollMix, count, thrRp);
            for (uint8_t i = 0; i < count; i++) {
                rollMix[i] *= rScale;
            }
            const float pScale = computeMotorBoundsAxisScale(mixerRows, rollMix, pitchMix, count, thrRp);
            for (uint8_t i = 0; i < count; i++) {
                rpMix[i] = rollMix[i] + pScale * pitchMix[i];
            }
        }
    }

    // Yaw allocation: span <= 1.0f AND per-motor [0, 1] inside bounded collective shift around thrRp
    float yawScale = computeAxisAllocationScale(rpMix, yawMix, count, 1.0f);
    float thrYawMin = 0.0f;
    float thrYawMax = 1.0f;
    if (restrictYawThrottleShift) {
        thrYawMin = MAX(0.0f, thrRp - ((thrRp > 0.50f) ? 0.0f : 0.10f));
        thrYawMax = MIN(1.0f, thrRp + ((airmodeEnabled && thrRp < 0.20f) ? 0.10f : 0.0f));
    }

    for (uint8_t i = 0; i < count; i++) {
        const float ti = mixerRows[i][0];
        if (yawMix[i] > 1e-6f) {
            const float headroomHigh = 1.0f - (thrYawMin * ti + rpMix[i]);
            const float maxS = (headroomHigh > 0.0f) ? (headroomHigh / yawMix[i]) : 0.0f;
            if (maxS < yawScale) {
                yawScale = maxS;
            }
        } else if (yawMix[i] < -1e-6f) {
            const float headroomLow = thrYawMax * ti + rpMix[i];
            const float maxS = (headroomLow > 0.0f) ? (headroomLow / (-yawMix[i])) : 0.0f;
            if (maxS < yawScale) {
                yawScale = maxS;
            }
        }
    }
    yawScale = constrainf(yawScale, 0.0f, 1.0f);

    float mixMin = 0.0f;
    float mixMax = 0.0f;
    float reqThrLow = thrYawMin;
    float reqThrHigh = thrYawMax;
    for (uint8_t i = 0; i < count; i++) {
        const float signedOut = rpMix[i] + yawScale * yawMix[i];
        const float ti = mixerRows[i][0];
        if (ti > 1e-6f) {
            reqThrLow = MAX(reqThrLow, -signedOut / ti);
            reqThrHigh = MIN(reqThrHigh, (1.0f - signedOut) / ti);
        }
        motorMixOut[i] = signedOut * signF;
        if (signedOut > mixMax) {
            mixMax = signedOut;
        } else if (signedOut < mixMin) {
            mixMin = signedOut;
        }
    }

    if (throttleOut) {
        *throttleOut = (reqThrLow <= reqThrHigh) ? constrainf(thrRp, reqThrLow, reqThrHigh) : thrRp;
    }
    if (mixMinOut) {
        *mixMinOut = mixMin;
    }
    if (mixMaxOut) {
        *mixMaxOut = mixMax;
    }
}

void FAST_CODE indiAllocatePriorityMixWithThrottle(const float (*mixerRows)[4], uint8_t motorCount,
                                                   float throttleIn, int8_t mixSign, bool airmodeEnabled,
                                                   float rollCmd, float pitchCmd, float yawCmd,
                                                   float *motorMixOut, float *throttleOut,
                                                   float *mixMinOut, float *mixMaxOut, float *rawRangeOut)
{
    indiAllocatePriorityMixInternal(mixerRows, motorCount, throttleIn, mixSign, airmodeEnabled, true,
                                    rollCmd, pitchCmd, yawCmd, motorMixOut, throttleOut,
                                    mixMinOut, mixMaxOut, rawRangeOut);
}

void FAST_CODE indiAllocatePriorityMix(const float (*mixerRows)[4], uint8_t motorCount,
                                       float rollCmd, float pitchCmd, float yawCmd,
                                       float *motorMixOut, float *mixMinOut, float *mixMaxOut, float *rawRangeOut)
{
    float dummyThrottle = 0.5f;
    indiAllocatePriorityMixInternal(mixerRows, motorCount, 0.5f, 1, true, false,
                                    rollCmd, pitchCmd, yawCmd, motorMixOut, &dummyThrottle,
                                    mixMinOut, mixMaxOut, rawRangeOut);
}

void FAST_CODE indiProjectMotorOutputsToAxes(const float *motorOutputs, const float (*mixerRows)[4],
                                             uint8_t motorCount, float axesOut[XYZ_AXIS_COUNT])
{
    if (!axesOut) {
        return;
    }
    axesOut[FD_ROLL] = 0.0f;
    axesOut[FD_PITCH] = 0.0f;
    axesOut[FD_YAW] = 0.0f;
    if (!motorOutputs || !mixerRows || motorCount == 0) {
        return;
    }

    const uint8_t count = MIN(motorCount, (uint8_t)MAX_SUPPORTED_MOTORS);
    float mat[4][5];
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            mat[r][c] = 0.0f;
        }
        mat[r][4] = 0.0f;
    }

    float colNormSq[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    for (uint8_t i = 0; i < count; i++) {
        const float m = motorOutputs[i];
        for (int r = 0; r < 4; r++) {
            const float mr = mixerRows[i][r];
            colNormSq[r] += mr * mr;
            mat[r][4] += mr * m;
            for (int c = 0; c < 4; c++) {
                mat[r][c] += mr * mixerRows[i][c];
            }
        }
    }
    for (int r = 0; r < 4; r++) {
        mat[r][r] += 1e-5f;
    }

    // 4x4 Gauss-Jordan elimination with partial pivoting
    for (int col = 0; col < 4; col++) {
        int pivot = col;
        float maxAbs = fabsf(mat[col][col]);
        for (int row = col + 1; row < 4; row++) {
            const float val = fabsf(mat[row][col]);
            if (val > maxAbs) {
                maxAbs = val;
                pivot = row;
            }
        }
        if (pivot != col) {
            for (int c = col; c < 5; c++) {
                const float tmp = mat[col][c];
                mat[col][c] = mat[pivot][c];
                mat[pivot][c] = tmp;
            }
        }
        const float diag = mat[col][col];
        if (fabsf(diag) < 1e-9f) {
            continue;
        }
        const float invDiag = 1.0f / diag;
        for (int c = col; c < 5; c++) {
            mat[col][c] *= invDiag;
        }
        for (int row = 0; row < 4; row++) {
            if (row != col) {
                const float factor = mat[row][col];
                if (factor != 0.0f) {
                    for (int c = col; c < 5; c++) {
                        mat[row][c] -= factor * mat[col][c];
                    }
                }
            }
        }
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        const float sol = mat[axis + 1][4];
        axesOut[axis] = (colNormSq[axis + 1] >= 1e-6f && floatIsFiniteBits(sol)) ? sol : 0.0f;
    }
}

const ladrcRuntimeAxis_t *ladrcGetAxisState(int axis)
{
    if (axis < 0 || axis >= XYZ_AXIS_COUNT) {
        return NULL;
    }
    return &ladrcRuntime[axis];
}
