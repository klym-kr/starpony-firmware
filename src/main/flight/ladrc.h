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

#include "common/axis.h"
#include "flight/pid.h"
#include "pg/pg.h"

#define LADRC_RATE_SCALE_TO_NORM   0.001f    // deg/s -> kdeg/s
#define LADRC_RATE_SCALE_FROM_NORM 1000.0f   // kdeg/s -> deg/s

typedef enum {
    LADRC_ORDER_1 = 1, // 2-state LESO: z1=rate, z2=total disturbance
    LADRC_ORDER_2 = 2, // 3-state LESO: z1=rate, z2=accel, z3=total disturbance
} ladrcOrder_e;

typedef enum {
    LADRC_RESET_NONE = 0,
    LADRC_RESET_DISTURBANCE = 1,   // In-flight: clear disturbance only (Order 1: z2; Order 2: z3), keep z1/z2 D-term active & predicted
    LADRC_RESET_OBSERVER_FULL = 2, // Pre-takeoff ground idle: clear z2 & z3 and skip prediction to prevent ground windup
} ladrcResetMode_e;

typedef struct ladrcAxisProfile_s {
    uint8_t order;      // ladrcOrder_e (1 or 2)
    uint16_t wc_deci;   // Controller bandwidth omega_c in 0.1 rad/s (e.g. 180 = 18.0 rad/s)
    uint16_t wo_deci;   // Observer bandwidth omega_o in 0.1 rad/s (e.g. 720 = 72.0 rad/s)
    uint16_t b0_deci;   // Nominal control gain b0 in 0.1 normalized units (kdeg/s^(order+1) per unit pidSum/1000)
    uint16_t td_hz;     // Setpoint tracking differentiator cutoff in Hz (0 = disabled)
} ladrcAxisProfile_t;

typedef struct ladrcProfile_s {
    uint8_t anti_windup;         // 1 = feed mixer-realized torque scale back into LESO + clamp disturbance
    uint8_t vbat_b0_comp;        // 1 = scale b0 dynamically with battery sag voltage
    uint8_t airspeed_sched;      // 1 = scale b0 quadratically with airspeed (fixed-wing)
    uint16_t airspeed_ref_cms;   // Reference cruise airspeed in cm/s for fixed-wing b0 scheduling
    uint16_t disturbance_limit;  // Max disturbance cancellation authority in pidSum units (0..1000)
    ladrcAxisProfile_t axis[XYZ_AXIS_COUNT];
    uint8_t sat_z3_inhibit;      // 1 = inhibit z3 disturbance growth while mixer is saturated/clipping (ADRC-033)
    uint8_t ground_wc_pct;       // % of wc active while grounded before liftoff (e.g. 20 = 20%, 0 = full wc)
} ladrcProfile_t;

PG_DECLARE_ARRAY(ladrcProfile_t, PID_PROFILE_COUNT, ladrcProfiles);

typedef struct ladrcRuntimeAxis_s {
    uint8_t order;
    float dt;
    float wc;
    float wo;
    float b0Nom;
    float b0Eff;
    float distLimitNorm;

    // Discrete pole-placed current-observer gains
    float lc1;
    float lc2;
    float lc3;

    // Tracking differentiator (unconditionally stable cascaded exact-ZOH 2nd-order critically damped filter)
    float tdOmega;
    float tdAlpha;
    float tdR1; // Stage 1 filter state (w)
    float tdR2; // Stage 2 filter state (r1)

    // LESO states (rates in kdeg/s)
    float z1; // Tracked rate
    float z2; // Order 1: disturbance; Order 2: estimated angular accel
    float z3; // Order 2: total disturbance

    // Step diagnostics & command memory
    float yLastNorm;
    float rLastNorm;
    float rDotLast;
    float z1Last;
    float z2Last;
    float z3Last;
    float uCmdNorm;
    float uRealNorm;
    float pTermSum;
    float dTermSum;
    float fTermSum;
    float distTermSum;
    bool pendingTimeUpdate;
    uint8_t resetMode; // ladrcResetMode_e
} ladrcRuntimeAxis_t;

const ladrcRuntimeAxis_t *ladrcGetAxisState(int axis);

void ladrcInit(uint8_t profileIndex, float dT);
void ladrcResetState(void);
void ladrcResetAxisState(int axis);
void ladrcResetAxisDisturbance(int axis);
void ladrcResetDisturbance(void);

void ladrcSetExternalB0Scale(float scale);
float ladrcGetExternalB0Scale(void);
float ladrcComputeB0Scale(const ladrcProfile_t *profile, float vbatRatio, float airspeedCms, bool fixedWing);

float ladrcStepAxis(int axis, float setpointDps, float gyroDps, float pidSumLimit, float b0Scale,
                    float *pTermOut, float *dTermOut, float *fTermOut, float *distTermOut);
float ladrcStepAxisEx(int axis, float setpointDps, float gyroDps, float pidSumLimit, float b0Scale,
                      float tdWeight, ladrcResetMode_e resetMode,
                      float *pTermOut, float *dTermOut, float *fTermOut, float *distTermOut);

void ladrcPostMixerUpdate(float mixerAuthorityScale, bool stabilised);

void ladrcSetMixerSaturated(bool saturated);
bool ladrcIsMixerSaturated(void);
void ladrcUpdateLiftoffState(float commandedThrottle, float gyroPeakDps, float dT);
void ladrcSetAirborne(bool airborne);
bool ladrcIsAirborne(void);

void ladrcUpdateSingleStep(int axis, float setpointDps, float gyroDps, float pidSumLimit,
                           float b0Scale, float mixerAuthorityScale);

const ladrcRuntimeAxis_t *ladrcGetAxisState(int axis);
