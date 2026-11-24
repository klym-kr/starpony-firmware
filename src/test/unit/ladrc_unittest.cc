/*
 * This file is part of St4rpony.
 *
 * St4rpony is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 */

#include <cmath>
#include <cstdint>

#include "gtest/gtest.h"

extern "C" {
    #include "platform.h"
    #include "build/debug.h"
    #include "common/axis.h"
    #include "common/maths.h"
    #include "flight/ladrc.h"
    #include "flight/pid.h"
    #include "pg/pg.h"
}

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode = 0;

class LadrcTest : public ::testing::Test {
protected:
    void SetUp() override {
        pgResetAll();
        debugMode = 0;
    }
};

TEST_F(LadrcTest, testFirstOrderStepAndDisturbanceRejection) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->anti_windup = 1;
    prof->disturbance_limit = 800;
    prof->axis[FD_YAW].order = LADRC_ORDER_1;
    prof->axis[FD_YAW].wc_deci = 150; // 15 rad/s
    prof->axis[FD_YAW].wo_deci = 600; // 60 rad/s
    prof->axis[FD_YAW].b0_deci = 100; // 10.0 kdeg/s^2 per unit norm
    prof->axis[FD_YAW].td_hz = 0;

    const float dt = 0.0005f; // 2 kHz
    ladrcInit(0, dt);

    // Plant: dy_kdeg/dt = b0_true * u + disturbance_kdeg_s2
    // Include 25% b0 model mismatch and strong constant external torque disturbance!
    const float b0True = 12.5f;
    const float extDisturbance = 2.0f; // equivalent to 2000 deg/s^2 torque offset
    float yKdeg = 0.0f;
    const float setpointDps = 200.0f;

    for (int step = 0; step < 3000; step++) { // 1.5 seconds
        const float sum = ladrcStepAxis(FD_YAW, setpointDps, yKdeg * 1000.0f, 500.0f, 1.0f,
                                        nullptr, nullptr, nullptr, nullptr);
        const float uNorm = sum / PID_MIXER_SCALING;
        yKdeg += dt * (b0True * uNorm + extDisturbance);
        ladrcPostMixerUpdate(1.0f, true);
    }

    const float finalRateDps = yKdeg * 1000.0f;
    EXPECT_NEAR(finalRateDps, setpointDps, 0.25f);

    const ladrcRuntimeAxis_t *st = ladrcGetAxisState(FD_YAW);
    ASSERT_NE(st, nullptr);
    EXPECT_NEAR(st->z1 * LADRC_RATE_SCALE_FROM_NORM, setpointDps, 0.25f);
    // At steady state, uNorm = -extDisturbance / b0True = -0.16 -> pidSum = -160
    EXPECT_NEAR(st->distTermSum, -160.0f, 5.0f);
}

TEST_F(LadrcTest, testSecondOrderWithMotorLagAndTrackingDifferentiator) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->anti_windup = 1;
    prof->disturbance_limit = 800;
    prof->axis[FD_ROLL].order = LADRC_ORDER_2;
    prof->axis[FD_ROLL].wc_deci = 200; // 20 rad/s
    prof->axis[FD_ROLL].wo_deci = 800; // 80 rad/s
    prof->axis[FD_ROLL].b0_deci = 5000; // 500.0
    prof->axis[FD_ROLL].td_hz = 60;

    const float dt = 0.00025f; // 4 kHz
    ladrcInit(0, dt);

    // 2nd-order rotational plant with motor lag tau_m = 20 ms + bias torque:
    // tau_m * d2y/dt2 + dy/dt = K_plant * u + d_bias
    const float tauM = 0.020f;
    const float kPlant = 10.0f; // 10 kdeg/s^2 per unit norm -> b0_true = kPlant / tauM = 500
    const float dBias = 1.2f;
    float yKdeg = 0.0f;
    float yDot = 0.0f;
    const float setpointDps = 350.0f;

    for (int step = 0; step < 6000; step++) { // 1.5 seconds
        const float sum = ladrcStepAxis(FD_ROLL, setpointDps, yKdeg * 1000.0f, 500.0f, 1.0f,
                                        nullptr, nullptr, nullptr, nullptr);
        const float uNorm = sum / PID_MIXER_SCALING;
        const float y2Dot = (kPlant * uNorm + dBias - yDot) / tauM;
        yKdeg += dt * yDot + 0.5f * dt * dt * y2Dot;
        yDot += dt * y2Dot;
        ladrcPostMixerUpdate(1.0f, true);
    }

    EXPECT_NEAR(yKdeg * 1000.0f, setpointDps, 1.0f);
}

TEST_F(LadrcTest, testSaturationAntiWindupBoundsDisturbanceAndRecoversCleanly) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->anti_windup = 1;
    prof->disturbance_limit = 350; // 0.35 normalized cap
    prof->axis[FD_PITCH].order = LADRC_ORDER_2;
    prof->axis[FD_PITCH].wc_deci = 180;
    prof->axis[FD_PITCH].wo_deci = 720;
    prof->axis[FD_PITCH].b0_deci = 4000;
    prof->axis[FD_PITCH].td_hz = 0;

    const float dt = 0.0005f;
    ladrcInit(0, dt);

    // Hold physical craft stationary (y = 0) while commanding 500 deg/s and reporting 30% mixer authority
    for (int step = 0; step < 1000; step++) {
        ladrcUpdateSingleStep(FD_PITCH, 500.0f, 0.0f, 500.0f, 1.0f, 0.3f);
    }

    const ladrcRuntimeAxis_t *st = ladrcGetAxisState(FD_PITCH);
    ASSERT_NE(st, nullptr);
    // Disturbance cancellation term must remain strictly bounded by disturbance_limit!
    EXPECT_LE(fabsf(st->distTermSum), 350.5f);
    EXPECT_NEAR(st->uRealNorm, st->uCmdNorm * 0.3f, 1e-5f);

    // Resetting clears all observer states
    ladrcResetState();
    EXPECT_FLOAT_EQ(st->z1, 0.0f);
    EXPECT_FLOAT_EQ(st->z2, 0.0f);
    EXPECT_FLOAT_EQ(st->z3, 0.0f);
}

TEST_F(LadrcTest, testB0ScalingForVbatAndFixedWingAirspeed) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->vbat_b0_comp = 1;
    prof->airspeed_sched = 1;
    prof->airspeed_ref_cms = 1500; // 15 m/s

    // Multirotor at 90% nominal voltage -> linear 0.90 (hover linearization: dT/du ~ V^1)
    EXPECT_NEAR(ladrcComputeB0Scale(prof, 0.9f, 0.0f, false), 0.9f, 1e-4f);
    // Multirotor extreme sag clamped at 0.70
    EXPECT_NEAR(ladrcComputeB0Scale(prof, 0.5f, 0.0f, false), 0.70f, 1e-4f);

    // Fixed-wing servos are BEC-powered -> Vbat sag must NOT scale fixed-wing b0
    EXPECT_NEAR(ladrcComputeB0Scale(prof, 0.75f, 1500.0f, true), 1.0f, 1e-4f);

    // Fixed-wing at 30 m/s (2x reference speed) -> (2.0)^2 = 4.0
    EXPECT_NEAR(ladrcComputeB0Scale(prof, 0.8f, 3000.0f, true), 4.0f, 1e-4f);

    // Fixed-wing slow speed (5 m/s -> 1/3 ref -> 1/9 clamped at 0.25)
    EXPECT_NEAR(ladrcComputeB0Scale(prof, 1.0f, 500.0f, true), 0.25f, 1e-4f);
}

TEST_F(LadrcTest, testOrder2ResetZeroesAccelerationZ2AndPreventsGroundWindup) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->anti_windup = 1;
    prof->disturbance_limit = 500;
    prof->axis[FD_ROLL].order = LADRC_ORDER_2;
    prof->axis[FD_ROLL].wc_deci = 105;
    prof->axis[FD_ROLL].wo_deci = 420;
    prof->axis[FD_ROLL].b0_deci = 1900;
    prof->axis[FD_ROLL].td_hz = 0;

    ladrcInit(0, 0.0005f);

    // Phase 1: Simulate stationary craft on ground while pre-takeoff full observer reset is active
    for (int step = 0; step < 200; step++) {
        float pOut = 0.0f, dOut = 0.0f, fOut = 0.0f, distOut = 0.0f;
        const float sum = ladrcStepAxisEx(FD_ROLL, 200.0f, 0.0f, 500.0f, 1.0f, 0.0f, LADRC_RESET_OBSERVER_FULL,
                                          &pOut, &dOut, &fOut, &distOut);
        EXPECT_FLOAT_EQ(dOut, 0.0f);
        EXPECT_FLOAT_EQ(distOut, 0.0f);
        EXPECT_FLOAT_EQ(fOut, 0.0f);
        EXPECT_FLOAT_EQ(sum, pOut);
        ladrcPostMixerUpdate(0.5f, true);
    }

    const ladrcRuntimeAxis_t *st = ladrcGetAxisState(FD_ROLL);
    ASSERT_NE(st, nullptr);
    EXPECT_FLOAT_EQ(st->z2, 0.0f);
    EXPECT_FLOAT_EQ(st->z3, 0.0f);
    EXPECT_FLOAT_EQ(st->distTermSum, 0.0f);

    // Phase 2: Build up actuator feedback without reset, then trigger ground full reset -> Sum clears immediately
    for (int step = 0; step < 200; step++) {
        ladrcStepAxisEx(FD_ROLL, 200.0f, 0.0f, 500.0f, 1.0f, 0.0f, LADRC_RESET_NONE, nullptr, nullptr, nullptr, nullptr);
        ladrcPostMixerUpdate(1.0f, true);
    }
    EXPECT_GT(fabsf(st->distTermSum), 1.0f);

    float pOut = 0.0f, dOut = 0.0f, fOut = 0.0f, distOut = 0.0f;
    const float resetSum = ladrcStepAxisEx(FD_ROLL, 200.0f, 0.0f, 500.0f, 1.0f, 0.0f, LADRC_RESET_OBSERVER_FULL,
                                           &pOut, &dOut, &fOut, &distOut);
    EXPECT_FLOAT_EQ(dOut, 0.0f);
    EXPECT_FLOAT_EQ(distOut, 0.0f);
    EXPECT_FLOAT_EQ(resetSum, pOut);
    EXPECT_FLOAT_EQ(st->z2, 0.0f);
    EXPECT_FLOAT_EQ(st->z3, 0.0f);
}

TEST_F(LadrcTest, testOrder2DisturbanceOnlyResetPreservesDampingZ2) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->manual_override = 1;
    prof->anti_windup = 1;
    prof->disturbance_limit = 500;
    prof->axis[FD_ROLL].order = LADRC_ORDER_2;
    prof->axis[FD_ROLL].wc_deci = 105;
    prof->axis[FD_ROLL].wo_deci = 420;
    prof->axis[FD_ROLL].b0_deci = 1900;
    prof->axis[FD_ROLL].td_hz = 0;
    prof->axis[FD_YAW].order = LADRC_ORDER_1;
    prof->axis[FD_YAW].wc_deci = 75;
    prof->axis[FD_YAW].wo_deci = 300;
    prof->axis[FD_YAW].b0_deci = 80;
    prof->axis[FD_YAW].td_hz = 0;

    ladrcInit(0, 0.0005f);

    // Build both gyro rate ramp and actuator feedback on Roll and Yaw
    for (int step = 0; step < 100; step++) {
        const float gyroRoll = 2.0f * (float)step;
        ladrcStepAxisEx(FD_ROLL, 200.0f, gyroRoll, 500.0f, 1.0f, 0.0f, LADRC_RESET_NONE,
                        nullptr, nullptr, nullptr, nullptr);
        ladrcStepAxisEx(FD_YAW, 150.0f, 0.0f, 400.0f, 1.0f, 0.0f, LADRC_RESET_NONE,
                        nullptr, nullptr, nullptr, nullptr);
        ladrcPostMixerUpdate(1.0f, true);
    }

    const ladrcRuntimeAxis_t *rollSt = ladrcGetAxisState(FD_ROLL);
    const ladrcRuntimeAxis_t *yawSt = ladrcGetAxisState(FD_YAW);
    ASSERT_NE(rollSt, nullptr);
    ASSERT_NE(yawSt, nullptr);
    EXPECT_GT(fabsf(rollSt->distTermSum), 1.0f);
    EXPECT_GT(fabsf(yawSt->distTermSum), 1.0f);

    // In-flight disturbance-only reset during positive angular kick on Roll:
    // Clears u_meas_filt (distOut == 0) while preserving active omegaDotMeas D-term damping!
    float rP = 0.0f, rD = 0.0f, rF = 0.0f, rDist = 0.0f;
    float yP = 0.0f, yD = 0.0f, yF = 0.0f, yDist = 0.0f;
    const float rSum = ladrcStepAxisEx(FD_ROLL, 0.0f, 500.0f, 500.0f, 1.0f, 0.0f, LADRC_RESET_DISTURBANCE,
                                       &rP, &rD, &rF, &rDist);
    ladrcStepAxisEx(FD_YAW, 0.0f, 0.0f, 400.0f, 1.0f, 0.0f, LADRC_RESET_DISTURBANCE,
                    &yP, &yD, &yF, &yDist);

    EXPECT_FLOAT_EQ(rDist, 0.0f);
    EXPECT_FLOAT_EQ(rSum, fmaxf(-500.0f, fminf(500.0f, rP + rD + rF)));
    EXPECT_FLOAT_EQ(rollSt->z3, 0.0f);
    EXPECT_GT(rollSt->z2, 0.0f);
    EXPECT_LT(rD, 0.0f); // D-term opposes positive angular acceleration

    EXPECT_FLOAT_EQ(yDist, 0.0f);
    EXPECT_FLOAT_EQ(yawSt->distTermSum, 0.0f);

    // Explicit ladrcResetAxisDisturbance clears feedback accumulator without wiping gyro derivative state
    ladrcStepAxisEx(FD_ROLL, 0.0f, 520.0f, 500.0f, 1.0f, 0.0f, LADRC_RESET_NONE,
                    nullptr, nullptr, nullptr, nullptr);
    ladrcPostMixerUpdate(1.0f, true);
    ladrcResetAxisDisturbance(FD_ROLL);
    EXPECT_FLOAT_EQ(rollSt->distTermSum, 0.0f);
    EXPECT_NE(rollSt->z2, 0.0f);
}

TEST_F(LadrcTest, testTrackingDifferentiatorBypassAndWeighting) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->manual_override = 1;
    prof->axis[FD_PITCH].order = LADRC_ORDER_2;
    prof->axis[FD_PITCH].wc_deci = 100;
    prof->axis[FD_PITCH].wo_deci = 400;
    prof->axis[FD_PITCH].b0_deci = 2000;
    prof->axis[FD_PITCH].td_hz = 20;

    ladrcInit(0, 0.0005f);

    // With tdWeight = 0.0f (outer attitude loop active), step setpoint reaches P term immediately with zero TD feedforward lag
    float pZero = 0.0f, dZero = 0.0f, fZero = 0.0f, distZero = 0.0f;
    ladrcStepAxisEx(FD_PITCH, 300.0f, 0.0f, 500.0f, 1.0f, 0.0f, LADRC_RESET_NONE,
                    &pZero, &dZero, &fZero, &distZero);
    EXPECT_FLOAT_EQ(fZero, 0.0f);
    EXPECT_GT(pZero, 0.5f);

    // Compare against full TD filtering from zero initial state: TD starts near zero and ramps smoothly
    ladrcInit(0, 0.0005f);
    float pFull = 0.0f, dFull = 0.0f, fFull = 0.0f, distFull = 0.0f;
    ladrcStepAxisEx(FD_PITCH, 300.0f, 0.0f, 500.0f, 1.0f, 1.0f, LADRC_RESET_NONE,
                    &pFull, &dFull, &fFull, &distFull);
    EXPECT_GT(pZero, pFull * 10.0f);
    EXPECT_GT(fFull, 0.0f);
}

TEST_F(LadrcTest, testSaturationDisturbanceGrowthInhibit) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->manual_override = 1;
    prof->sat_z3_inhibit = 1;
    prof->anti_windup = 1;
    prof->disturbance_limit = 400;
    prof->axis[FD_ROLL].order = LADRC_ORDER_2;
    prof->axis[FD_ROLL].wc_deci = 150;
    prof->axis[FD_ROLL].wo_deci = 600;
    prof->axis[FD_ROLL].b0_deci = 200;
    prof->axis[FD_ROLL].td_hz = 0;

    const float dt = 0.0005f;
    ladrcInit(0, dt);

    // With actuator saturated (authority = 0.25) and commanding 500 dps while stationary:
    // uMeasFilt remains strictly bounded within [-disturbance_limit, +disturbance_limit] and tracks realized 0.25 scale!
    ladrcSetMixerSaturated(true);
    EXPECT_TRUE(ladrcIsMixerSaturated());

    for (int step = 0; step < 400; step++) {
        ladrcStepAxis(FD_ROLL, 500.0f, 0.0f, 500.0f, 1.0f, nullptr, nullptr, nullptr, nullptr);
        ladrcPostMixerUpdate(0.25f, true);
    }
    const ladrcRuntimeAxis_t *st = ladrcGetAxisState(FD_ROLL);
    ASSERT_NE(st, nullptr);
    EXPECT_LE(fabsf(st->distTermSum), 400.0f + 1e-3f);
    EXPECT_LE(fabsf(st->uRealNorm), 0.25f + 1e-3f);
}

TEST_F(LadrcTest, testLiftoffStateAndGroundWcAttenuation) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->manual_override = 1;
    prof->ground_wc_pct = 50; // 50% wc on ground
    prof->axis[FD_ROLL].order = LADRC_ORDER_2;
    prof->axis[FD_ROLL].wc_deci = 200; // 20 rad/s -> wcEff = 10 rad/s on ground
    prof->axis[FD_ROLL].wo_deci = 800;
    prof->axis[FD_ROLL].b0_deci = 2000;
    prof->axis[FD_ROLL].td_hz = 0;

    const float dt = 0.0005f; // 2 kHz -> 500 us
    ladrcInit(0, dt);

    // Explicitly set grounded state
    ladrcSetAirborne(false);
    EXPECT_FALSE(ladrcIsAirborne());

    // On ground with ground_wc_pct = 50:
    // INDI uP = (wcEff * (r - omegaFilt)) / b0, so uP scales linearly 50%
    float pGrounded = 0.0f;
    ladrcStepAxisEx(FD_ROLL, 100.0f, 0.0f, 500.0f, 1.0f, 0.0f, LADRC_RESET_NONE,
                    &pGrounded, nullptr, nullptr, nullptr);

    // Reset and test airborne
    ladrcInit(0, dt);
    ladrcSetAirborne(true);
    EXPECT_TRUE(ladrcIsAirborne());

    float pAirborne = 0.0f;
    ladrcStepAxisEx(FD_ROLL, 100.0f, 0.0f, 500.0f, 1.0f, 0.0f, LADRC_RESET_NONE,
                    &pAirborne, nullptr, nullptr, nullptr);

    EXPECT_NEAR(pGrounded / pAirborne, 0.50f, 0.02f);

    // Now test liftoff state transition via throttle trigger
    ladrcSetAirborne(false);
    EXPECT_FALSE(ladrcIsAirborne());

    // Low throttle: stays grounded
    ladrcUpdateLiftoffState(0.10f, 0.0f, dt);
    EXPECT_FALSE(ladrcIsAirborne());

    // Throttle past liftoff threshold (>= 0.20): triggers airborne transition
    ladrcUpdateLiftoffState(0.25f, 0.0f, dt);
    EXPECT_TRUE(ladrcIsAirborne());

    // Over 200 ms (400 steps at 2kHz), airborneBlend ramps to 1.0
    for (int i = 0; i < 400; i++) {
        ladrcUpdateLiftoffState(0.25f, 0.0f, dt);
    }
    // Authority should now be 100%
    float pRamped = 0.0f;
    ladrcStepAxisEx(FD_ROLL, 100.0f, 0.0f, 500.0f, 1.0f, 0.0f, LADRC_RESET_NONE,
                    &pRamped, nullptr, nullptr, nullptr);
    EXPECT_NEAR(pRamped, pAirborne, 1.0f);
}

TEST_F(LadrcTest, testPhaseMatchedBiquadLpfZeroPhaseDiffGyroVsDshotRpm) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->prop_size_deci_in = 50; // 5.0" -> f_indi = 90 Hz
    prof->auw_centi_kg = 65;      // 0.65 kg -> b0_roll = 35.0 kdeg/s^2
    prof->agility_pct = 100;
    prof->authority_trim_pct = 100;
    prof->manual_override = 0;

    const float dt = 0.0003125f; // 3.2 kHz
    ladrcSetFixedWing(false);
    ladrcInit(0, dt);
    ladrcSetDshotFeedbackStatus(true, 4);

    // Feed an ideal instantaneous plant: omegaDot(t) = b0 * u(t)
    // With twin phase-matched 2nd-order Butterworth SVF Biquad LPFs at f_indi = 90 Hz:
    // Both omegaFilt and uMeasFilt undergo identical phase delay, so d(omegaFilt)/dt == b0 * uMeasFilt!
    const float b0 = ladrcGetAxisState(FD_ROLL)->b0Nom;
    float yKdeg = 0.0f;
    float maxDiff = 0.0f;
    for (int k = 0; k < 400; k++) {
        const float uSin = 0.3f * sinf(2.0f * M_PIf * 15.0f * (k * dt));
        yKdeg += dt * (b0 * uSin);
        ladrcSetAxisActuatorFeedback(FD_ROLL, uSin, uSin, true);
        ladrcStepAxisEx(FD_ROLL, 0.0f, yKdeg * 1000.0f, 500.0f, 1.0f, 0.0f, LADRC_RESET_NONE,
                        nullptr, nullptr, nullptr, nullptr);
        ladrcPostMixerUpdate(1.0f, true);

        if (k > 100) {
            const ladrcRuntimeAxis_t *st = ladrcGetAxisState(FD_ROLL);
            const float diff = fabsf(st->z2 - b0 * (st->distTermSum / PID_MIXER_SCALING));
            if (diff > maxDiff) {
                maxDiff = diff;
            }
        }
    }
    EXPECT_LT(maxDiff, 0.15f);
}

TEST_F(LadrcTest, testFiveInchLightQuadStepAndDisturbance) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->prop_size_deci_in = 50; // 5.0"
    prof->auw_centi_kg = 65;      // 0.65 kg
    prof->agility_pct = 100;
    prof->authority_trim_pct = 100;
    prof->manual_override = 0;
    prof->disturbance_limit = 800;

    const float dt = 0.0003125f; // 3.2 kHz
    ladrcSetFixedWing(false);
    ladrcInit(0, dt);
    ladrcSetDshotFeedbackStatus(true, 4);

    // 5" light quad: tau_m = 15 ms, 20% b0 mismatch + constant disturbance
    const float tauM = 0.015f;
    const float b0True = 42.0f; // nominal is 35.0
    const float dExt = 3.5f;    // 3500 deg/s^2 torque offset
    float yKdeg = 0.0f;
    float uMotor = 0.0f;
    const float setpointDps = 300.0f;

    for (int step = 0; step < 3200; step++) { // 1.0 s
        ladrcSetAxisActuatorFeedback(FD_ROLL, uMotor, uMotor, true);
        const float sum = ladrcStepAxis(FD_ROLL, setpointDps, yKdeg * 1000.0f, 500.0f, 1.0f,
                                        nullptr, nullptr, nullptr, nullptr);
        const float uCmd = sum / PID_MIXER_SCALING;
        uMotor += (dt / (dt + tauM)) * (uCmd - uMotor);
        yKdeg += dt * (b0True * uMotor + dExt);
        ladrcPostMixerUpdate(1.0f, true);
    }

    EXPECT_NEAR(yKdeg * 1000.0f, setpointDps, 1.0f);
    EXPECT_EQ(indiGetFeedbackMode(), INDI_FEEDBACK_DSHOT_RPM);
    EXPECT_FALSE(indiIsDshotWarningActive());
}

TEST_F(LadrcTest, testTwentyInchHeavyLiftQuad90msLagZeroWobble) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->prop_size_deci_in = 200; // 20.0" -> f_indi = 22.5 Hz, tau_act ~ 90.9 ms
    prof->auw_centi_kg = 1800;     // 18.00 kg -> realistic heavy-lift quad
    prof->agility_pct = 100;
    prof->authority_trim_pct = 100;
    prof->manual_override = 0;
    prof->disturbance_limit = 800;

    const float dt = 0.000625f; // 1.6 kHz
    ladrcSetFixedWing(false);
    ladrcInit(0, dt);
    ladrcSetDshotFeedbackStatus(false, 0); // Fallback actuator model!

    const float tauM = 0.090f; // 90 ms heavy-prop lag
    const float b0True = 22.0f; // realistic 1g-capable 20" 18 kg plant with default b0Nom
    const float dExt = 0.6f; // steady payload CG offset torque
    float yKdeg = 0.0f;
    float uMotor = 0.0f;
    float peakDps = 0.0f;
    float tailMinDps = 1e9f;
    float tailMaxDps = -1e9f;
    const float setpointDps = 150.0f;

    for (int step = 0; step < 3200; step++) { // 2.0 s
        const float sum = ladrcStepAxis(FD_ROLL, setpointDps, yKdeg * 1000.0f, 500.0f, 1.0f,
                                        nullptr, nullptr, nullptr, nullptr);
        const float uCmd = sum / PID_MIXER_SCALING;
        ladrcSetAxisActuatorFeedback(FD_ROLL, uCmd, 0.0f, false);
        uMotor += (dt / (dt + tauM)) * (uCmd - uMotor);
        yKdeg += dt * (b0True * uMotor + dExt);
        ladrcPostMixerUpdate(1.0f, true);

        const float rateDps = yKdeg * 1000.0f;
        if (rateDps > peakDps) {
            peakDps = rateDps;
        }
        if (step >= 2400) { // final 0.5 s settled window
            if (rateDps < tailMinDps) {
                tailMinDps = rateDps;
            }
            if (rateDps > tailMaxDps) {
                tailMaxDps = rateDps;
            }
        }
    }

    EXPECT_NEAR(yKdeg * 1000.0f, setpointDps, 2.0f);
    // Zero wobble: overshoot < 15% and peak-to-peak ripple in final 0.5s < 1.0 deg/s
    EXPECT_LT(peakDps, setpointDps * 1.15f);
    EXPECT_LT(tailMaxDps - tailMinDps, 1.0f);
}

TEST_F(LadrcTest, testFallbackActuatorModelAndDshotWarningTriggers) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->prop_size_deci_in = 100; // 10.0"
    prof->manual_override = 0;

    const float dt = 0.000625f;
    // Case 1: Multirotor without DShot RPM -> fallback actuator model + warning active
    ladrcSetFixedWing(false);
    ladrcInit(0, dt);
    ladrcSetDshotFeedbackStatus(false, 0);
    EXPECT_EQ(indiGetFeedbackMode(), INDI_FEEDBACK_ACTUATOR_MODEL);
    EXPECT_EQ(indiGetDshotActiveMotorCount(), 0);
    EXPECT_TRUE(indiIsDshotWarningActive());
    EXPECT_NEAR(indiGetLpfHz(), 45.0f, 0.01f);
    EXPECT_NEAR(indiGetRpmMinHz(), 50.0f, 0.01f);

    // Case 2: Multirotor with DShot configured ON on USB without battery -> standby, NO false warning
    ladrcSetDshotFeedbackStatus(false, 0);
    ladrcSetDshotConfiguredStatus(true, false);
    EXPECT_EQ(indiGetFeedbackMode(), INDI_FEEDBACK_ACTUATOR_MODEL);
    EXPECT_TRUE(indiIsDshotStandbyNoBattery());
    EXPECT_FALSE(indiIsDshotWarningActive());

    // Case 2b: Multirotor with DShot configured ON and battery connected, still 0 eRPM -> warning active
    ladrcSetDshotConfiguredStatus(true, true);
    EXPECT_FALSE(indiIsDshotStandbyNoBattery());
    EXPECT_TRUE(indiIsDshotWarningActive());
    ladrcSetDshotConfiguredStatus(false, true);

    // Case 3: Multirotor with DShot RPM -> DSHOT_RPM mode + no warning
    ladrcSetDshotFeedbackStatus(true, 4);
    EXPECT_EQ(indiGetFeedbackMode(), INDI_FEEDBACK_DSHOT_RPM);
    EXPECT_EQ(indiGetDshotActiveMotorCount(), 4);
    EXPECT_FALSE(indiIsDshotWarningActive());

    // Case 4: Fixed-wing -> actuator model (tau_act = 0.030s) and NO DShot warning
    ladrcSetDshotFeedbackStatus(false, 0);
    ladrcSetFixedWing(true);
    ladrcInit(0, dt);
    EXPECT_EQ(indiGetFeedbackMode(), INDI_FEEDBACK_ACTUATOR_MODEL);
    EXPECT_FALSE(indiIsDshotWarningActive());
    EXPECT_NEAR(ladrcGetAxisState(FD_ROLL)->tauActS, 0.030f, 1e-4f);
    ladrcSetFixedWing(false);
}

TEST_F(LadrcTest, testRollPitchYawSaturationPriority) {
    // QuadX mixer: { throttle, roll, pitch, yaw }
    const float quadX[4][4] = {
        { 1.0f, -1.0f,  1.0f, -1.0f }, // REAR_R
        { 1.0f, -1.0f, -1.0f,  1.0f }, // FRONT_R
        { 1.0f,  1.0f,  1.0f,  1.0f }, // REAR_L
        { 1.0f,  1.0f, -1.0f, -1.0f }, // FRONT_L
    };

    float motorMix[4] = { 0 };
    float mixMin = 0.0f, mixMax = 0.0f, rawRange = 0.0f;

    // Request Roll=0.35, Pitch=0.25, Yaw=0.80 -> rawRange = 1.40 - (-0.90) = 2.30 > 1.0!
    // Roll=0.35 keeps 100%, Pitch scales to 0.15, Yaw fits 0.15 in the non-opposing QuadX nullspace (span = 1.0)!
    indiAllocatePriorityMix(quadX, 4, 0.35f, 0.25f, 0.80f, motorMix, &mixMin, &mixMax, &rawRange);
    EXPECT_NEAR(rawRange, 2.30f, 1e-4f);
    EXPECT_LE(mixMax - mixMin, 1.0f + 1e-5f);

    float axes[XYZ_AXIS_COUNT] = { 0 };
    indiProjectMotorOutputsToAxes(motorMix, quadX, 4, axes);
    EXPECT_NEAR(axes[FD_ROLL], 0.35f, 1e-4f);  // Roll preserved 100%!
    EXPECT_NEAR(axes[FD_PITCH], 0.15f, 1e-4f); // Pitch gets remaining 0.15 headroom!
    EXPECT_NEAR(axes[FD_YAW], 0.15f, 1e-4f);   // Yaw fits 0.15 without clipping Roll or Pitch!

    // Request Roll=0.20, Pitch=0.15, Yaw=0.50 -> Roll+Pitch keep 100%, Yaw scales to 0.30 (span = 0.65 - (-0.35) = 1.0)!
    indiAllocatePriorityMix(quadX, 4, 0.20f, 0.15f, 0.50f, motorMix, &mixMin, &mixMax, &rawRange);
    EXPECT_LE(mixMax - mixMin, 1.0f + 1e-5f);
    indiProjectMotorOutputsToAxes(motorMix, quadX, 4, axes);
    EXPECT_NEAR(axes[FD_ROLL], 0.20f, 1e-4f);
    EXPECT_NEAR(axes[FD_PITCH], 0.15f, 1e-4f);
    EXPECT_NEAR(axes[FD_YAW], 0.30f, 1e-4f);
}

TEST_F(LadrcTest, testBumplessLiveReconfigurePreservesTrimAndAvoidsDerivativeSpike) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->prop_size_deci_in = 50;
    prof->auw_centi_kg = 65;
    prof->agility_pct = 100;
    prof->authority_trim_pct = 100;
    prof->manual_override = 0;
    prof->disturbance_limit = 800;

    const float dt = 0.00025f; // 4 kHz
    ladrcSetFixedWing(false);
    ladrcInit(0, dt);
    ladrcSetDshotFeedbackStatus(false, 0);

    // Settle steady flight at 200 deg/s against a steady disturbance
    const float gyroRateDps = 200.0f;
    const float spDps = 200.0f;
    float pOut = 0.0f, dOut = 0.0f, fOut = 0.0f, distOut = 0.0f;
    for (int i = 0; i < 600; i++) {
        const float sum = ladrcStepAxis(FD_ROLL, spDps, gyroRateDps, 500.0f, 1.0f,
                                        &pOut, &dOut, &fOut, &distOut);
        ladrcSetAxisActuatorFeedback(FD_ROLL, sum / PID_MIXER_SCALING, 0.0f, false);
        ladrcPostMixerUpdate(1.0f, true);
    }
    const ladrcRuntimeAxis_t *st = ladrcGetAxisState(FD_ROLL);
    ASSERT_NE(st, nullptr);
    const float preDistTrim = st->distTermSum;
    const float preCmd = st->uCmdNorm;

    // Live reconfigure while flying: must preserve filter states & trim without derivative spike
    prof->prop_size_deci_in = 70;
    prof->agility_pct = 120;
    ladrcReconfigureLive(0, dt);

    const float postSum = ladrcStepAxis(FD_ROLL, spDps, gyroRateDps, 500.0f, 1.0f,
                                        &pOut, &dOut, &fOut, &distOut);
    EXPECT_NEAR(distOut, preDistTrim, 5.0f);
    EXPECT_NEAR(dOut, 0.0f, 1.0f);
    EXPECT_NEAR(fOut, 0.0f, 1.0f);
    EXPECT_NEAR(postSum / PID_MIXER_SCALING, preCmd, 0.05f);
}

TEST_F(LadrcTest, testResetStatePreservesDisarmedGroundedAirborneFalse) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
    prof->prop_size_deci_in = 50;
    prof->manual_override = 0;

    const float dt = 0.0005f;
    ladrcInit(0, dt);
    ladrcSetAirborne(false);
    EXPECT_FALSE(ladrcIsAirborne());

    // Calling ladrcResetState while disarmed must keep airborneState == false
    ladrcResetState();
    EXPECT_FALSE(ladrcIsAirborne());

    // High gyro without sufficient throttle (< 0.08) must NOT trigger liftoff
    for (int i = 0; i < 200; i++) {
        ladrcUpdateLiftoffState(0.05f, 80.0f, dt);
    }
    EXPECT_FALSE(ladrcIsAirborne());

    // Throttle >= 0.08 + gyroPeak >= 50 dps must require 50ms sustained before liftoff
    for (int i = 0; i < 50; i++) { // 25 ms
        ladrcUpdateLiftoffState(0.10f, 60.0f, dt);
    }
    EXPECT_FALSE(ladrcIsAirborne());
    for (int i = 0; i < 60; i++) { // +30 ms = 55 ms
        ladrcUpdateLiftoffState(0.10f, 60.0f, dt);
    }
    EXPECT_TRUE(ladrcIsAirborne());

    // Restore airborne default for subsequent standalone tests
    ladrcSetAirborne(true);
}

TEST_F(LadrcTest, testNonOrthogonalVtailProjectionAndClimbThrottleSafeYawAllocation) {
    const float vtail[4][4] = {
        { 1.0f, -0.58f,  0.58f,  1.0f }, // REAR_R
        { 1.0f, -0.46f, -0.39f, -0.5f }, // FRONT_R
        { 1.0f,  0.58f,  0.58f, -1.0f }, // REAR_L
        { 1.0f,  0.46f, -0.39f,  0.5f }, // FRONT_L
    };

    float motorOutputs[4];
    const float trueThr = 0.55f;
    const float trueRoll = 0.12f;
    const float truePitch = -0.18f;
    const float trueYaw = 0.25f;
    for (int i = 0; i < 4; i++) {
        motorOutputs[i] = trueThr * vtail[i][0] + trueRoll * vtail[i][1] + truePitch * vtail[i][2] + trueYaw * vtail[i][3];
    }

    float axes[XYZ_AXIS_COUNT] = { 0 };
    indiProjectMotorOutputsToAxes(motorOutputs, vtail, 4, axes);
    EXPECT_NEAR(axes[FD_ROLL], trueRoll, 1e-3f);
    EXPECT_NEAR(axes[FD_PITCH], truePitch, 1e-3f);
    EXPECT_NEAR(axes[FD_YAW], trueYaw, 1e-3f);

    // Climb-throttle-safe yaw allocation on QuadX: at 85% climb throttle, yaw must NOT drop collective throttle!
    const float quadX[4][4] = {
        { 1.0f, -1.0f,  1.0f, -1.0f },
        { 1.0f, -1.0f, -1.0f,  1.0f },
        { 1.0f,  1.0f,  1.0f,  1.0f },
        { 1.0f,  1.0f, -1.0f, -1.0f },
    };
    float motorMix[4] = { 0 };
    float thrOut = 0.0f, mixMin = 0.0f, mixMax = 0.0f, rawRange = 0.0f;
    indiAllocatePriorityMixWithThrottle(quadX, 4, 0.85f, 1, true,
                                        0.05f, 0.05f, 0.60f,
                                        motorMix, &thrOut, &mixMin, &mixMax, &rawRange);
    EXPECT_GE(thrOut, 0.85f - 1e-5f);
    for (int i = 0; i < 4; i++) {
        const float m = thrOut + motorMix[i];
        EXPECT_GE(m, -1e-5f);
        EXPECT_LE(m, 1.0f + 1e-5f);
    }
    indiProjectMotorOutputsToAxes(motorMix, quadX, 4, axes);
    EXPECT_NEAR(axes[FD_ROLL], 0.05f, 1e-4f);
    EXPECT_NEAR(axes[FD_PITCH], 0.05f, 1e-4f);
    EXPECT_NEAR(axes[FD_YAW], 0.05f, 1e-4f); // Roll+Pitch take 0.10 headroom -> remaining 0.05 for Yaw without dropping climb throttle

    // Low-throttle airmode yaw collective shift capped at +0.10
    indiAllocatePriorityMixWithThrottle(quadX, 4, 0.05f, 1, true,
                                        0.0f, 0.0f, 0.60f,
                                        motorMix, &thrOut, &mixMin, &mixMax, &rawRange);
    EXPECT_LE(thrOut, 0.15f + 1e-5f);
    for (int i = 0; i < 4; i++) {
        const float m = thrOut + motorMix[i];
        EXPECT_GE(m, -1e-5f);
        EXPECT_LE(m, 1.0f + 1e-5f);
    }
}

TEST_F(LadrcTest, testNaNHardeningInConstrainfAndLadrcStep) {
    float nanVal = 0.0f;
    float posInf = 0.0f;
    float negInf = 0.0f;
    const uint32_t nanBits = 0x7FC00000U;
    const uint32_t posInfBits = 0x7F800000U;
    const uint32_t negInfBits = 0xFF800000U;
    memcpy(&nanVal, &nanBits, sizeof(nanVal));
    memcpy(&posInf, &posInfBits, sizeof(posInf));
    memcpy(&negInf, &negInfBits, sizeof(negInf));

    EXPECT_TRUE(floatIsNaNBits(nanVal));
    EXPECT_FALSE(floatIsFiniteBits(nanVal));
    EXPECT_FALSE(floatIsFiniteBits(posInf));
    EXPECT_FALSE(floatIsFiniteBits(negInf));
    EXPECT_TRUE(floatIsFiniteBits(1.25f));

    EXPECT_FLOAT_EQ(constrainf(nanVal, -1.0f, 1.0f), 0.0f);
    EXPECT_FLOAT_EQ(constrainf(nanVal, 0.5f, 1.5f), 0.5f);
    EXPECT_FLOAT_EQ(constrainf(posInf, -1.0f, 1.0f), 1.0f);
    EXPECT_FLOAT_EQ(constrainf(negInf, -1.0f, 1.0f), -1.0f);

    ladrcInit(0, 0.0005f);
    float pOut = 0.0f, dOut = 0.0f, fOut = 0.0f, distOut = 0.0f;
    const float out = ladrcStepAxisEx(FD_ROLL, nanVal, nanVal, 500.0f, nanVal, 1.0f, LADRC_RESET_NONE,
                                      &pOut, &dOut, &fOut, &distOut);
    EXPECT_TRUE(floatIsFiniteBits(out));
    EXPECT_TRUE(floatIsFiniteBits(pOut));
    EXPECT_TRUE(floatIsFiniteBits(dOut));
    EXPECT_TRUE(floatIsFiniteBits(fOut));
    EXPECT_TRUE(floatIsFiniteBits(distOut));
    ladrcPostMixerUpdate(1.0f, true);

    const float recOut = ladrcStepAxisEx(FD_ROLL, 100.0f, 0.0f, 500.0f, 1.0f, 1.0f, LADRC_RESET_NONE,
                                         &pOut, &dOut, &fOut, &distOut);
    EXPECT_TRUE(floatIsFiniteBits(recOut));
    EXPECT_GT(recOut, 0.0f);
}
