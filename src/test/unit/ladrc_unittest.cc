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
        const float sum = ladrcStepAxisEx(FD_ROLL, 200.0f, 0.0f, 500.0f, 1.0f, 1.0f, LADRC_RESET_OBSERVER_FULL,
                                          &pOut, &dOut, &fOut, &distOut);
        EXPECT_FLOAT_EQ(dOut, 0.0f);
        EXPECT_FLOAT_EQ(distOut, 0.0f);
        EXPECT_FLOAT_EQ(sum, pOut);
        ladrcPostMixerUpdate(0.5f, true);
    }

    const ladrcRuntimeAxis_t *st = ladrcGetAxisState(FD_ROLL);
    ASSERT_NE(st, nullptr);
    EXPECT_FLOAT_EQ(st->z2, 0.0f);
    EXPECT_FLOAT_EQ(st->z3, 0.0f);
    EXPECT_FLOAT_EQ(st->distTermSum, 0.0f);

    // Phase 2: Build up disturbance without reset, then trigger ground full observer reset -> Sum clears immediately
    for (int step = 0; step < 200; step++) {
        ladrcStepAxisEx(FD_ROLL, 200.0f, 0.0f, 500.0f, 1.0f, 1.0f, LADRC_RESET_NONE, nullptr, nullptr, nullptr, nullptr);
        ladrcPostMixerUpdate(1.0f, true);
    }
    EXPECT_GT(fabsf(st->z3), 0.1f);

    float pOut = 0.0f, dOut = 0.0f, fOut = 0.0f, distOut = 0.0f;
    const float resetSum = ladrcStepAxisEx(FD_ROLL, 200.0f, 0.0f, 500.0f, 1.0f, 1.0f, LADRC_RESET_OBSERVER_FULL,
                                           &pOut, &dOut, &fOut, &distOut);
    EXPECT_FLOAT_EQ(dOut, 0.0f);
    EXPECT_FLOAT_EQ(distOut, 0.0f);
    EXPECT_FLOAT_EQ(resetSum, pOut);
    EXPECT_FLOAT_EQ(st->z2, 0.0f);
    EXPECT_FLOAT_EQ(st->z3, 0.0f);
}

TEST_F(LadrcTest, testOrder2DisturbanceOnlyResetPreservesDampingZ2) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
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

    // Build both z2 (accel) and z3 (disturbance) on Order-2 roll, and z2 (disturbance) on Order-1 yaw
    for (int step = 0; step < 100; step++) {
        const float gyroRoll = 2.0f * (float)step;
        ladrcStepAxisEx(FD_ROLL, 200.0f, gyroRoll, 500.0f, 1.0f, 1.0f, LADRC_RESET_NONE,
                        nullptr, nullptr, nullptr, nullptr);
        ladrcStepAxisEx(FD_YAW, 150.0f, 0.0f, 400.0f, 1.0f, 1.0f, LADRC_RESET_NONE,
                        nullptr, nullptr, nullptr, nullptr);
        ladrcPostMixerUpdate(1.0f, true);
    }

    const ladrcRuntimeAxis_t *rollSt = ladrcGetAxisState(FD_ROLL);
    const ladrcRuntimeAxis_t *yawSt = ladrcGetAxisState(FD_YAW);
    ASSERT_NE(rollSt, nullptr);
    ASSERT_NE(yawSt, nullptr);
    EXPECT_GT(fabsf(rollSt->z3), 0.1f);
    EXPECT_GT(fabsf(yawSt->z2), 0.1f);

    // In-flight disturbance-only reset during positive angular kick on Roll:
    // Order-2 clears z3 (distOut == 0) while preserving active z2 D-term damping!
    // Order-1 clears z2 (distOut == 0).
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
    EXPECT_FLOAT_EQ(yawSt->z2, 0.0f);

    // Post-mixer prediction must keep z1/z2 active and predicted in disturbance-only mode
    const float z1BeforePredict = rollSt->z1;
    const float z2BeforePredict = rollSt->z2;
    ladrcPostMixerUpdate(0.5f, true);
    EXPECT_FLOAT_EQ(rollSt->z3, 0.0f);
    EXPECT_NE(rollSt->z1, z1BeforePredict);
    EXPECT_NE(rollSt->z2, 0.0f);
    EXPECT_NE(rollSt->z2, z2BeforePredict);

    // Explicit ladrcResetAxisDisturbance also clears only z3 on Order-2 without wiping z2
    ladrcStepAxisEx(FD_ROLL, 0.0f, 520.0f, 500.0f, 1.0f, 0.0f, LADRC_RESET_NONE,
                    nullptr, nullptr, nullptr, nullptr);
    EXPECT_NE(rollSt->z3, 0.0f);
    ladrcResetAxisDisturbance(FD_ROLL);
    EXPECT_FLOAT_EQ(rollSt->z3, 0.0f);
    EXPECT_FLOAT_EQ(rollSt->distTermSum, 0.0f);
    EXPECT_NE(rollSt->z2, 0.0f);
}

TEST_F(LadrcTest, testTrackingDifferentiatorBypassAndWeighting) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
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
    EXPECT_GT(pZero, 100.0f);

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
    prof->sat_z3_inhibit = 1;
    prof->anti_windup = 1;
    prof->disturbance_limit = 800;
    prof->axis[FD_ROLL].order = LADRC_ORDER_2;
    prof->axis[FD_ROLL].wc_deci = 150;
    prof->axis[FD_ROLL].wo_deci = 600;
    prof->axis[FD_ROLL].b0_deci = 2000;
    prof->axis[FD_ROLL].td_hz = 0;

    const float dt = 0.0005f;
    ladrcInit(0, dt);

    // 1. With actuator saturated (mixerSaturatedFlag = true) and commanding 500 dps while craft is held at 0 dps:
    ladrcSetMixerSaturated(true);
    EXPECT_TRUE(ladrcIsMixerSaturated());

    for (int step = 0; step < 100; step++) {
        ladrcStepAxis(FD_ROLL, 500.0f, 0.0f, 500.0f, 1.0f, nullptr, nullptr, nullptr, nullptr);
        ladrcPostMixerUpdate(1.0f, true);
    }
    const ladrcRuntimeAxis_t *st = ladrcGetAxisState(FD_ROLL);
    ASSERT_NE(st, nullptr);
    // With sat_z3_inhibit active and starting from 0, z3 must remain zero (growth completely inhibited)
    EXPECT_FLOAT_EQ(st->z3, 0.0f);

    // 2. Compare against uninhibited behavior (sat_z3_inhibit = 0):
    ladrcInit(0, dt);
    prof->sat_z3_inhibit = 0;
    ladrcSetMixerSaturated(true);

    for (int step = 0; step < 100; step++) {
        ladrcStepAxis(FD_ROLL, 500.0f, 0.0f, 500.0f, 1.0f, nullptr, nullptr, nullptr, nullptr);
        ladrcPostMixerUpdate(1.0f, true);
    }
    // Without inhibit, z3 winds up strongly negative (-30 or worse)
    EXPECT_LT(st->z3, -30.0f);

    // 3. Re-enable sat_z3_inhibit and verify that wound-up z3 decays towards 0 during saturation:
    prof->sat_z3_inhibit = 1;
    ladrcSetMixerSaturated(true);
    // Command 0 dps while gyro is at 200 dps (produces positive innovation towards 0)
    for (int step = 0; step < 100; step++) {
        ladrcStepAxis(FD_ROLL, 0.0f, 200.0f, 500.0f, 1.0f, nullptr, nullptr, nullptr, nullptr);
        ladrcPostMixerUpdate(1.0f, true);
    }
    // Wound-up negative disturbance must have unwound towards 0!
    EXPECT_GT(st->z3, -10.0f);
}

TEST_F(LadrcTest, testLiftoffStateAndGroundWcAttenuation) {
    ladrcProfile_t *prof = ladrcProfilesMutable(0);
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
    // uP = (wcEff^2 * error) / b0, where wcEff = 20 * 0.50 = 10 rad/s -> wcEff^2 = 100
    // Compared to airborne (wc = 20 -> wc^2 = 400), uP should be 100/400 = 25% (wcEff^2)
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

    EXPECT_NEAR(pGrounded / pAirborne, 0.25f, 0.02f);

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
