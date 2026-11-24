/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <cmath>

#include "unittest_macros.h"
#include "gtest/gtest.h"
#include "build/debug.h"

bool simulatedThrottleRaised = true;
float simulatedSetpointRate[3] = { 0,0,0 };
float simulatedPrevSetpointRate[3] = { 0,0,0 };
float simulatedRcDeflection[3] = { 0,0,0 };
float simulatedMaxRcDeflectionAbs = 0;
float simulatedMixerGetRcThrottle = 0;
float simulatedRcCommandDelta[3] = { 1,1,1 };
float simulatedRawSetpoint[3] = { 0,0,0 };
float simulatedMaxRate[3] = { 670,670,670 };
float simulatedFeedforward[3] = { 0,0,0 };
float simulatedMotorMixRange = 0.0f;

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/filter.h"

    #include "config/config.h"
    #include "config/config_reset.h"

    #include "drivers/sound_beeper.h"
    #include "drivers/time.h"

    #include "fc/controlrate_profile.h"
    #include "fc/core.h"
    #include "fc/rc.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/imu.h"
    #include "flight/ladrc.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/pid_init.h"
    #include "flight/position.h"

    #include "io/gps.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "pg/rx.h"
    #include "rx/rx.h"

    #include "sensors/gyro.h"
    #include "sensors/acceleration.h"

    acc_t acc;
    gyro_t gyro;
    attitudeEulerAngles_t attitude;

    rxRuntimeState_t rxRuntimeState = {};

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);
    PG_REGISTER(positionConfig_t, positionConfig, PG_SYSTEM_CONFIG, 4);

    bool unitLaunchControlActive = false;
    launchControlMode_e unitLaunchControlMode = LAUNCH_CONTROL_MODE_NORMAL;

    float getMotorMixRange(void) { return simulatedMotorMixRange; }
    float getSetpointRate(int axis) { return simulatedSetpointRate[axis]; }
    bool wasThrottleRaised(void) { return simulatedThrottleRaised; }
    float getRcDeflectionAbs(int axis) { return fabsf(simulatedRcDeflection[axis]); }

    // used by auto-disarm code
    float getMaxRcDeflectionAbs() { return fabsf(simulatedMaxRcDeflectionAbs); }
    float mixerGetRcThrottle() { return fabsf(simulatedMixerGetRcThrottle); }


    bool isBelowLandingAltitude(void) { return false; }

    void systemBeep(bool) { }
    bool gyroOverflowDetected(void) { return false; }
    float getRcDeflection(int axis) { return simulatedRcDeflection[axis]; }
    float getRcDeflectionRaw(int axis) { return simulatedRcDeflection[axis]; }
    float getRawSetpoint(int axis) { return simulatedRawSetpoint[axis]; }
    float getFeedforward(int axis) {
        return simulatedSetpointRate[axis] - simulatedPrevSetpointRate[axis];
    }
    void beeperConfirmationBeeps(uint8_t) { }
    bool isLaunchControlActive(void) {return unitLaunchControlActive; }
    void disarm(flightLogDisarmReason_e) { }
    float getMaxRcRate(int axis)
    {
        UNUSED(axis);
        float maxRate = simulatedMaxRate[axis];
        return maxRate;
    }
    void initRcProcessing(void) { }
}

pidProfile_t *pidProfile;

int loopIter = 0;

// Always use same defaults for testing in future releases even when defaults change
void setDefaultTestSettings(void)
{
    pgResetAll();
    pidProfile = pidProfilesMutable(1);
    pidProfile->pid[PID_ROLL]  =  { 40, 40, 30, 65, 0 };
    pidProfile->pid[PID_PITCH] =  { 58, 50, 35, 60, 0 };
    pidProfile->pid[PID_YAW]   =  { 70, 45, 20, 60, 0 };
    pidProfile->pid[PID_LEVEL] =  { 50, 50, 75, 50, 0 };

    // Compensate for the upscaling done without 'use_integrated_yaw'
    pidProfile->pid[PID_YAW].I = pidProfile->pid[PID_YAW].I / 2.5f;

    pidProfile->pidSumLimit = PIDSUM_LIMIT;        // 500
    pidProfile->pidSumLimitYaw = PIDSUM_LIMIT_YAW; // 400
    pidProfile->yaw_lowpass_hz = 0;
    pidProfile->dterm_lpf1_static_hz = 100;
    pidProfile->dterm_lpf2_static_hz = 0;
    pidProfile->dterm_notch_hz = 260;
    pidProfile->dterm_notch_cutoff = 160;
    pidProfile->dterm_lpf1_type = FILTER_SVF;
    pidProfile->itermWindup = 80;
    pidProfile->pidAtMinThrottle = PID_STABILISATION_ON;
    pidProfile->angle_limit = 60;
    pidProfile->feedforward_transition = 100;
    pidProfile->yawRateAccelLimit = 100;
    pidProfile->rateAccelLimit = 0;
    pidProfile->anti_gravity_gain = 10;
    pidProfile->crash_time = 500;
    pidProfile->crash_delay = 0;
    pidProfile->crash_recovery_angle = 10;
    pidProfile->crash_recovery_rate = 100;
    pidProfile->crash_dthreshold = 50;
    pidProfile->crash_gthreshold = 400;
    pidProfile->crash_setpoint_threshold = 350;
    pidProfile->crash_recovery = PID_CRASH_RECOVERY_OFF;
    pidProfile->horizon_limit_degrees = 135;
    pidProfile->horizon_ignore_sticks = false;
    pidProfile->crash_limit_yaw = 200;
    pidProfile->itermLimit = 150;
    pidProfile->throttle_boost = 0;
    pidProfile->throttle_boost_cutoff = 15;
    pidProfile->iterm_rotation = false;
    pidProfile->iterm_relax = ITERM_RELAX_OFF,
    pidProfile->iterm_relax_cutoff = 11,
    pidProfile->iterm_relax_type = ITERM_RELAX_SETPOINT,
    pidProfile->launchControlMode = LAUNCH_CONTROL_MODE_NORMAL,
    pidProfile->launchControlGain = 40,
    pidProfile->level_race_mode = false,

    gyro.targetLooptime = 8000;
}

timeUs_t currentTestTime(void)
{
    return targetPidLooptime * loopIter++;
}

void resetTest(void)
{
    loopIter = 0;
    pidRuntime.tpaFactor = 1.0f;
    simulatedMotorMixRange = 0.0f;
    simulatedThrottleRaised = true;

    pidStabilisationState(PID_STABILISATION_OFF);
    pidSetItermReset(false);
    DISABLE_ARMING_FLAG(ARMED);

    setDefaultTestSettings();
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidData[axis].P = 0;
        pidData[axis].I = 0;
        pidData[axis].D = 0;
        pidData[axis].F = 0;
        pidData[axis].S = 0;
        pidData[axis].Sum = 0;
        simulatedSetpointRate[axis] = 0;
        simulatedRcDeflection[axis] = 0;
        simulatedRawSetpoint[axis] = 0;
        gyro.gyroADCf[axis] = 0;
    }
    attitude.values.roll = 0;
    attitude.values.pitch = 0;
    attitude.values.yaw = 0;

    flightModeFlags = 0;
    unitLaunchControlActive = false;
    pidProfile->launchControlMode = unitLaunchControlMode;
    pidInit(pidProfile);
    loadControlRateProfile();

    // Run pidloop for a while after reset
    for (int loop = 0; loop < 20; loop++) {
        pidController(pidProfile, currentTestTime());
    }
}

void setStickPosition(int axis, float stickRatio)
{
    simulatedPrevSetpointRate[axis] = simulatedSetpointRate[axis];
    simulatedSetpointRate[axis] = 1998.0f * stickRatio;
    simulatedRcDeflection[axis] = stickRatio;
}

// All calculations will have 10% tolerance
float calculateTolerance(float input)
{
    return fabsf(input * 0.1f);
}

TEST(pidControllerTest, testInitialisation)
{
    resetTest();

    // In initial state PIDsums should be 0
    for (int axis = 0; axis <= FD_YAW; axis++) {
        EXPECT_FLOAT_EQ(0, pidData[axis].P);
        EXPECT_FLOAT_EQ(0, pidData[axis].I);
        EXPECT_FLOAT_EQ(0, pidData[axis].D);
    }
}

TEST(pidControllerTest, testStabilisationDisabled)
{
    ENABLE_ARMING_FLAG(ARMED);
    // Run few loops to make sure there is no error building up when stabilisation disabled

    for (int loop = 0; loop < 10; loop++) {
        pidController(pidProfile, currentTestTime());

        // PID controller should not do anything, while stabilisation disabled
        EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
        EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
        EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
        EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
        EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
        EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
        EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
        EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
        EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);
    }
}

TEST(pidControllerTest, testAdrcLoop)
{
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidController(pidProfile, currentTestTime());
    ladrcPostMixerUpdate(1.0f, true);

    // Loop 1 - Expecting zero since setpoint and gyro are zero
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].Sum);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].Sum);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].Sum);

    // Add positive gyro rate disturbance on ROLL -> controller must push negative torque
    gyro.gyroADCf[FD_ROLL] = 100;
    pidController(pidProfile, currentTestTime());
    ladrcPostMixerUpdate(1.0f, true);
    EXPECT_LT(pidData[FD_ROLL].Sum, 0.0f);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].Sum);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].Sum);

    // Add negative gyro rate disturbance on PITCH -> controller must push positive torque
    gyro.gyroADCf[FD_PITCH] = -100;
    pidController(pidProfile, currentTestTime());
    ladrcPostMixerUpdate(1.0f, true);
    EXPECT_GT(pidData[FD_PITCH].Sum, 0.0f);

    // Add positive gyro rate on YAW -> negative yaw torque
    gyro.gyroADCf[FD_YAW] = 50;
    pidController(pidProfile, currentTestTime());
    ladrcPostMixerUpdate(1.0f, true);
    EXPECT_LT(pidData[FD_YAW].Sum, 0.0f);

    // Disable stabilisation -> outputs and observer reset to zero
    pidStabilisationState(PID_STABILISATION_OFF);
    pidController(pidProfile, currentTestTime());
    ladrcPostMixerUpdate(0.0f, false);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].Sum);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].Sum);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].Sum);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
}

TEST(pidControllerTest, testPidLevel)
{
    // Make sure to start with fresh values
    resetTest();
    ladrcProfilesMutable(systemConfig()->pidProfileIndex)->manual_override = 1;
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    // Test Angle mode response
    enableFlightMode(ANGLE_MODE);
    float currentPidSetpointRoll = 0;
    float currentPidSetpointPitch = 0;
    float calculatedAngleSetpoint = 0;
    rollAndPitchTrims_t angleTrim = { { 0, 0 } };

    calculatedAngleSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpointRoll, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(0, calculatedAngleSetpoint);
    calculatedAngleSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpointPitch, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(0, calculatedAngleSetpoint);

    currentPidSetpointRoll = 200;
    calculatedAngleSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpointRoll, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(51.456356, calculatedAngleSetpoint);
    currentPidSetpointPitch = -200;
    calculatedAngleSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpointPitch, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(-51.456356, calculatedAngleSetpoint);

    currentPidSetpointRoll = 400;
    calculatedAngleSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpointRoll, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(128.94597, calculatedAngleSetpoint);
    currentPidSetpointPitch = -400;
    calculatedAngleSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpointPitch, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(-128.94597, calculatedAngleSetpoint);

    // Test attitude response
    attitude.values.roll = -275;
    attitude.values.pitch = 275;
    calculatedAngleSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpointRoll, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(242.76686, calculatedAngleSetpoint);
    calculatedAngleSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpointPitch, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(-242.76686, calculatedAngleSetpoint);

    // Disable ANGLE_MODE
    disableFlightMode(ANGLE_MODE);
    calculatedAngleSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpointRoll, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(393.44571, calculatedAngleSetpoint);
    calculatedAngleSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpointPitch, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(-392.88422, calculatedAngleSetpoint);

    // Test level mode expo
    enableFlightMode(ANGLE_MODE);
    attitude.values.roll = 0;
    attitude.values.pitch = 0;
    currentPidSetpointRoll = 400;
    currentPidSetpointPitch = -400;
    // need to set some rates type and some expo here ??? HELP !!
    calculatedAngleSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpointRoll, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(231.55479, calculatedAngleSetpoint);
    calculatedAngleSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpointPitch, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(-231.55479, calculatedAngleSetpoint);
}


TEST(pidControllerTest, testPidHorizon)
{
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    enableFlightMode(HORIZON_MODE);

    // Test stick response greater than default limit of 0.75
    setStickPosition(FD_ROLL, 0.76f);
    setStickPosition(FD_PITCH, -0.76f);
    EXPECT_FLOAT_EQ(0.0f, calcHorizonLevelStrength());

    // Return sticks to center, should expect some levelling, but will be delayed
    setStickPosition(FD_ROLL, 0);
    setStickPosition(FD_PITCH, 0);
    EXPECT_FLOAT_EQ(0.0078740157, calcHorizonLevelStrength());

    // Test small stick response when flat, considering delay
    setStickPosition(FD_ROLL, 0.1f);
    setStickPosition(FD_PITCH, -0.1f);
    EXPECT_NEAR(0.01457f, calcHorizonLevelStrength(), calculateTolerance(0.01457));

    // Test larger stick response when flat
    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    EXPECT_NEAR(0.0166, calcHorizonLevelStrength(), calculateTolerance(0.0166));

    // set attitude of craft to 90 degrees
    attitude.values.roll = 900;
    attitude.values.pitch = 900;

    // Test centered sticks at 90 degrees
    setStickPosition(FD_ROLL, 0);
    setStickPosition(FD_PITCH, 0);
    // with gain of 50, and max angle of 135 deg, strength = 0.5 * (135-90) / 90 ie 0.5 * 45/136 or 0.5 * 0.333 = 0.166
    EXPECT_NEAR(0.0193f, calcHorizonLevelStrength(), calculateTolerance(0.0193));

    // Test small stick response at 90 degrees
    setStickPosition(FD_ROLL, 0.1f);
    setStickPosition(FD_PITCH, -0.1f);
    EXPECT_NEAR(0.0213f, calcHorizonLevelStrength(), calculateTolerance(0.0213));

    // Test larger stick response at 90 degrees
    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    EXPECT_NEAR(0.0218f, calcHorizonLevelStrength(), calculateTolerance(0.0218));

    // set attitude of craft to 120 degrees, inside limit of 135
    attitude.values.roll = 1200;
    attitude.values.pitch = 1200;

    // Test centered sticks at 120 degrees
    setStickPosition(FD_ROLL, 0);
    setStickPosition(FD_PITCH, 0);
    EXPECT_NEAR(0.0224f, calcHorizonLevelStrength(), calculateTolerance(0.0224));

    // Test small stick response at 120 degrees
    setStickPosition(FD_ROLL, 0.1f);
    setStickPosition(FD_PITCH, -0.1f);
    EXPECT_NEAR(0.0228f, calcHorizonLevelStrength(), calculateTolerance(0.0228));

    // Test larger stick response at 120 degrees
    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    EXPECT_NEAR(0.018f, calcHorizonLevelStrength(), calculateTolerance(0.018));

    // set attitude of craft to 1500 degrees, outside limit of 135
    attitude.values.roll = 1500;
    attitude.values.pitch = 1500;

    // Test centered sticks at 150 degrees - should be zero at any stick angle
    setStickPosition(FD_ROLL, 0);
    setStickPosition(FD_PITCH, 0);
    EXPECT_NEAR(0.0f, calcHorizonLevelStrength(), calculateTolerance(0.0));

    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    EXPECT_NEAR(0.0f, calcHorizonLevelStrength(), calculateTolerance(0.0));

}

// trying to fix



TEST(pidControllerTest, testAdrcMixerSaturationAntiWindup)
{
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    setStickPosition(FD_ROLL, 1.0f);
    setStickPosition(FD_PITCH, -1.0f);
    setStickPosition(FD_YAW, 1.0f);

    const float distCap = (float)ladrcProfiles(systemConfig()->pidProfileIndex)->disturbance_limit;
    for (int loop = 0; loop < 100; loop++) {
        pidController(pidProfile, currentTestTime());
        ladrcPostMixerUpdate(0.25f, true);
    }

    EXPECT_LE(fabsf(pidData[FD_ROLL].I), distCap + 1.0f);
    EXPECT_LE(fabsf(pidData[FD_PITCH].I), distCap + 1.0f);
    EXPECT_LE(fabsf(pidData[FD_YAW].I), distCap + 1.0f);
    EXPECT_LE(fabsf(pidData[FD_ROLL].Sum), pidProfile->pidSumLimit + 0.01f);
    EXPECT_LE(fabsf(pidData[FD_YAW].Sum), pidProfile->pidSumLimitYaw + 0.01f);
}

// TODO - Add more scenarios
TEST(pidControllerTest, testCrashRecoveryMode)
{
    resetTest();
    pidProfile->crash_recovery = PID_CRASH_RECOVERY_ON;
    pidInit(pidProfile);
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    sensorsSet(SENSOR_ACC);

    EXPECT_FALSE(crashRecoveryModeActive());

    int loopsToCrashTime = (int)((pidProfile->crash_time * 1000) / targetPidLooptime) + 1;

    // generate crash detection for roll axis
    gyro.gyroADCf[FD_ROLL]  = 800;
    simulatedMotorMixRange = 1.2f;
    for (int loop =0; loop <= loopsToCrashTime; loop++) {
        gyro.gyroADCf[FD_ROLL] += gyro.gyroADCf[FD_ROLL];
        // advance the time to avoid initialized state prevention of crash recovery
        pidController(pidProfile, currentTestTime() + 2000000);
    }

    EXPECT_TRUE(crashRecoveryModeActive());
    // Add additional verifications
}

TEST(pidControllerTest, testTpaClassic)
{
    resetTest();

    pidProfile->tpa_curve_type = TPA_CURVE_CLASSIC;
    pidProfile->tpa_rate = 30;
    pidProfile->tpa_breakpoint = 1600;
    pidProfile->tpa_low_rate = -50;
    pidProfile->tpa_low_breakpoint = 1200;
    pidProfile->tpa_low_always = 1;

    pidInit(pidProfile);

    pidUpdateTpaFactor(0.0f);
    EXPECT_FLOAT_EQ(1.5f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.1f);
    EXPECT_FLOAT_EQ(1.25f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.2f);
    EXPECT_FLOAT_EQ(1.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.6f);
    EXPECT_FLOAT_EQ(1.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.8f);
    EXPECT_FLOAT_EQ(0.85f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(1.0f);
    EXPECT_FLOAT_EQ(0.7f, pidRuntime.tpaFactor);


    pidProfile->tpa_curve_type = TPA_CURVE_CLASSIC;
    pidProfile->tpa_rate = 30;
    pidProfile->tpa_breakpoint = 1600;
    pidProfile->tpa_low_rate = -50;
    pidProfile->tpa_low_breakpoint = 1000;
    pidProfile->tpa_low_always = 1;

    pidInit(pidProfile);

    pidUpdateTpaFactor(0.0f);
    EXPECT_FLOAT_EQ(1.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.1f);
    EXPECT_FLOAT_EQ(1.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.2f);
    EXPECT_FLOAT_EQ(1.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.6f);
    EXPECT_FLOAT_EQ(1.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.8f);
    EXPECT_FLOAT_EQ(0.85f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(1.0f);
    EXPECT_FLOAT_EQ(0.7f, pidRuntime.tpaFactor);
}

TEST(pidControllerTest, testTpaHyperbolic)
{
    resetTest();

    // curve sligly down - edge case where internal expo -> inf
    pidProfile->tpa_curve_type = TPA_CURVE_HYPERBOLIC;
    pidProfile->tpa_curve_pid_thr100 = 50;
    pidProfile->tpa_curve_pid_thr0 = 500;
    pidProfile->tpa_curve_expo = 10;
    pidProfile->tpa_curve_stall_throttle = 30;

    pidInit(pidProfile);

    pidUpdateTpaFactor(0.0f);
    EXPECT_FLOAT_EQ(5.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.15f);
    EXPECT_FLOAT_EQ(5.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.5);
    EXPECT_NEAR(2.588f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.9);
    EXPECT_NEAR(0.693f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(1.0);
    EXPECT_NEAR(0.5f, pidRuntime.tpaFactor, 0.01f);

    // linear curve
    pidProfile->tpa_curve_type = TPA_CURVE_HYPERBOLIC;
    pidProfile->tpa_curve_pid_thr100 = 10;
    pidProfile->tpa_curve_pid_thr0 = 300;
    pidProfile->tpa_curve_expo = 0;
    pidProfile->tpa_curve_stall_throttle = 0;

    pidInit(pidProfile);

    pidUpdateTpaFactor(0.0f);
    EXPECT_FLOAT_EQ(3.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.15f);
    EXPECT_NEAR(2.565f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.5);
    EXPECT_NEAR(1.550f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.9);
    EXPECT_NEAR(0.390f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(1.0);
    EXPECT_NEAR(0.1f, pidRuntime.tpaFactor, 0.01f);

    // curve bends up
    pidProfile->tpa_curve_type = TPA_CURVE_HYPERBOLIC;
    pidProfile->tpa_curve_pid_thr100 = 60;
    pidProfile->tpa_curve_pid_thr0 = 1000;
    pidProfile->tpa_curve_expo = -50;
    pidProfile->tpa_curve_stall_throttle = 40;

    pidInit(pidProfile);

    pidUpdateTpaFactor(0.0f);
    EXPECT_FLOAT_EQ(10.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.15f);
    EXPECT_NEAR(10.0f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.5);
    EXPECT_NEAR(9.700f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.9);
    EXPECT_NEAR(7.364f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(1.0);
    EXPECT_NEAR(0.625f, pidRuntime.tpaFactor, 0.01f);

    // curve bends down
    pidProfile->tpa_curve_type = TPA_CURVE_HYPERBOLIC;
    pidProfile->tpa_curve_pid_thr100 = 90;
    pidProfile->tpa_curve_pid_thr0 = 250;
    pidProfile->tpa_curve_expo = 60;
    pidProfile->tpa_curve_stall_throttle = 60;

    pidInit(pidProfile);

    pidUpdateTpaFactor(0.0f);
    EXPECT_FLOAT_EQ(2.5f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.15f);
    EXPECT_NEAR(2.5f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.5);
    EXPECT_NEAR(2.5f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.9);
    EXPECT_NEAR(0.954f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(1.0);
    EXPECT_NEAR(0.9f, pidRuntime.tpaFactor, 0.01f);
}

TEST(pidControllerTest, testZeroThrottleResetClearsDisturbanceBeforeOutput)
{
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    // Case 1: Pre-takeoff ground idle (zeroThrottleItermReset && !wasThrottleRaised()) -> full observer reset (z2=0, z3=0, D=0, I=0)
    simulatedThrottleRaised = false;
    setStickPosition(FD_ROLL, 0.4f);
    for (int i = 0; i < 100; i++) {
        pidController(pidProfile, currentTestTime());
        ladrcPostMixerUpdate(1.0f, true);
    }
    EXPECT_GT(fabsf(pidData[FD_ROLL].I), 1.0f);

    pidSetItermReset(true);
    pidController(pidProfile, currentTestTime());
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(pidData[FD_ROLL].P + pidData[FD_ROLL].F, pidData[FD_ROLL].Sum);
    const ladrcRuntimeAxis_t *st = ladrcGetAxisState(FD_ROLL);
    ASSERT_NE(st, nullptr);
    EXPECT_FLOAT_EQ(0.0f, st->z2);
    EXPECT_FLOAT_EQ(0.0f, st->z3);

    // Case 2: Airborne zero-throttle chop (zeroThrottleItermReset && wasThrottleRaised()) -> clears disturbance/I while keeping D-term damping active!
    pidSetItermReset(false);
    simulatedThrottleRaised = true;
    for (int i = 0; i < 100; i++) {
        pidController(pidProfile, currentTestTime());
        ladrcPostMixerUpdate(1.0f, true);
    }
    EXPECT_GT(fabsf(pidData[FD_ROLL].I), 1.0f);

    pidSetItermReset(true);
    gyro.gyroADCf[FD_ROLL] = 250.0f;
    pidController(pidProfile, currentTestTime());
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(0.0f, st->z3);
    EXPECT_GT(st->z2, 0.0f);
    EXPECT_LT(pidData[FD_ROLL].D, 0.0f);
}

TEST(pidControllerTest, testAngleModeBypassesDoubleTdFiltering)
{
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    sensorsSet(SENSOR_ACC);
    ENABLE_FLIGHT_MODE(ANGLE_MODE);
    setStickPosition(FD_ROLL, 0.5f);

    pidController(pidProfile, currentTestTime());
    // Closed-loop ANGLE_MODE must bypass LADRC TD (no F-term derivative of already-filtered angleRate)
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].F);
    EXPECT_GT(fabsf(pidData[FD_ROLL].Sum), 0.0f);
    DISABLE_FLIGHT_MODE(ANGLE_MODE);
}

TEST(pidControllerTest, testCrashRecoveryResetsDisturbanceAndBypassesTd)
{
    resetTest();
    pidProfile->crash_recovery = PID_CRASH_RECOVERY_ON;
    pidProfile->crash_delay = 0;
    pidProfile->crash_limit_yaw = 120;
    pidInit(pidProfile);
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    sensorsSet(SENSOR_ACC);

    // Build some disturbance state first
    setStickPosition(FD_ROLL, 0.3f);
    for (int i = 0; i < 40; i++) {
        pidController(pidProfile, currentTestTime());
        ladrcPostMixerUpdate(1.0f, true);
    }
    EXPECT_GT(fabsf(pidData[FD_ROLL].I), 0.5f);

    // Trigger crash detection with saturated mix and large gyro delta on roll + huge yaw rate error
    attitude.values.roll = 300; // 30 deg
    simulatedMotorMixRange = 1.2f;
    setStickPosition(FD_ROLL, 0.0f);
    setStickPosition(FD_YAW, 0.9f); // ~600 deg/s >> crash_limit_yaw 120
    gyro.gyroADCf[FD_ROLL] = 800.0f;
    gyro.gyroADCf[FD_YAW] = 0.0f;
    pidController(pidProfile, currentTestTime() + 2000000);
    ladrcPostMixerUpdate(1.0f, true);
    ASSERT_TRUE(crashRecoveryModeActive());

    // Next cycle during crash recovery: disturbance/I reset + TD/F bypass on roll/pitch + active D-term damping preserved + yaw setpoint clamped
    pidController(pidProfile, currentTestTime() + 2000000);
    const ladrcRuntimeAxis_t *stRoll = ladrcGetAxisState(FD_ROLL);
    ASSERT_NE(stRoll, nullptr);
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(0.0f, stRoll->z3);
    EXPECT_GT(stRoll->z2, 0.0f);
    EXPECT_LT(pidData[FD_ROLL].D, 0.0f);
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].F);
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_YAW].I);
    EXPECT_LE(fabsf(pidRuntime.previousPidSetpoint[FD_YAW] - gyro.gyroADCf[FD_YAW]), 120.1f);

    // Subsequent step after post-mixer update during crash recovery continues D-term damping while keeping disturbance reset
    ladrcPostMixerUpdate(1.0f, true);
    EXPECT_GT(stRoll->z2, 0.0f);
    gyro.gyroADCf[FD_ROLL] = 1200.0f;
    pidController(pidProfile, currentTestTime() + 2000000);
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(0.0f, stRoll->z3);
    EXPECT_LT(pidData[FD_ROLL].D, 0.0f);
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].F);
}

TEST(pidControllerTest, testAcroTrainerResetsDisturbanceWhilePreservingDamping)
{
    resetTest();
    pidProfile->acro_trainer_angle_limit = 20;
    pidProfile->acro_trainer_gain = 75;
    pidInit(pidProfile);
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    sensorsSet(SENSOR_ACC);
    pidSetAcroTrainerState(true);

    // Build disturbance trim on roll first
    setStickPosition(FD_ROLL, 0.3f);
    for (int i = 0; i < 40; i++) {
        pidController(pidProfile, currentTestTime());
        ladrcPostMixerUpdate(1.0f, true);
    }
    const float preTrimI = pidData[FD_ROLL].I;
    EXPECT_GT(fabsf(preTrimI), 0.5f);

    // Exceed angle limit in stick direction with active gyro rate -> Acro Trainer shapes setpoint
    // while preserving both D-term damping and non-zero disturbance/I trim!
    attitude.values.roll = 300; // 30 deg > 20 deg limit
    gyro.gyroADCf[FD_ROLL] = 200.0f;
    pidController(pidProfile, currentTestTime());
    const ladrcRuntimeAxis_t *stRoll = ladrcGetAxisState(FD_ROLL);
    ASSERT_NE(stRoll, nullptr);
    EXPECT_LT(pidRuntime.previousPidSetpoint[FD_ROLL], 0.0f);
    EXPECT_GT(fabsf(pidData[FD_ROLL].I), 0.5f);
    EXPECT_NEAR(stRoll->distTermSum, pidData[FD_ROLL].I, 1e-3f);
    EXPECT_GT(stRoll->z2, 0.0f);
    EXPECT_LT(pidData[FD_ROLL].D, 0.0f);
    pidSetAcroTrainerState(false);
}

TEST(pidControllerTest, testIndiAutoAngleGainAndDtermFilterBypass)
{
    resetTest();
    gyro.targetLooptime = 500; // 2 kHz PID loop so 100Hz/260Hz legacy filters are below Nyquist
    // 4-Param Auto INDI (manual_override == 0):
    // 5.0" default -> wc_roll = 18.0 -> K_angle = 4.5 deg/s/deg, and legacy D-term filters bypassed to nullFilterApply
    ladrcProfile_t *ladrcProf = ladrcProfilesMutable(systemConfig()->pidProfileIndex);
    ladrcProf->prop_size_deci_in = 50;
    ladrcProf->agility_pct = 100;
    ladrcProf->manual_override = 0;
    pidInit(pidProfile);
    EXPECT_NEAR(indiGetAngleGain(), 4.5f, 1e-4f);
    EXPECT_EQ(pidRuntime.dtermLowpassApplyFn, nullFilterApply);
    EXPECT_EQ(pidRuntime.dtermLowpass2ApplyFn, nullFilterApply);
    EXPECT_EQ(pidRuntime.dtermNotchApplyFn, nullFilterApply);
    EXPECT_EQ(pidRuntime.ptermYawLowpassApplyFn, nullFilterApply);

    // 20.0" prop -> wc_roll = 9.0 -> K_angle = 2.25 deg/s/deg
    ladrcProf->prop_size_deci_in = 200;
    pidInit(pidProfile);
    EXPECT_NEAR(indiGetAngleGain(), 2.25f, 1e-4f);

    // Manual override re-enables legacy D-term filters
    ladrcProf->manual_override = 1;
    pidInit(pidProfile);
    EXPECT_NE(pidRuntime.dtermLowpassApplyFn, nullFilterApply);
    EXPECT_NE(pidRuntime.dtermNotchApplyFn, nullFilterApply);
}
