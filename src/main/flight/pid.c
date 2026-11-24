/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/simplified_tuning.h"

#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/ladrc.h"
#include "flight/mixer.h"

#include "io/gps.h"
#ifdef USE_COMPANION_LINK
#include "io/companion_link.h"
#endif

#include "pg/autopilot.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/autopilot.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

typedef enum {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
} levelMode_e;

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

FAST_DATA_ZERO_INIT uint32_t targetPidLooptime;
FAST_DATA_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];
FAST_DATA_ZERO_INIT pidRuntime_t pidRuntime;

#if defined(USE_THROTTLE_BOOST)
FAST_DATA_ZERO_INIT float throttleBoost;
pt1Filter_t throttleLpf;
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 4);

#ifndef DEFAULT_PID_PROCESS_DENOM
#define DEFAULT_PID_PROCESS_DENOM       1
#endif

#ifdef USE_RUNAWAY_TAKEOFF
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = DEFAULT_PID_PROCESS_DENOM,
    .runaway_takeoff_prevention = true,
    .runaway_takeoff_deactivate_throttle = 20,  // throttle level % needed to accumulate deactivation time
    .runaway_takeoff_deactivate_delay = 500,    // Accumulated time (in milliseconds) before deactivation in successful takeoff
);
#else
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = DEFAULT_PID_PROCESS_DENOM,
);
#endif

#ifdef USE_ACRO_TRAINER
#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f  // Max gyro rate for lookahead time scaling
#define ACRO_TRAINER_SETPOINT_LIMIT       1000.0f // Limit the correcting setpoint
#endif // USE_ACRO_TRAINER

#define CRASH_RECOVERY_DETECTION_DELAY_US 1000000  // 1 second delay before crash recovery detection is active after entering a self-level mode

#define LAUNCH_CONTROL_YAW_ITERM_LIMIT 50 // yaw iterm windup limit when launch mode is "FULL" (all axes)

#ifdef USE_ACC
#define IS_AXIS_IN_ANGLE_MODE(i) (pidRuntime.axisInAngleMode[(i)])
#else
#define IS_AXIS_IN_ANGLE_MODE(i) false
#endif // USE_ACC

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 11);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_PITCH_DEFAULT,
            [PID_YAW] =   PID_YAW_DEFAULT,
            [PID_LEVEL] = { 50, 75, 75, 50, 0 },
            [PID_MAG] =   { 40, 0, 0, 0, 0 },
        },
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .yaw_lowpass_hz = 100,
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 0,
        .itermWindup = 80,         // sets iTerm limit to this percentage below pidSumLimit
        .pidAtMinThrottle = PID_STABILISATION_ON,
        .angle_limit = 60,
        .feedforward_transition = 0,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
        .anti_gravity_gain = 80,
        .crash_time = 500,          // ms
        .crash_delay = 0,           // ms
        .crash_recovery_angle = 10, // degrees
        .crash_recovery_rate = 100, // degrees/second
        .crash_dthreshold = 50,     // degrees/second/second
        .crash_gthreshold = 400,    // degrees/second
        .crash_setpoint_threshold = 350, // degrees/second
        .crash_recovery = PID_CRASH_RECOVERY_OFF, // off by default
        .horizon_limit_degrees = 135,
        .horizon_ignore_sticks = false,
        .crash_limit_yaw = 200,
        .itermLimit = 400,
        .throttle_boost = 5,
        .throttle_boost_cutoff = 15,
        .iterm_rotation = false,
        .iterm_relax = ITERM_RELAX_RP,
        .iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .acro_trainer_gain = 75,
        .dterm_lpf1_static_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
            // NOTE: dynamic lpf is enabled by default so this setting is actually
            // overridden and the static lowpass 1 is disabled. We can't set this
            // value to 0 otherwise Configurator versions 10.4 and earlier will also
            // reset the lowpass filter type to PT1 overriding the desired Butterworth setting.
        .dterm_lpf2_static_hz = DTERM_LPF2_HZ_DEFAULT,   // second Dterm LPF ON by default
        .dterm_lpf1_type = FILTER_PT1,
        .dterm_lpf2_type = FILTER_PT1,
        .dterm_lpf1_dyn_min_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
        .dterm_lpf1_dyn_max_hz = DTERM_LPF1_DYN_MAX_HZ_DEFAULT,
        .launchControlMode = LAUNCH_CONTROL_MODE_PITCHONLY,
        .launchControlThrottlePercent = 20,
        .launchControlAngleLimit = 0,
        .launchControlGain = 40,
        .launchControlAllowTriggerReset = true,
        .use_integrated_yaw = false,
        .integrated_yaw_relax = 200,
        .thrustLinearization = 0,
        .d_max = D_MAX_DEFAULT,
        .d_max_gain = 0,
        .d_max_advance = 35,
        .motor_output_limit = 100,
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .profileName = { 0 },
        .dyn_idle_min_rpm = 0,
        .dyn_idle_p_gain = 50,
        .dyn_idle_i_gain = 50,
        .dyn_idle_d_gain = 50,
        .dyn_idle_max_increase = 150,
        .feedforward_averaging = FEEDFORWARD_AVERAGING_2_POINT,
        .feedforward_max_rate_limit = 90,
        .feedforward_smooth_factor = 65,
        .feedforward_jitter_factor = 7,
        .feedforward_boost = 15,
        .dterm_lpf1_dyn_expo = 5,
        .level_race_mode = false,
        .vbat_sag_compensation = 0,
        .simplified_pids_mode = PID_SIMPLIFIED_TUNING_RPY,
        .simplified_master_multiplier = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_roll_pitch_ratio = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_i_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_d_gain = SIMPLIFIED_TUNING_D_DEFAULT,
        .simplified_pi_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_d_max_gain = SIMPLIFIED_TUNING_D_DEFAULT,
        .simplified_feedforward_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_pitch_pi_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_dterm_filter = true,
        .simplified_dterm_filter_multiplier = SIMPLIFIED_TUNING_DEFAULT,
        .anti_gravity_cutoff_hz = 5,
        .anti_gravity_p_gain = 100,
        .tpa_mode = TPA_MODE_D,
        .tpa_rate = 65,
        .tpa_breakpoint = 1350,
        .angle_feedforward_smoothing_ms = 80,
        .angle_earth_ref = 100,
        .horizon_delay_ms = 500, // 500ms time constant on any increase in horizon strength
        .tpa_low_rate = 20,
        .tpa_low_breakpoint = 1050,
        .tpa_low_always = 0,
        .ez_landing_threshold = 25,
        .ez_landing_limit = 15,
        .ez_landing_speed = 50,
        .spa_center = { 0, 0, 0 },
        .spa_width = { 0, 0, 0 },
        .spa_mode = { 0, 0, 0 },
        .landing_disarm_threshold = 0, // relatively safe values are around 100
        .feedforward_yaw_hold_gain = 15,  // zero disables; 15-20 is OK for 5in
        .feedforward_yaw_hold_time = 100,  // a value of 100 is a time constant of about 100ms, and is OK for a 5in; smaller values decay faster, eg for smaller props
        .tpa_curve_type = TPA_CURVE_CLASSIC,
        .tpa_curve_stall_throttle = 30,
        .tpa_curve_pid_thr0 = 200,
        .tpa_curve_pid_thr100 = 70,
        .tpa_curve_expo = 20,
        .tpa_speed_type = TPA_SPEED_BASIC,
        .tpa_speed_basic_delay = 1000,
        .tpa_speed_basic_gravity = 50,
        .tpa_speed_adv_prop_pitch = 370,
        .tpa_speed_adv_mass = 1000,
        .tpa_speed_adv_drag_k = 1000,
        .tpa_speed_adv_thrust = 2000,
        .tpa_speed_max_voltage = 2520,
        .tpa_speed_pitch_offset = 0,
        .yaw_type = YAW_TYPE_RUDDER,
        .angle_pitch_offset = 0,
        .chirp_lag_freq_hz = 3,
        .chirp_lead_freq_hz = 30,
        .chirp_amplitude_roll = 230,
        .chirp_amplitude_pitch = 230,
        .chirp_amplitude_yaw = 180,
        .chirp_frequency_start_deci_hz = 2,
        .chirp_frequency_end_deci_hz = 6000,
        .chirp_time_seconds = 20,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

// Scale factors to make best use of range with D_LPF debugging, aiming for max +/-16K as debug values are 16 bit
#define D_LPF_RAW_SCALE 25
#define D_LPF_PRE_TPA_SCALE 10

void pidSetItermAccelerator(float newItermAccelerator)
{
    pidRuntime.itermAccelerator = newItermAccelerator;
}

bool pidOsdAntiGravityActive(void)
{
    return (pidRuntime.itermAccelerator > pidRuntime.antiGravityOsdCutoff);
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidRuntime.pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

bool pidIsStabilisationEnabled(void)
{
    return pidRuntime.pidStabilisationEnabled;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
    }
#ifdef USE_LADRC
    ladrcResetDisturbance();
#endif
}

#ifdef USE_WING
float pidGetWingAirspeedCms(void)
{
    return pidRuntime.tpaSpeed.speed * 100.0f;
}

static float calcWingThrottle(void)
{
    float batteryThrottleFactor = 1.0f;
    if (pidRuntime.tpaSpeed.maxVoltage > 0.0f) {
        batteryThrottleFactor = getBatteryVoltageLatest() / 100.0f / pidRuntime.tpaSpeed.maxVoltage;
        batteryThrottleFactor = constrainf(batteryThrottleFactor, 0.0f, 1.0f);
    }

    return getMotorOutputRms() * batteryThrottleFactor;
}

static float calcWingAcceleration(float throttle, float pitchAngleRadians)
{
    const tpaSpeedParams_t *tpa = &pidRuntime.tpaSpeed;

    const float thrust = (throttle * throttle - throttle * tpa->speed * tpa->inversePropMaxSpeed) * tpa->twr * G_ACCELERATION;
    const float drag = tpa->speed * tpa->speed * tpa->dragMassRatio;
    const float gravity = G_ACCELERATION * sin_approx(pitchAngleRadians);

    return thrust - drag + gravity;
}

static float calcWingTpaArgument(void)
{
    if (!ARMING_FLAG(ARMED)) {
        pidRuntime.tpaSpeed.speed = 0.0f;
        DEBUG_SET(DEBUG_TPA, 4, 0);
        DEBUG_SET(DEBUG_TPA, 5, 0);
        return 0.0f;
    }

    const float t = calcWingThrottle();
    const float pitchRadians = DECIDEGREES_TO_RADIANS(attitude.values.pitch);
    const float rollRadians = DECIDEGREES_TO_RADIANS(attitude.values.roll);

    DEBUG_SET(DEBUG_TPA, 1, lrintf(attitude.values.roll)); // decidegrees
    DEBUG_SET(DEBUG_TPA, 2, lrintf(attitude.values.pitch)); // decidegrees
    DEBUG_SET(DEBUG_TPA, 3, lrintf(t * 1000.0f)); // calculated throttle in the range of 0 - 1000

    // pitchRadians is always -90 to 90 degrees. The bigger the ABS(pitch) the less portion of pitchOffset is needed.
    // If ABS(roll) > 90 degrees - flying inverted, then negative portion of pitchOffset is needed.
    // If ABS(roll) ~ 90 degrees - flying sideways, no pitchOffset is applied.
    const float correctedPitchAnge = pitchRadians + cos_approx(pitchRadians) * cos_approx(rollRadians) * pidRuntime.tpaSpeed.pitchOffset;

    const float a = calcWingAcceleration(t, correctedPitchAnge);

    pidRuntime.tpaSpeed.speed += a * pidRuntime.dT;
    pidRuntime.tpaSpeed.speed = MAX(0.0f, pidRuntime.tpaSpeed.speed);
    const float tpaArgument = constrainf(pidRuntime.tpaSpeed.speed / pidRuntime.tpaSpeed.maxSpeed, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_TPA, 4, lrintf(pidRuntime.tpaSpeed.speed * 10.0f));
    DEBUG_SET(DEBUG_TPA, 5, lrintf(tpaArgument * 1000.0f));

    return tpaArgument;
}

static void updateStermTpaFactor(int axis, float tpaFactor)
{
    float tpaFactorSterm = tpaFactor;
    if (pidRuntime.tpaCurveType == TPA_CURVE_HYPERBOLIC) {
        const float maxSterm = tpaFactorSterm * (float)currentPidProfile->pid[axis].S * S_TERM_SCALE;
        if (maxSterm > 1.0f) {
            tpaFactorSterm *=  1.0f / maxSterm;
        }
    }
    pidRuntime.tpaFactorSterm[axis] = tpaFactorSterm;
}

static void updateStermTpaFactors(void) {
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        float tpaFactor = pidRuntime.tpaFactor;
        if (i == FD_YAW && currentPidProfile->yaw_type == YAW_TYPE_DIFF_THRUST) {
            tpaFactor = pidRuntime.tpaFactorYaw;
        }
        updateStermTpaFactor(i, tpaFactor);
    }
}
#endif // USE_WING

static float wingAdjustSetpoint(float currentPidSetpoint, int axis)
{
#ifdef USE_WING
    float adjustedSetpoint = currentPidSetpoint;
    if (!IS_AXIS_IN_ANGLE_MODE(axis)) {
        const bool skipYaw = axis == FD_YAW && currentPidProfile->yaw_type == YAW_TYPE_DIFF_THRUST;
        if (pidRuntime.tpaFactorSterm[axis] > 0.0f && pidRuntime.tpaFactor > 0.0f && !skipYaw) {
            adjustedSetpoint = currentPidSetpoint * pidRuntime.tpaFactorSterm[axis] / pidRuntime.tpaFactor;
        }
    }

    DEBUG_SET(DEBUG_WING_SETPOINT, 2 * axis, lrintf(currentPidSetpoint));
    DEBUG_SET(DEBUG_WING_SETPOINT, 2 * axis + 1, lrintf(adjustedSetpoint));
    return adjustedSetpoint;
#else
    UNUSED(axis);
    return currentPidSetpoint;
#endif // USE_WING
}

static float getTpaFactorClassic(float tpaArgument)
{
    static bool isTpaLowFaded = false;
    bool isThrottlePastTpaLowBreakpoint = (tpaArgument >= pidRuntime.tpaLowBreakpoint || pidRuntime.tpaLowBreakpoint <= 0.01f);
    float tpaRate = 0.0f;
    if (isThrottlePastTpaLowBreakpoint || isTpaLowFaded) {
        tpaRate = pidRuntime.tpaMultiplier * fmaxf(tpaArgument - pidRuntime.tpaBreakpoint, 0.0f);
        if (!pidRuntime.tpaLowAlways && !isTpaLowFaded) {
            isTpaLowFaded = true;
        }
    } else {
        tpaRate = pidRuntime.tpaLowMultiplier * (pidRuntime.tpaLowBreakpoint - tpaArgument);
    }

    return 1.0f - tpaRate;
}

void pidUpdateTpaFactor(float throttle)
{
    throttle = constrainf(throttle, 0.0f, 1.0f);
    float tpaFactor;

#ifdef USE_WING
    const float tpaArgument = isFixedWing() ?  calcWingTpaArgument() : throttle;
#else
    const float tpaArgument = throttle;
#endif

#ifdef USE_ADVANCED_TPA
    switch (pidRuntime.tpaCurveType) {
    case TPA_CURVE_HYPERBOLIC:
        tpaFactor = pwlInterpolate(&pidRuntime.tpaCurvePwl, tpaArgument);
        break;
    case TPA_CURVE_CLASSIC:
    default:
        tpaFactor = getTpaFactorClassic(tpaArgument);
    }
#else
    tpaFactor = getTpaFactorClassic(tpaArgument);
#endif

    DEBUG_SET(DEBUG_TPA, 0, lrintf(tpaFactor * 1000));
    pidRuntime.tpaFactor = tpaFactor;

#ifdef USE_WING
    switch (currentPidProfile->yaw_type) {
    case YAW_TYPE_DIFF_THRUST:
        pidRuntime.tpaFactorYaw = getTpaFactorClassic(tpaArgument);
        break;
    case YAW_TYPE_RUDDER:
    default:
        pidRuntime.tpaFactorYaw = pidRuntime.tpaFactor;
        break;
    }
    updateStermTpaFactors();
#endif // USE_WING
}

void pidUpdateAntiGravityThrottleFilter(float throttle)
{
    static float previousThrottle = 0.0f;
    const float throttleInv = 1.0f - throttle;
    float throttleDerivative = fabsf(throttle - previousThrottle) * pidRuntime.pidFrequency;
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 0, lrintf(throttleDerivative * 100));
    throttleDerivative *= throttleInv * throttleInv;
    // generally focus on the low throttle period
    if (throttle > previousThrottle) {
        throttleDerivative *= throttleInv * 0.5f;
        // when increasing throttle, focus even more on the low throttle range
    }
    previousThrottle = throttle;
    throttleDerivative = pt2FilterApply(&pidRuntime.antiGravityLpf, throttleDerivative);
    // lower cutoff suppresses peaks relative to troughs and prolongs the effects
    // PT2 smoothing of throttle derivative.
    // 6 is a typical value for the peak boost factor with default cutoff of 6Hz
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 1, lrintf(throttleDerivative * 100));
    pidRuntime.antiGravityThrottleD = throttleDerivative;
}

#ifdef USE_ACRO_TRAINER
void pidAcroTrainerInit(void)
{
    pidRuntime.acroTrainerAxisState[FD_ROLL] = 0;
    pidRuntime.acroTrainerAxisState[FD_PITCH] = 0;
}
#endif // USE_ACRO_TRAINER

#ifdef USE_THRUST_LINEARIZATION
// Compensate motor command for a non-linear (~quadratic) motor thrust response.
// Uses a 2nd-order Taylor expansion of ArduPilot's MOT_THST_EXPO model:
//   c(x) = x * (1 + e * (1-x) * (1 + e * (1-2x)))
// where e = thrustLinearization (0..1.5, scaled from the 0..150 CLI value).
// This matches the small-e Taylor series of (1-e)*m + e*m^2 to second order,
// so thrust_linear ≈ MOT_THST_EXPO * 100 and ArduPilot's per-prop recommendations
// (≈55 for 5", ≈65 for 10", ≈75 for 20"+) port directly.
//
// The 2nd-order term flips sign at x=0.5, dropping the curve's slope at full
// throttle from 1.0 to (1 - e + e^2). This prevents the current cubic shape
// from going tangent to identity at x=1 and softens the motor's natural
// steepening near the top of the throttle range.
//
// Cost: 4 mults, 2 adds, 2 subs per motor (no sqrt). Called per-motor per loop.
float pidApplyThrustLinearization(float motorOutput)
{
    const float e = pidRuntime.thrustLinearization;
    const float inv = 1.0f - motorOutput;
    motorOutput *= 1.0f + e * inv * (1.0f + e * (inv - motorOutput));
    return motorOutput;
}

// Inverse of the curve above, applied to base throttle so hover throttle is
// preserved when thrust linearization is active. Expanding c⁻¹(y) as a Taylor
// series in e gives c⁻¹(y) = y - e·y·(1-y) + 0·e² + O(e³): the e² coefficient
// vanishes by algebraic cancellation, so this two-term expression is accurate
// to second order in e — matching the forward — and is cheaper than dividing
// by the full forward multiplier.
float pidCompensateThrustLinearization(float throttle)
{
    if (pidRuntime.thrustLinearization != 0.0f) {
        const float e = pidRuntime.thrustLinearization;
        throttle *= 1.0f - e * (1.0f - throttle);
    }
    return throttle;
}

float pidGetThrustLinearization(void)
{
    return pidRuntime.thrustLinearization;
}
#endif

#if defined(USE_ACC)
// Calculate strength of horizon leveling; 0 = none, 1.0 = most leveling
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float calcHorizonLevelStrength(void)
{
    const float currentInclination = MAX(abs(attitude.values.roll), abs(attitude.values.pitch)) * 0.1f;
    // 0 when level, 90 when vertical, 180 when inverted (degrees):
    float absMaxStickDeflection = MAX(fabsf(getRcDeflection(FD_ROLL)), fabsf(getRcDeflection(FD_PITCH)));
    // 0-1, smoothed if RC smoothing is enabled

    float horizonLevelStrength = MAX((pidRuntime.horizonLimitDegrees - currentInclination) * pidRuntime.horizonLimitDegreesInv, 0.0f);
    // 1.0 when attitude is 'flat', 0 when angle is equal to, or greater than, horizonLimitDegrees
    horizonLevelStrength *= MAX((pidRuntime.horizonLimitSticks - absMaxStickDeflection) * pidRuntime.horizonLimitSticksInv, pidRuntime.horizonIgnoreSticks);
    // use the value of horizonIgnoreSticks to enable/disable this effect.
    // value should be 1.0 at center stick, 0.0 at max stick deflection:
    horizonLevelStrength *= pidRuntime.horizonGain;

    if (pidRuntime.horizonDelayMs) {
        const float horizonLevelStrengthSmoothed = pt1FilterApply(&pidRuntime.horizonSmoothingPt1, horizonLevelStrength);
        horizonLevelStrength = MIN(horizonLevelStrength, horizonLevelStrengthSmoothed);
    }
    return horizonLevelStrength;
    // 1 means full levelling, 0 means none
}

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim,
                                                        float currentPidSetpoint, float horizonLevelStrength)
{
    // Applies only to axes that are in Angle mode
    // We now use Acro Rates, transformed into the range +/- 1, to provide setpoints
    float angleLimit = pidProfile->angle_limit;
    float angleFeedforward = 0.0f;
    // if user changes rates profile, update the max setpoint for angle mode
    const float maxSetpointRateInv = 1.0f / getMaxRcRate(axis);

#ifdef USE_FEEDFORWARD
    angleFeedforward = angleLimit * getFeedforward(axis) * pidRuntime.angleFeedforwardGain * maxSetpointRateInv;
    //  angle feedforward must be heavily filtered, at the PID loop rate, with limited user control over time constant
    // it MUST be very delayed to avoid early overshoot and being too aggressive
    angleFeedforward = pt3FilterApply(&pidRuntime.angleFeedforwardPt3[axis], angleFeedforward);
#endif

    float angleTarget = angleLimit * currentPidSetpoint * maxSetpointRateInv;
    // use acro rates for the angle target in both horizon and angle modes, converted to -1 to +1 range using maxRate

#ifdef USE_WING
    if (axis == FD_PITCH) {
        angleTarget += (float)pidProfile->angle_pitch_offset / 10.0f;
    }
#endif // USE_WING

#ifdef USE_COMPANION_LINK
    const bool companionAttitudeActive = companionLinkIsOffboardActive()
        && (companionLinkGetSetpoint()->mode == COMPANION_MODE_ATTITUDE_THRUST);
    const bool companionLevelHold = companionLinkIsLevelHoldActive()
        || (companionLinkShouldTriggerRth() && !FLIGHT_MODE(GPS_RESCUE_MODE));
    if (companionAttitudeActive) {
        const companionSetpoint_t *sp = companionLinkGetSetpoint();
        angleTarget = sp->target[axis];
        angleFeedforward = 0.0f;
    } else if (companionLevelHold) {
        angleTarget = 0.0f;
        angleFeedforward = 0.0f;
    }
#else
    const bool companionAttitudeActive = false;
    const bool companionLevelHold = false;
#endif

#ifdef USE_GPS_RESCUE
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        angleTarget = autopilotAngle[axis]; // autopilotAngle in degrees
        angleLimit = (float)autopilotConfig()->maxAngle;
        angleFeedforward = 0.0f;
    }
#endif
#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        angleFeedforward = 0.0f; // otherwise the lag of the PT3 carries recent stick inputs into the hold
        if (isAutopilotInControl()) {
            // sticks are not deflected
            angleTarget = autopilotAngle[axis]; // autopilotAngle in degrees
            angleLimit = 85.0f; // allow autopilot to use whatever angle it needs to stop
        }
        // limit pilot requested angle to max autopilot angle to avoid excess speed
        angleLimit = fminf((float)autopilotConfig()->maxAngle, angleLimit);
    }
#endif

    angleTarget = constrainf(angleTarget, -angleLimit, angleLimit);

    const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f; // stepped at 500hz with some 4ms flat spots
    const float errorAngle = angleTarget - currentAngle;
    float angleGain = pidRuntime.angleGain;
#ifdef USE_LADRC
    if (!ladrcProfiles(systemConfig()->pidProfileIndex)->manual_override) {
        angleGain = indiGetAngleGain();
    }
#endif
    float angleRate = errorAngle * angleGain + angleFeedforward;

    // minimise cross-axis wobble due to faster yaw responses than roll or pitch, and make co-ordinated yaw turns
    // by compensating for the effect of yaw on roll while pitched, and on pitch while rolled
    // earthRef code here takes about 76 cycles, if conditional on angleEarthRef it takes about 100.  sin_approx costs most of those cycles.
    float sinAngle = sin_approx(DEGREES_TO_RADIANS(pidRuntime.angleTarget[axis == FD_ROLL ? FD_PITCH : FD_ROLL]));
    sinAngle *= (axis == FD_ROLL) ? -1.0f : 1.0f; // must be negative for Roll
    const float earthRefGain = FLIGHT_MODE(GPS_RESCUE_MODE | ALT_HOLD_MODE) ? 1.0f : pidRuntime.angleEarthRef;
    angleRate += pidRuntime.angleYawSetpoint * sinAngle * earthRefGain;
    pidRuntime.angleTarget[axis] = angleTarget;  // set target for alternate axis to current axis, for use in preceding calculation

    // smooth final angle rate output to clean up attitude signal steps (500hz), GPS steps (10 or 100hz), RC steps etc
    // this filter runs at ATTITUDE_CUTOFF_HZ, currently 50hz, so GPS roll may be a bit steppy
    angleRate = pt3FilterApply(&pidRuntime.attitudeFilter[axis], angleRate);

    if (FLIGHT_MODE(ANGLE_MODE | GPS_RESCUE_MODE | POS_HOLD_MODE) || companionAttitudeActive || companionLevelHold) {
        currentPidSetpoint = angleRate;
    } else {
        // can only be HORIZON mode - crossfade Angle rate and Acro rate
        currentPidSetpoint = currentPidSetpoint * (1.0f - horizonLevelStrength) + angleRate * horizonLevelStrength;
    }

    //logging
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_ANGLE_MODE, 0, lrintf(angleTarget * 10.0f)); // target angle
        DEBUG_SET(DEBUG_ANGLE_MODE, 1, lrintf(errorAngle * angleGain * 10.0f)); // un-smoothed error correction in degrees
        DEBUG_SET(DEBUG_ANGLE_MODE, 2, lrintf(angleFeedforward * 10.0f)); // feedforward amount in degrees
        DEBUG_SET(DEBUG_ANGLE_MODE, 3, lrintf(currentAngle * 10.0f)); // angle returned

        DEBUG_SET(DEBUG_ANGLE_TARGET, 0, lrintf(angleTarget * 10.0f));
        DEBUG_SET(DEBUG_ANGLE_TARGET, 1, lrintf(sinAngle * 10.0f)); // modification factor from earthRef
        // debug ANGLE_TARGET 2 is yaw attenuation
        DEBUG_SET(DEBUG_ANGLE_TARGET, 3, lrintf(currentAngle * 10.0f)); // angle returned
    }

    DEBUG_SET(DEBUG_CURRENT_ANGLE, axis, lrintf(currentAngle * 10.0f)); // current angle
    return currentPidSetpoint;
}

static FAST_CODE_NOINLINE void handleCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const rollAndPitchTrims_t *angleTrim,
    const int axis, const timeUs_t currentTimeUs, const float gyroRate, float *currentPidSetpoint, float *errorRate,
    bool *resetDisturbance)
{
    if (pidRuntime.inCrashRecoveryMode && cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) > pidRuntime.crashTimeDelayUs) {
        if (crash_recovery == PID_CRASH_RECOVERY_BEEP) {
            BEEP_ON;
        }
        if (axis == FD_YAW) {
            *errorRate = constrainf(*errorRate, -pidRuntime.crashLimitYaw, pidRuntime.crashLimitYaw);
            *currentPidSetpoint = gyroRate + *errorRate;
        } else {
            // on roll and pitch axes calculate currentPidSetpoint and errorRate to level the aircraft to recover from crash
            if (sensors(SENSOR_ACC)) {
                // errorAngle is deviation from horizontal
                const float errorAngle =  -(attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
                float angleGain = pidRuntime.angleGain;
#ifdef USE_LADRC
                if (!ladrcProfiles(systemConfig()->pidProfileIndex)->manual_override) {
                    angleGain = indiGetAngleGain();
                }
#endif
                *currentPidSetpoint = errorAngle * angleGain;
                *errorRate = *currentPidSetpoint - gyroRate;
            }
        }
        // Reset LADRC disturbance windup before stepping LSEF while keeping Order-2 D-term damping active
        if (resetDisturbance) {
            *resetDisturbance = true;
        }
        if (cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) > pidRuntime.crashTimeLimitUs
            || (getMotorMixRange() < 1.0f
                   && fabsf(gyro.gyroADCf[FD_ROLL]) < pidRuntime.crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_PITCH]) < pidRuntime.crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_YAW]) < pidRuntime.crashRecoveryRate)) {
            if (sensors(SENSOR_ACC)) {
                // check aircraft nearly level
                if (abs(attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) < pidRuntime.crashRecoveryAngleDeciDegrees
                   && abs(attitude.raw[FD_PITCH] - angleTrim->raw[FD_PITCH]) < pidRuntime.crashRecoveryAngleDeciDegrees) {
                    pidRuntime.inCrashRecoveryMode = false;
                    BEEP_OFF;
                }
            } else {
                pidRuntime.inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        }
    }
}

static FAST_CODE_NOINLINE void detectAndSetCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const int axis,
    const timeUs_t currentTimeUs, const float delta, const float errorRate)
{
    if (!ARMING_FLAG(ARMED)) {
        if (pidRuntime.inCrashRecoveryMode) {
            pidRuntime.inCrashRecoveryMode = false;
            BEEP_OFF;
        }
        return;
    }

    // if crash recovery is on and accelerometer enabled and there is no gyro overflow, then check for a crash
    // no point in trying to recover if the crash is so severe that the gyro overflows
    if ((crash_recovery || FLIGHT_MODE(GPS_RESCUE_MODE)) && !gyroOverflowDetected()) {
        if (getMotorMixRange() >= 1.0f && !pidRuntime.inCrashRecoveryMode
            && fabsf(delta) > pidRuntime.crashDtermThreshold
            && fabsf(errorRate) > pidRuntime.crashGyroThreshold
            && fabsf(getSetpointRate(axis)) < pidRuntime.crashSetpointThreshold) {
            if (crash_recovery == PID_CRASH_RECOVERY_DISARM) {
                setArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
                disarm(DISARM_REASON_CRASH_PROTECTION);
            } else {
                pidRuntime.inCrashRecoveryMode = true;
                pidRuntime.crashDetectedAtUs = currentTimeUs;
            }
        }
        if (pidRuntime.inCrashRecoveryMode && cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) < pidRuntime.crashTimeDelayUs && (fabsf(errorRate) < pidRuntime.crashGyroThreshold
            || fabsf(getSetpointRate(axis)) > pidRuntime.crashSetpointThreshold)) {
            pidRuntime.inCrashRecoveryMode = false;
            BEEP_OFF;
        }
    }
}
#endif // USE_ACC

#ifdef USE_ACRO_TRAINER

int acroTrainerSign(float x)
{
    return x > 0 ? 1 : -1;
}

// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
// There are three states:
// 1. Current angle has exceeded limit
//    Apply correction to return to limit (similar to pidLevel)
// 2. Future overflow has been projected based on current angle and gyro rate
//    Manage the setPoint to control the gyro rate as the actual angle  approaches the limit (try to prevent overshoot)
// 3. If no potential overflow is detected, then return the original setPoint

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM. We accept the
// performance decrease when Acro Trainer mode is active under the assumption that user is unlikely to be
// expecting ultimate flight performance at very high loop rates when in this mode.
static FAST_CODE_NOINLINE float applyAcroTrainer(int axis, const rollAndPitchTrims_t *angleTrim, float setPoint)
{
    float ret = setPoint;

    if (!FLIGHT_MODE(ANGLE_MODE  | HORIZON_MODE | GPS_RESCUE_MODE | ALT_HOLD_MODE | POS_HOLD_MODE)) {
        float projectedAngle = 0;
        const int setpointSign = acroTrainerSign(setPoint);
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const int angleSign = acroTrainerSign(currentAngle);

        if ((pidRuntime.acroTrainerAxisState[axis] != 0) && (pidRuntime.acroTrainerAxisState[axis] != setpointSign)) {  // stick has reversed - stop limiting
            pidRuntime.acroTrainerAxisState[axis] = 0;
        }

        // Limit and correct the angle when it exceeds the limit
        if ((fabsf(currentAngle) > pidRuntime.acroTrainerAngleLimit) && (pidRuntime.acroTrainerAxisState[axis] == 0)) {
            if (angleSign == setpointSign) {
                pidRuntime.acroTrainerAxisState[axis] = angleSign;
            }
        }

        if (pidRuntime.acroTrainerAxisState[axis] != 0) {
            ret = constrainf(((pidRuntime.acroTrainerAngleLimit * angleSign) - currentAngle) * pidRuntime.acroTrainerGain, -ACRO_TRAINER_SETPOINT_LIMIT, ACRO_TRAINER_SETPOINT_LIMIT);
        } else {

        // Not currently over the limit so project the angle based on current angle and
        // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
        // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f) * pidRuntime.acroTrainerLookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;
            const int projectedAngleSign = acroTrainerSign(projectedAngle);
            if ((fabsf(projectedAngle) > pidRuntime.acroTrainerAngleLimit) && (projectedAngleSign == setpointSign)) {
                ret = ((pidRuntime.acroTrainerAngleLimit * projectedAngleSign) - projectedAngle) * pidRuntime.acroTrainerGain;
            }
        }

        if (axis == pidRuntime.acroTrainerDebugAxis) {
            DEBUG_SET(DEBUG_ACRO_TRAINER, 0, lrintf(currentAngle * 10.0f));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 1, pidRuntime.acroTrainerAxisState[axis]);
            DEBUG_SET(DEBUG_ACRO_TRAINER, 2, lrintf(ret));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 3, lrintf(projectedAngle * 10.0f));
        }
    }

    return ret;
}
#endif // USE_ACRO_TRAINER

static float accelerationLimit(int axis, float currentPidSetpoint)
{
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];

    if (fabsf(currentVelocity) > pidRuntime.maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + pidRuntime.maxVelocity[axis] : previousSetpoint[axis] - pidRuntime.maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

static FAST_CODE_NOINLINE void disarmOnImpact(void)
{
    // if, being armed, and after takeoff...
    if (wasThrottleRaised()
        // and, either sticks are centred and throttle zeroed,
        && ((getMaxRcDeflectionAbs() < 0.05f && mixerGetRcThrottle() < 0.05f)
#ifdef USE_ALTITUDE_HOLD
            // or, in altitude hold mode, where throttle can be non-zero
            || FLIGHT_MODE(ALT_HOLD_MODE | GPS_RESCUE_MODE)
#endif
        )) {
        // increase sensitivity by 50% when low and in altitude hold or failsafe landing
        // for more reliable disarm with gentle controlled landings
        float lowAltitudeSensitivity = 1.0f;
#ifdef USE_ALTITUDE_HOLD
        lowAltitudeSensitivity = (FLIGHT_MODE(ALT_HOLD_MODE) && isBelowLandingAltitude()) ? 1.5f : 1.0f;
#endif
        // and disarm if jerk exceeds threshold...
        if ((acc.jerkMagnitude * lowAltitudeSensitivity) > pidRuntime.landingDisarmThreshold) {
            // then disarm
            setArmingDisabled(ARMING_DISABLED_ARM_SWITCH); // NB: need a better message
            disarm(DISARM_REASON_LANDING);
            // note: threshold should be high enough to avoid unwanted disarms in the air on throttle chops, eg around 10
        }
    }
    DEBUG_SET(DEBUG_EZLANDING, 6, lrintf(getMaxRcDeflectionAbs() * 100.0f));
    DEBUG_SET(DEBUG_EZLANDING, 7, lrintf(acc.jerkMagnitude * 1e3f));
}

#ifdef USE_LAUNCH_CONTROL
#define LAUNCH_CONTROL_MAX_RATE 100.0f
#define LAUNCH_CONTROL_MIN_RATE 5.0f
#define LAUNCH_CONTROL_ANGLE_WINDOW 10.0f  // The remaining angle degrees where rate dampening starts

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
static FAST_CODE_NOINLINE float applyLaunchControl(int axis, const rollAndPitchTrims_t *angleTrim)
{
    float ret = 0.0f;

    // Scale the rates based on stick deflection only. Fixed rates with a max of 100deg/sec
    // reached at 50% stick deflection. This keeps the launch control positioning consistent
    // regardless of the user's rates.
    if ((axis == FD_PITCH) || (pidRuntime.launchControlMode != LAUNCH_CONTROL_MODE_PITCHONLY)) {
        const float stickDeflection = constrainf(getRcDeflection(axis), -0.5f, 0.5f);
        ret = LAUNCH_CONTROL_MAX_RATE * stickDeflection * 2;
    }

#if defined(USE_ACC)
    // If ACC is enabled and a limit angle is set, then try to limit forward tilt
    // to that angle and slow down the rate as the limit is approached to reduce overshoot
    if ((axis == FD_PITCH) && (pidRuntime.launchControlAngleLimit > 0) && (ret > 0)) {
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        if (currentAngle >= pidRuntime.launchControlAngleLimit) {
            ret = 0.0f;
        } else {
            //for the last 10 degrees scale the rate from the current input to 5 dps
            const float angleDelta = pidRuntime.launchControlAngleLimit - currentAngle;
            if (angleDelta <= LAUNCH_CONTROL_ANGLE_WINDOW) {
                ret = scaleRangef(angleDelta, 0, LAUNCH_CONTROL_ANGLE_WINDOW, LAUNCH_CONTROL_MIN_RATE, ret);
            }
        }
    }
#else
    UNUSED(angleTrim);
#endif

    return ret;
}
#endif

// St4rpony primary flight controller: outer-loop attitude/safety shaping + inner-loop LADRC
void FAST_CODE pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    static float previousGyroRateCrash[XYZ_AXIS_COUNT];

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinActive = gyroYawSpinDetected();
#endif

    const bool launchControlActive = isLaunchControlActive();
    UNUSED(launchControlActive);

#ifdef USE_COMPANION_LINK
    const bool companionOffboard = companionLinkIsOffboardActive();
    const uint8_t companionMode = companionOffboard ? companionLinkGetSetpoint()->mode : COMPANION_MODE_NONE;
    const bool companionOffboardRate = companionOffboard && (companionMode == COMPANION_MODE_RATE_THRUST);
    const bool companionOffboardAttitude = companionOffboard && (companionMode == COMPANION_MODE_ATTITUDE_THRUST);
    const bool companionLevelHold = companionLinkIsLevelHoldActive()
        || (companionLinkShouldTriggerRth() && !FLIGHT_MODE(GPS_RESCUE_MODE));
    const bool companionControlsFlight = companionOffboard || companionLevelHold;
#else
    const bool companionOffboardRate = false;
    const bool companionOffboardAttitude = false;
    const bool companionLevelHold = false;
    const bool companionControlsFlight = false;
#endif
    UNUSED(companionOffboardRate);
    UNUSED(companionOffboardAttitude);
    UNUSED(companionLevelHold);
    UNUSED(companionControlsFlight);

#if defined(USE_ACC)
    static timeUs_t levelModeStartTimeUs = 0;
    static bool prevExternalAngleRequest = false;
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
    float horizonLevelStrength = 0.0f;

    const bool isExternalAngleModeRequest = FLIGHT_MODE(GPS_RESCUE_MODE)
                || companionOffboardAttitude
                || companionLevelHold
#ifdef USE_ALTITUDE_HOLD
                || FLIGHT_MODE(ALT_HOLD_MODE)
#endif
#ifdef USE_POSITION_HOLD
                || FLIGHT_MODE(POS_HOLD_MODE)
#endif
                ;
    levelMode_e levelMode;
    if (companionOffboardRate) {
        levelMode = LEVEL_MODE_OFF;
        levelModeStartTimeUs = 0;
    } else if (((companionOffboardAttitude || companionLevelHold) && sensors(SENSOR_ACC))
        || FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | GPS_RESCUE_MODE)) {
        if (pidRuntime.levelRaceMode && !isExternalAngleModeRequest) {
            levelMode = LEVEL_MODE_R;
        } else {
            levelMode = LEVEL_MODE_RP;
        }

        if ((levelModeStartTimeUs == 0) || (isExternalAngleModeRequest && !prevExternalAngleRequest)) {
            levelModeStartTimeUs = currentTimeUs;
        }

        if (!isExternalAngleModeRequest && !FLIGHT_MODE(ANGLE_MODE) && FLIGHT_MODE(HORIZON_MODE)) {
            horizonLevelStrength = calcHorizonLevelStrength();
        }
    } else {
        levelMode = LEVEL_MODE_OFF;
        levelModeStartTimeUs = 0;
    }

    prevExternalAngleRequest = isExternalAngleModeRequest;
#else
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);
#endif

    if (pidRuntime.useEzDisarm) {
        disarmOnImpact();
    }

#ifdef USE_CHIRP
    static int chirpAxis = 0;
    static bool shouldChirpAxisToggle = false;

    float chirp = 0.0f;
    float sinarg = 0.0f;
    if (FLIGHT_MODE(CHIRP_MODE)) {
        shouldChirpAxisToggle = true;
        if (chirpUpdate(&pidRuntime.chirp)) {
            chirp = pidRuntime.chirp.exc;
            sinarg = pidRuntime.chirp.sinarg;
        }
    } else {
        if (shouldChirpAxisToggle) {
            shouldChirpAxisToggle = false;
            chirpAxis = (++chirpAxis > FD_YAW) ? 0 : chirpAxis;
            chirpReset(&pidRuntime.chirp);
        }
    }

    float chirpFiltered = phaseCompApply(&pidRuntime.chirpFilter, chirp);
    DEBUG_SET(DEBUG_CHIRP, 0, lrintf(5.0e3f * sinarg));
    DEBUG_SET(DEBUG_CHIRP, 1, FLIGHT_MODE(CHIRP_MODE) ? chirpAxis : -1);
    DEBUG_SET(DEBUG_CHIRP, 2, lrintf(10.0f * pidRuntime.chirp.fchirp));
    DEBUG_SET(DEBUG_CHIRP, 3, lrintf(1.0e3f * chirp));
#endif // USE_CHIRP

    const float ladrcB0Scale = ladrcGetExternalB0Scale();

    // ---------- LADRC inner-loop controller ----------
    for (flight_dynamics_index_t axis = FD_ROLL; axis <= FD_YAW; ++axis) {
#ifdef USE_CHIRP
        float currentChirp = 0.0f;
        if (axis == chirpAxis) {
            currentChirp = pidRuntime.chirpAmplitude[axis] * chirpFiltered;
        }
#endif // USE_CHIRP

        float currentPidSetpoint = getSetpointRate(axis);
        if (pidRuntime.maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }

#if defined(USE_ACC)
        pidRuntime.axisInAngleMode[axis] = false;
        if (axis < FD_YAW) {
            if (levelMode == LEVEL_MODE_RP || (levelMode == LEVEL_MODE_R && axis == FD_ROLL)) {
                pidRuntime.axisInAngleMode[axis] = true;
                currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint, horizonLevelStrength);
            }
        } else {
            if (levelMode == LEVEL_MODE_RP) {
                const float earthRefGain = FLIGHT_MODE(GPS_RESCUE_MODE) ? 1.0f : pidRuntime.angleEarthRef;
                if (earthRefGain) {
                    pidRuntime.angleYawSetpoint = currentPidSetpoint;
                    float maxAngleTargetAbs = earthRefGain * fmaxf(fabsf(pidRuntime.angleTarget[FD_ROLL]), fabsf(pidRuntime.angleTarget[FD_PITCH]));
                    maxAngleTargetAbs *= (FLIGHT_MODE(HORIZON_MODE)) ? horizonLevelStrength : 1.0f;
                    currentPidSetpoint *= cos_approx(DEGREES_TO_RADIANS(maxAngleTargetAbs));
                    DEBUG_SET(DEBUG_ANGLE_TARGET, 2, currentPidSetpoint);
                }
            }
#ifdef USE_WING
            if (isFixedWing() && levelMode != LEVEL_MODE_OFF && sensors(SENSOR_ACC)) {
                float airspeedMs = pidGetWingAirspeedCms() * 0.01f;
                if (airspeedMs < 5.0f) {
#ifdef USE_LADRC
                    airspeedMs = ladrcProfiles(systemConfig()->pidProfileIndex)->airspeed_ref_cms * 0.01f;
#else
                    airspeedMs = 15.0f;
#endif
                }
                airspeedMs = constrainf(airspeedMs, 8.0f, 60.0f);
                const float phiRad = DECIDEGREES_TO_RADIANS(attitude.values.roll);
                const float thetaRad = DECIDEGREES_TO_RADIANS(attitude.values.pitch);
                const float rCoord = RADIANS_TO_DEGREES((G_ACCELERATION / airspeedMs) * sin_approx(phiRad) * cos_approx(thetaRad));
                const float yawStickAtten = 1.0f - constrainf(fabsf(getRcDeflection(FD_YAW)) * 2.0f, 0.0f, 1.0f);
                currentPidSetpoint += rCoord * yawStickAtten;
            }
#endif
        }
#endif

        bool resetDisturbance = false;

        currentPidSetpoint = wingAdjustSetpoint(currentPidSetpoint, axis);

#ifdef USE_ACRO_TRAINER
        if ((axis != FD_YAW) && pidRuntime.acroTrainerActive && !pidRuntime.inCrashRecoveryMode && !launchControlActive && !companionControlsFlight) {
            currentPidSetpoint = applyAcroTrainer(axis, angleTrim, currentPidSetpoint);
        }
#endif // USE_ACRO_TRAINER

#ifdef USE_LAUNCH_CONTROL
        if (launchControlActive && !companionControlsFlight) {
#if defined(USE_ACC)
            currentPidSetpoint = applyLaunchControl(axis, angleTrim);
#else
            currentPidSetpoint = applyLaunchControl(axis, NULL);
#endif
        }
#endif

#ifdef USE_YAW_SPIN_RECOVERY
        if (yawSpinActive) {
            if (axis == FD_YAW) {
                currentPidSetpoint = 0.0f;
                resetDisturbance = true;
            } else {
                ladrcResetAxisState(axis);
                pidData[axis].P = 0.0f;
                pidData[axis].I = 0.0f;
                pidData[axis].D = 0.0f;
                pidData[axis].F = 0.0f;
                pidData[axis].S = 0.0f;
                pidData[axis].Sum = 0.0f;
                continue;
            }
        }
#endif // USE_YAW_SPIN_RECOVERY

        const float gyroRate = gyro.gyroADCf[axis];
#ifdef USE_CHIRP
        currentPidSetpoint += currentChirp;
#endif // USE_CHIRP
        float errorRate = currentPidSetpoint - gyroRate;

#if defined(USE_ACC)
        if (!ARMING_FLAG(ARMED)) {
            if (pidRuntime.inCrashRecoveryMode) {
                pidRuntime.inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        } else {
            handleCrashRecovery(
                pidProfile->crash_recovery, angleTrim, axis, currentTimeUs, gyroRate,
                &currentPidSetpoint, &errorRate, &resetDisturbance);

            const float gyroDelta = -(gyroRate - previousGyroRateCrash[axis]) * pidRuntime.pidFrequency;
            if (cmpTimeUs(currentTimeUs, levelModeStartTimeUs) > CRASH_RECOVERY_DETECTION_DELAY_US) {
                detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, currentTimeUs, gyroDelta, errorRate);
            }
        }
#endif
        previousGyroRateCrash[axis] = gyroRate;

        pidRuntime.previousPidSetpoint[axis] = currentPidSetpoint;

        // Avoid double-filtering / 2nd-order TD phase lag inside closed-loop outer attitude controllers or crash recovery
        float tdWeight = 1.0f;
#if defined(USE_ACC)
        if (pidRuntime.inCrashRecoveryMode) {
            tdWeight = 0.0f;
        } else if (pidRuntime.axisInAngleMode[axis]) {
            if (FLIGHT_MODE(ANGLE_MODE | GPS_RESCUE_MODE | POS_HOLD_MODE | ALT_HOLD_MODE)
                || companionOffboardAttitude
                || companionLevelHold) {
                tdWeight = 0.0f;
            } else {
                tdWeight = 1.0f - constrainf(horizonLevelStrength * 4.0f, 0.0f, 1.0f);
            }
        }
#endif

        ladrcResetMode_e ladrcResetMode = LADRC_RESET_NONE;
        if (pidRuntime.zeroThrottleItermReset && !wasThrottleRaised()) {
            ladrcResetMode = LADRC_RESET_OBSERVER_FULL;
        } else if (pidRuntime.zeroThrottleItermReset || resetDisturbance) {
            ladrcResetMode = LADRC_RESET_DISTURBANCE;
        }

        const float sumLimit = (axis == FD_YAW) ? pidProfile->pidSumLimitYaw : pidProfile->pidSumLimit;
        pidData[axis].S = 0.0f;
        pidData[axis].Sum = ladrcStepAxisEx(
            axis,
            currentPidSetpoint,
            gyroRate,
            sumLimit,
            ladrcB0Scale,
            tdWeight,
            ladrcResetMode,
            &pidData[axis].P,
            &pidData[axis].D,
            &pidData[axis].F,
            &pidData[axis].I
        );
    }

#ifdef USE_WING
    bool isFixedWingAndPassthru = isFixedWing() && FLIGHT_MODE(PASSTHRU_MODE);
#endif // USE_WING
    if (!pidRuntime.pidStabilisationEnabled
        || gyroOverflowDetected()
#ifdef USE_WING
        || isFixedWingAndPassthru
#endif
        ) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;
            pidData[axis].S = 0;
            pidData[axis].Sum = 0;
        }
        ladrcResetState();
    }
}

bool crashRecoveryModeActive(void)
{
    return pidRuntime.inCrashRecoveryMode;
}

#ifdef USE_ACRO_TRAINER
void pidSetAcroTrainerState(bool newState)
{
    if (pidRuntime.acroTrainerActive != newState) {
        if (newState) {
            pidAcroTrainerInit();
        }
        pidRuntime.acroTrainerActive = newState;
    }
}

bool pidIsAcroTrainerActive(void)
{
    return pidRuntime.acroTrainerActive;
}
#endif // USE_ACRO_TRAINER

void pidSetAntiGravityState(bool newState)
{
    if (newState != pidRuntime.antiGravityEnabled) {
        // reset the accelerator on state changes
        pidRuntime.itermAccelerator = 0.0f;
    }
    pidRuntime.antiGravityEnabled = newState;
}

bool pidAntiGravityEnabled(void)
{
    return pidRuntime.antiGravityEnabled;
}

#ifdef USE_DYN_LPF
void dynLpfDTermUpdate(float throttle)
{
    if (pidRuntime.dynLpfFilter != DYN_LPF_NONE) {
        float cutoffFreq;
        if (pidRuntime.dynLpfCurveExpo > 0) {
            cutoffFreq = dynLpfCutoffFreq(throttle, pidRuntime.dynLpfMin, pidRuntime.dynLpfMax, pidRuntime.dynLpfCurveExpo);
        } else {
            cutoffFreq = fmaxf(dynThrottle(throttle) * pidRuntime.dynLpfMax, pidRuntime.dynLpfMin);
        }

        switch (pidRuntime.dynLpfFilter) {
        case DYN_LPF_PT1:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        case DYN_LPF_SVF:
            cutoffFreq = MIN(cutoffFreq, 0.475f / pidRuntime.dT); // constrain to be below the nyquist
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                svfLowpassFilterUpdate(&pidRuntime.dtermLowpass[axis].svfLowpassFilter, cutoffFreq, pidRuntime.dT);
            }
            break;
        case DYN_LPF_PT2:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt2FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt2Filter, pt2FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        case DYN_LPF_PT3:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt3FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt3Filter, pt3FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        }
    }
}
#endif

float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo)
{
    const float expof = expo / 10.0f;
    const float curve = throttle * (1 - throttle) * expof + throttle;
    return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

void pidSetItermReset(bool enabled)
{
    pidRuntime.zeroThrottleItermReset = enabled;
}

float pidGetPreviousSetpoint(int axis)
{
    return pidRuntime.previousPidSetpoint[axis];
}

float pidGetDT(void)
{
    return pidRuntime.dT;
}

float pidGetPidFrequency(void)
{
    return pidRuntime.pidFrequency;
}

#ifdef USE_CHIRP
bool  pidChirpIsFinished(void)
{
    return pidRuntime.chirp.isFinished;
}
#endif
