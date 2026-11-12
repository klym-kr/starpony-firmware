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
    #include "common/time.h"
    #include "fc/runtime_config.h"
    #include "io/companion_link.h"
    #include "pg/pg.h"

    void beeperConfirmationBeeps(uint8_t) { }
}

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode = 0;

class CompanionLinkTest : public ::testing::Test {
protected:
    void SetUp() override {
        pgResetAll();
        DISABLE_ARMING_FLAG(ARMED);
        companionLinkInit();
        debugMode = 0;
    }
};

TEST_F(CompanionLinkTest, testDisabledLinkRejectsCommands) {
    const float target[4] = { 10.0f, -5.0f, 0.0f, 0.4f };
    EXPECT_FALSE(companionLinkAcceptOffboardCmd(1, 150, COMPANION_MODE_RATE_THRUST, target, 1000));
    EXPECT_FALSE(companionLinkIsOffboardActive());
}

TEST_F(CompanionLinkTest, testAcceptsAndClampsThrottleWithTtlWatchdog) {
    companionConfigMutable()->enabled = 1;
    companionConfigMutable()->default_ttl_ms = 100;

    const float target[4] = { 45.0f, -20.0f, 15.0f, 1.5f }; // throttle out of [0..1]
    EXPECT_TRUE(companionLinkAcceptOffboardCmd(42, 100, COMPANION_MODE_RATE_THRUST, target, 10000));

    // Must NOT report offboard active while disarmed!
    EXPECT_FALSE(companionLinkIsOffboardActive());

    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_TRUE(companionLinkIsOffboardActive());

    const companionSetpoint_t *sp = companionLinkGetSetpoint();
    EXPECT_EQ(sp->seq, 42);
    EXPECT_FLOAT_EQ(sp->target[3], 1.0f);

    // Within TTL (100 ms = 100000 us): still active
    companionLinkUpdate(10000 + 80000);
    EXPECT_TRUE(companionLinkIsOffboardActive());

    // After TTL expires: watchdog disables offboard and increments timeoutCount
    companionLinkUpdate(10000 + 120000);
    EXPECT_FALSE(companionLinkIsOffboardActive());
    EXPECT_FALSE(companionLinkGetStatus()->linkAlive);
    EXPECT_EQ(companionLinkGetStatus()->timeoutCount, 1U);
}

TEST_F(CompanionLinkTest, testRejectsNonFiniteTargetsAndClampsAttitudeBounds) {
    companionConfigMutable()->enabled = 1;

    const float nanTarget[4] = { NAN, 0.0f, 0.0f, 0.3f };
    EXPECT_FALSE(companionLinkAcceptOffboardCmd(1, 100, COMPANION_MODE_RATE_THRUST, nanTarget, 1000));

    const float infTarget[4] = { 0.0f, 0.0f, 0.0f, INFINITY };
    EXPECT_FALSE(companionLinkAcceptOffboardCmd(2, 100, COMPANION_MODE_RATE_THRUST, infTarget, 2000));

    const float bigAttTarget[4] = { 130.0f, -120.0f, 3500.0f, 0.5f };
    EXPECT_TRUE(companionLinkAcceptOffboardCmd(3, 100, COMPANION_MODE_ATTITUDE_THRUST, bigAttTarget, 3000));
    const companionSetpoint_t *sp = companionLinkGetSetpoint();
    EXPECT_FLOAT_EQ(sp->target[0], 85.0f);
    EXPECT_FLOAT_EQ(sp->target[1], -85.0f);
    EXPECT_FLOAT_EQ(sp->target[2], 2000.0f);
    EXPECT_FLOAT_EQ(sp->target[3], 0.5f);
}

TEST_F(CompanionLinkTest, testFailsafePoliciesAndPilotOverrideLatching) {
    companionConfigMutable()->enabled = 1;
    companionConfigMutable()->failsafe_policy = COMPANION_FAILSAFE_LEVEL_HOLD;
    ENABLE_ARMING_FLAG(ARMED);

    const float target[4] = { 10.0f, -5.0f, 0.0f, 0.42f };
    EXPECT_TRUE(companionLinkAcceptOffboardCmd(10, 50, COMPANION_MODE_ATTITUDE_THRUST, target, 1000));
    companionLinkSetPilotOverride(false, false);
    EXPECT_TRUE(companionLinkIsOffboardActive());

    // Timeout triggers LEVEL_HOLD with last throttle
    companionLinkUpdate(1000 + 60000);
    EXPECT_FALSE(companionLinkIsOffboardActive());
    EXPECT_TRUE(companionLinkIsLevelHoldActive());
    EXPECT_FLOAT_EQ(companionLinkGetFailsafeThrottle(), 0.42f);

    // Pilot stick move immediately exits LEVEL_HOLD and latches override against rogue resumed packets
    companionLinkSetPilotOverride(false, true);
    EXPECT_FALSE(companionLinkIsLevelHoldActive());
    companionLinkSetPilotOverride(false, false);
    EXPECT_TRUE(companionLinkAcceptOffboardCmd(11, 50, COMPANION_MODE_ATTITUDE_THRUST, target, 70000));
    EXPECT_FALSE(companionLinkIsOffboardActive());

    // Disarming or MODE_NONE releases stick override latch; RTH policy activates on timeout
    DISABLE_ARMING_FLAG(ARMED);
    companionLinkUpdate(71000);
    ENABLE_ARMING_FLAG(ARMED);
    companionConfigMutable()->failsafe_policy = COMPANION_FAILSAFE_RTH;
    companionLinkSetPilotOverride(false, false);
    EXPECT_TRUE(companionLinkAcceptOffboardCmd(12, 50, COMPANION_MODE_RATE_THRUST, target, 80000));
    EXPECT_TRUE(companionLinkIsOffboardActive());
    companionLinkUpdate(80000 + 60000);
    EXPECT_TRUE(companionLinkShouldTriggerRth());
}
