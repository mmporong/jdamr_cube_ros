#include <gtest/gtest.h>

#include "jdamr_base_driver/estop_latch.hpp"

using jdamr::commands_motion;
using jdamr::EmergencyStopLatch;

TEST(EmergencyStopLatch, StartsReleasedAndEngagesOnce)
{
  EmergencyStopLatch latch;
  EXPECT_FALSE(latch.engaged());
  EXPECT_TRUE(latch.engage());
  EXPECT_FALSE(latch.engage());   // already engaged: no second transition
  EXPECT_TRUE(latch.engaged());
}

TEST(EmergencyStopLatch, ResetRefusedWhileMotionIsCommanded)
{
  EmergencyStopLatch latch;
  latch.engage();
  EXPECT_FALSE(latch.reset(true));
  EXPECT_TRUE(latch.engaged());
  EXPECT_TRUE(latch.reset(false));
  EXPECT_FALSE(latch.engaged());
}

TEST(EmergencyStopLatch, OnlyAFreshNonZeroCommandIsMotion)
{
  EXPECT_FALSE(commands_motion(false, 0.12, 0.0));   // stale command
  EXPECT_FALSE(commands_motion(true, 0.0, 0.0));
  EXPECT_FALSE(commands_motion(true, 5e-4, -5e-4));  // below the stop band
  EXPECT_TRUE(commands_motion(true, -0.08, 0.0));
  EXPECT_TRUE(commands_motion(true, 0.0, 0.3));
}
