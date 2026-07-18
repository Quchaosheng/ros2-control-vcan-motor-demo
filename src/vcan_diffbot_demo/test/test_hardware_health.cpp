#include <gtest/gtest.h>

#include <chrono>

#include "vcan_diffbot_demo/hardware_health.hpp"

using namespace std::chrono_literals;
using vcan_diffbot_demo::AckStatus;
using vcan_diffbot_demo::HardwareHealth;

TEST(HardwareHealth, AcceptsMatchingAck)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);
  health.command_sent(0, 7, now);

  EXPECT_EQ(health.ack_received(0, 7, 0), AckStatus::ACCEPTED);
  EXPECT_FALSE(health.ack_fault());
  EXPECT_FALSE(health.ack_timed_out(now + 250ms, 200ms));
}

TEST(HardwareHealth, RejectsFailedOrUnexpectedAck)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);
  health.command_sent(0, 8, now);

  EXPECT_EQ(health.ack_received(0, 8, 2), AckStatus::REJECTED);
  EXPECT_TRUE(health.ack_fault());

  health.reset(now);
  health.command_sent(0, 9, now);
  EXPECT_EQ(health.ack_received(0, 10, 0), AckStatus::UNEXPECTED);
  EXPECT_TRUE(health.ack_fault());
}

TEST(HardwareHealth, DetectsMissingAck)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);
  health.command_sent(0, 9, now);

  EXPECT_FALSE(health.ack_timed_out(now + 200ms, 200ms));
  EXPECT_TRUE(health.ack_timed_out(now + 201ms, 200ms));
}

TEST(HardwareHealth, IgnoresSafeStopAck)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);
  health.safe_stop_sent(0, 11);

  EXPECT_EQ(health.ack_received(0, 11, 0), AckStatus::IGNORED);
  EXPECT_FALSE(health.ack_fault());
}

TEST(HardwareHealth, RecoversStopEpisodeOnlyWhenBothFeedbacksAreFresh)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);
  health.mark_stop_sent();

  health.feedback_received(0, now + 550ms);
  health.recover_if_healthy(now + 550ms, 500ms, 200ms);
  EXPECT_TRUE(health.stop_sent());

  health.feedback_received(1, now + 550ms);
  health.recover_if_healthy(now + 550ms, 500ms, 200ms);
  EXPECT_FALSE(health.stop_sent());
}

TEST(HardwareHealth, DoesNotRecoverWhileAckIsTimedOut)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);
  health.command_sent(0, 12, now);
  health.mark_stop_sent();
  health.feedback_received(0, now + 250ms);
  health.feedback_received(1, now + 250ms);

  health.recover_if_healthy(now + 250ms, 500ms, 200ms);
  EXPECT_TRUE(health.stop_sent());
}
