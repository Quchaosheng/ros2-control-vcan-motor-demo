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

  EXPECT_EQ(health.pending_ack_count(0), 1U);
  EXPECT_EQ(health.last_ack_status(0), AckStatus::NONE);
  EXPECT_EQ(health.ack_received(0, 7, 0), AckStatus::ACCEPTED);
  EXPECT_EQ(health.pending_ack_count(0), 0U);
  EXPECT_EQ(health.last_ack_status(0), AckStatus::ACCEPTED);
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
  EXPECT_EQ(health.last_ack_status(0), AckStatus::REJECTED);
  EXPECT_TRUE(health.ack_fault());

  health.reset(now);
  health.command_sent(0, 9, now);
  EXPECT_EQ(health.ack_received(0, 10, 0), AckStatus::UNEXPECTED);
  EXPECT_EQ(health.last_ack_status(0), AckStatus::UNEXPECTED);
  EXPECT_TRUE(health.ack_fault());
}

TEST(HardwareHealth, ExposesFeedbackAgeAndBoundsSafeDiagnosticState)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);

  EXPECT_EQ(health.feedback_age(0, now + 75ms), 75ms);
  EXPECT_EQ(health.pending_ack_count(2), 0U);
  EXPECT_EQ(health.last_ack_status(2), AckStatus::UNEXPECTED);
  EXPECT_EQ(health.feedback_age(2, now + 75ms), 0ms);
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

TEST(HardwareHealth, BoundsOutstandingCommandWindow)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);

  for (std::size_t offset = 0; offset < HardwareHealth::kMaxOutstandingCommands; ++offset) {
    const auto sequence = static_cast<uint8_t>(offset + 1U);
    EXPECT_TRUE(health.can_accept_command(0, sequence)) << "offset " << offset;
    health.command_sent(0, sequence, now);
  }

  EXPECT_EQ(health.pending_ack_count(0), HardwareHealth::kMaxOutstandingCommands);
  // The window is full: the next sequence must be refused before it is sent.
  EXPECT_FALSE(health.can_accept_command(0, 5));
  EXPECT_FALSE(health.can_accept_command(0, 6));

  // Draining one slot frees exactly one slot.
  EXPECT_EQ(health.ack_received(0, 1, 0), AckStatus::ACCEPTED);
  EXPECT_EQ(health.pending_ack_count(0), HardwareHealth::kMaxOutstandingCommands - 1U);
  EXPECT_TRUE(health.can_accept_command(0, 5));
}

TEST(HardwareHealth, RejectsCommandSentBeyondCapacityWithoutGrowing)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);
  for (std::size_t offset = 0; offset < HardwareHealth::kMaxOutstandingCommands + 3U; ++offset) {
    health.command_sent(0, static_cast<uint8_t>(offset + 1U), now);
  }

  // Storage is preallocated, so overflow drops instead of allocating.
  EXPECT_EQ(health.pending_ack_count(0), HardwareHealth::kMaxOutstandingCommands);
}

TEST(HardwareHealth, FullWindowStillReportsAckTimeoutAsTheFault)
{
  // Regression: a full outstanding window is backpressure, not a transport
  // failure. If it latched a send error instead, it would mask the ACK-timeout
  // fault that read() detects when the device stops acknowledging.
  HardwareHealth health;
  const auto t0 = HardwareHealth::Clock::now();
  health.reset(t0);
  const auto ack_timeout = std::chrono::milliseconds(250);
  const auto period = std::chrono::milliseconds(20);

  std::size_t accepted = 0U;
  bool timed_out = false;
  for (std::size_t cycle = 0; cycle < 100U; ++cycle) {
    const auto now = t0 + period * cycle;
    const uint8_t sequence = static_cast<uint8_t>(accepted + 1U);
    if (health.can_accept_command(0, sequence)) {
      health.command_sent(0, sequence, now);
      ++accepted;
    }
    if (health.ack_timed_out(0, now, ack_timeout)) {
      timed_out = true;
      break;
    }
  }

  EXPECT_TRUE(timed_out);
  EXPECT_EQ(accepted, HardwareHealth::kMaxOutstandingCommands);
  EXPECT_EQ(health.pending_ack_count(0), HardwareHealth::kMaxOutstandingCommands);
}

TEST(HardwareHealth, RejectsInFlightSequenceReuseBeforeSending)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);
  health.command_sent(0, 7, now);

  // 7 is still unacknowledged: reusing it would make the ACK ambiguous.
  EXPECT_FALSE(health.can_accept_command(0, 7));
  // A sequence that is not in flight is fine.
  EXPECT_TRUE(health.can_accept_command(0, 8));
  // The safe-stop shadow slot is not part of the outstanding window.
  EXPECT_TRUE(health.can_accept_command(1, 7));
}

TEST(HardwareHealth, KeepsWindowConsistentAcrossSequenceWrap)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);

  health.command_sent(0, 254, now);
  health.command_sent(0, 255, now);
  health.command_sent(0, 0, now);
  health.command_sent(0, 1, now);
  EXPECT_FALSE(health.can_accept_command(0, 2));

  // Acknowledgments keep arriving in order across the 8-bit wrap.
  EXPECT_EQ(health.ack_received(0, 254, 0), AckStatus::ACCEPTED);
  EXPECT_TRUE(health.can_accept_command(0, 2));
  health.command_sent(0, 2, now);
  EXPECT_EQ(health.ack_received(0, 255, 0), AckStatus::ACCEPTED);
  EXPECT_EQ(health.ack_received(0, 0, 0), AckStatus::ACCEPTED);
  EXPECT_EQ(health.ack_received(0, 1, 0), AckStatus::ACCEPTED);
  EXPECT_EQ(health.ack_received(0, 2, 0), AckStatus::ACCEPTED);
  EXPECT_EQ(health.pending_ack_count(0), 0U);
  EXPECT_FALSE(health.ack_fault());
}

TEST(HardwareHealth, TreatsDuplicateAndLateAcksAsUnexpected)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);
  health.command_sent(0, 20, now);
  health.command_sent(0, 21, now);

  EXPECT_EQ(health.ack_received(0, 20, 0), AckStatus::ACCEPTED);
  // A second ACK for an already-retired sequence is a protocol fault.
  EXPECT_EQ(health.ack_received(0, 20, 0), AckStatus::UNEXPECTED);
  EXPECT_TRUE(health.ack_fault());

  // The window still holds only the genuinely outstanding command.
  EXPECT_EQ(health.pending_ack_count(0), 1U);
  EXPECT_EQ(health.ack_received(0, 21, 0), AckStatus::ACCEPTED);
}

TEST(HardwareHealth, LateDuplicateOfTheOldestOutstandingCommandIsUnexpected)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);
  health.command_sent(0, 30, now);
  health.command_sent(0, 31, now);

  // 31 arrives first (reordering), so 30 is no longer the front of the window.
  EXPECT_EQ(health.ack_received(0, 31, 0), AckStatus::UNEXPECTED);
  EXPECT_TRUE(health.ack_fault());
  EXPECT_EQ(health.pending_ack_count(0), 2U);
}

TEST(HardwareHealth, ResetReactivatesAnEmptyWindow)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);
  for (std::size_t offset = 0; offset < HardwareHealth::kMaxOutstandingCommands; ++offset) {
    health.command_sent(0, static_cast<uint8_t>(offset + 1U), now);
  }
  EXPECT_FALSE(health.can_accept_command(0, 9));
  EXPECT_EQ(health.ack_received(0, 4, 2), AckStatus::UNEXPECTED);
  EXPECT_TRUE(health.ack_fault());

  // Reactivation must clear the window, the fault and the safe-stop shadow.
  health.reset(now);
  EXPECT_EQ(health.pending_ack_count(0), 0U);
  EXPECT_FALSE(health.ack_fault());
  EXPECT_TRUE(health.can_accept_command(0, 9));
  EXPECT_EQ(health.ack_received(0, 9, 0), AckStatus::UNEXPECTED);
}

TEST(HardwareHealth, KeepsSafeStopAckTrackingBoundedAfterReactivation)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);

  // A safe stop only records its shadow sequence and never enters the window.
  health.safe_stop_sent(0, 42);
  EXPECT_EQ(health.pending_ack_count(0), 0U);
  EXPECT_EQ(health.ack_received(0, 42, 0), AckStatus::IGNORED);
  EXPECT_FALSE(health.ack_fault());

  health.reset(now);
  health.safe_stop_sent(0, 43);
  EXPECT_EQ(health.ack_received(0, 43, 0), AckStatus::IGNORED);
  EXPECT_FALSE(health.ack_fault());
}
