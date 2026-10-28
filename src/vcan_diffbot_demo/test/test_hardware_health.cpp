#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <new>

#include "vcan_diffbot_demo/hardware_health.hpp"

using namespace std::chrono_literals;
using vcan_diffbot_demo::AckStatus;
using vcan_diffbot_demo::HardwareHealth;

namespace
{
// Counts heap traffic only inside the scope that turns the flag on, so gtest's
// own allocations are not attributed to the code under test.
std::atomic<bool> g_count_allocations{false};
std::atomic<std::size_t> g_allocations{0};
}  // namespace

void * operator new(std::size_t size)
{
  if (g_count_allocations.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1U, std::memory_order_relaxed);
  }
  void * memory = std::malloc(size);
  if (memory == nullptr) {
    throw std::bad_alloc();
  }
  return memory;
}

void operator delete(void * memory) noexcept
{
  std::free(memory);
}

void operator delete(void * memory, std::size_t) noexcept
{
  std::free(memory);
}

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

TEST(HardwareHealth, RefusesToTrackPastTheWindow)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);

  for (std::size_t i = 0; i < HardwareHealth::kMaxOutstandingAcks; ++i) {
    ASSERT_TRUE(health.can_track(0, static_cast<uint8_t>(i)));
    ASSERT_TRUE(health.command_sent(0, static_cast<uint8_t>(i), now));
  }
  EXPECT_EQ(health.pending_ack_count(0), HardwareHealth::kMaxOutstandingAcks);

  // The window is full: a new sequence must not be accepted, and the count must
  // not grow even if a caller ignores can_track().
  EXPECT_FALSE(health.can_track(0, 99U));
  EXPECT_FALSE(health.command_sent(0, 99U, now));
  EXPECT_EQ(health.pending_ack_count(0), HardwareHealth::kMaxOutstandingAcks);
}

TEST(HardwareHealth, RefusesAnAmbiguousSequenceAlreadyInFlight)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);

  ASSERT_TRUE(health.command_sent(0, 7U, now));

  // Same sequence while it is still outstanding: tracking a second copy would
  // make the next ACK unattributable.
  EXPECT_FALSE(health.can_track(0, 7U));
  EXPECT_FALSE(health.command_sent(0, 7U, now));
  EXPECT_EQ(health.pending_ack_count(0), 1U);
}

TEST(HardwareHealth, SequenceReuseIsFineOnceDrained)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);

  ASSERT_TRUE(health.command_sent(0, 7U, now));
  EXPECT_EQ(health.ack_received(0, 7U, 0U), AckStatus::ACCEPTED);

  EXPECT_TRUE(health.can_track(0, 7U));
  EXPECT_TRUE(health.command_sent(0, 7U, now));
}

TEST(HardwareHealth, WrapsAroundTheSequenceSpace)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);

  ASSERT_TRUE(health.command_sent(0, 255U, now));
  EXPECT_EQ(health.ack_received(0, 255U, 0U), AckStatus::ACCEPTED);
  ASSERT_TRUE(health.command_sent(0, 0U, now));
  EXPECT_EQ(health.ack_received(0, 0U, 0U), AckStatus::ACCEPTED);
  EXPECT_EQ(health.pending_ack_count(0), 0U);
}

TEST(HardwareHealth, ReactivationClearsTheWindow)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);

  for (std::size_t i = 0; i < HardwareHealth::kMaxOutstandingAcks; ++i) {
    ASSERT_TRUE(health.command_sent(0, static_cast<uint8_t>(i), now));
  }
  EXPECT_FALSE(health.can_track(0, 200U));

  health.reset(now);
  EXPECT_EQ(health.pending_ack_count(0), 0U);
  EXPECT_TRUE(health.can_track(0, 200U));
}

TEST(HardwareHealth, LateAckAfterTheWindowDrainedIsUnexpected)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);

  ASSERT_TRUE(health.command_sent(0, 3U, now));
  EXPECT_EQ(health.ack_received(0, 3U, 0U), AckStatus::ACCEPTED);
  // A duplicate of an ACK that was already taken has nothing to match.
  EXPECT_EQ(health.ack_received(0, 3U, 0U), AckStatus::UNEXPECTED);
}

TEST(HardwareHealth, CommandAndAckDoNotAllocate)
{
  HardwareHealth health;
  const auto now = HardwareHealth::Clock::now();
  health.reset(now);

  g_allocations.store(0U);
  g_count_allocations.store(true);

  // Steady state: one command outstanding at a time.
  for (std::size_t i = 0; i < 1000U; ++i) {
    const auto sequence = static_cast<uint8_t>(i);
    ASSERT_TRUE(health.can_track(0, sequence));
    ASSERT_TRUE(health.command_sent(0, sequence, now));
    ASSERT_EQ(health.ack_received(0, sequence, 0U), AckStatus::ACCEPTED);
  }

  // Burst: the window filled to its bound and drained again.
  for (std::size_t round = 0; round < 10U; ++round) {
    for (std::size_t i = 0; i < HardwareHealth::kMaxOutstandingAcks; ++i) {
      ASSERT_TRUE(health.command_sent(0, static_cast<uint8_t>(i), now));
    }
    for (std::size_t i = 0; i < HardwareHealth::kMaxOutstandingAcks; ++i) {
      ASSERT_EQ(health.ack_received(0, static_cast<uint8_t>(i), 0U), AckStatus::ACCEPTED);
    }
  }

  g_count_allocations.store(false);

  EXPECT_EQ(g_allocations.load(), 0U)
    << "the write path allocated; the window is supposed to be preallocated";
}
