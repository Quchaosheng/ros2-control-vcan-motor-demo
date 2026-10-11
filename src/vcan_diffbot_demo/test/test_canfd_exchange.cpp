#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>

#include "vcan_diffbot_demo/can_transport.hpp"
#include "vcan_diffbot_demo/canfd_protocol.hpp"
#include "vcan_diffbot_demo/hardware_health.hpp"

using namespace std::chrono_literals;
using namespace vcan_diffbot_demo;
using namespace vcan_diffbot_demo::protocol;

namespace
{

/// The smallest closed loop the FD profile needs: encode a command, carry it
/// through the wire layout, and feed the resulting request identity to the ACK
/// tracker. This is the part of issue #11 that is testable without a CAN
/// interface, so it stays free of any hardware plugin dependency.
class FdExchange
{
public:
  explicit FdExchange(const uint8_t node_id, const uint32_t first_command_id)
  : node_id_(node_id), next_command_id_(first_command_id)
  {
    health_.reset(HardwareHealth::Clock::now());
  }

  FdCommand command(const int16_t velocity, const bool enabled = true)
  {
    return FdCommand{{node_id_, ++next_command_id_}, enabled, velocity, 200U};
  }

  /// Encode and track one command, mirroring what the hardware write path does.
  const FdFrameData & send(const FdCommand & command, const bool track_ack = true)
  {
    frames_ = encode_fd_command(command);
    const auto sequence = static_cast<uint8_t>(command.request.command_id);
    if (track_ack) {
      health_.command_sent(0U, sequence, HardwareHealth::Clock::now());
    } else {
      health_.safe_stop_sent(0U, sequence);
    }
    return frames_;
  }

  FdAck acknowledge(const uint8_t result)
  {
    return FdAck{{node_id_, next_command_id_}, result};
  }

  AckStatus deliver_ack(const FdAck & ack)
  {
    return health_.ack_received(
      0U, static_cast<uint8_t>(ack.request.command_id), ack.result);
  }

  FdFeedback feedback(const int16_t velocity, const int32_t encoder)
  {
    return FdFeedback{{node_id_, next_command_id_}, 0x0001U, velocity, encoder, 12345U};
  }

  HardwareHealth & health() {return health_;}

private:
  uint8_t node_id_;
  uint32_t next_command_id_;
  FdFrameData frames_{};
  HardwareHealth health_;
};

}  // namespace

TEST(CanFdExchange, CompletesACommandAckFeedbackCycle)
{
  FdExchange link(7U, 0x1000U);

  const auto command = link.command(1500);
  const auto decoded_command = decode_fd_command(link.send(command));
  ASSERT_TRUE(decoded_command.has_value());
  EXPECT_EQ(decoded_command->request.command_id, command.request.command_id);
  EXPECT_EQ(decoded_command->request.node_id, 7U);
  EXPECT_TRUE(decoded_command->enabled);
  EXPECT_EQ(decoded_command->velocity_mrad_s, 1500);
  EXPECT_EQ(link.health().pending_ack_count(0U), 1U);

  EXPECT_EQ(link.deliver_ack(link.acknowledge(kFdResultAccepted)), AckStatus::ACCEPTED);
  EXPECT_EQ(link.health().pending_ack_count(0U), 0U);
  EXPECT_FALSE(link.health().ack_fault());

  const auto decoded_feedback = decode_fd_feedback(encode_fd_feedback(link.feedback(1500, 4096)));
  ASSERT_TRUE(decoded_feedback.has_value());
  EXPECT_EQ(decoded_feedback->encoder_count, 4096);
  EXPECT_EQ(decoded_feedback->velocity_mrad_s, 1500);
  EXPECT_EQ(decoded_feedback->request.command_id, command.request.command_id);
}

TEST(CanFdExchange, DistinguishesCommandsThatShareALowByte)
{
  FdExchange link(7U, 0x1000U);

  // Two live commands whose classic 8-bit sequence is identical must remain
  // separately identifiable through the widened request identity.
  const auto first = link.command(1000);
  const auto second_bytes = FdCommand{{7U, first.request.command_id + 256U}, true, 1000, 200U};
  const auto first_bytes = link.send(first);
  const auto second_frame = encode_fd_command(second_bytes);

  EXPECT_EQ(first_bytes[0], second_frame[0]);
  EXPECT_NE(
    decode_fd_command(first_bytes)->request.command_id,
    decode_fd_command(second_frame)->request.command_id);

  EXPECT_EQ(link.deliver_ack(link.acknowledge(kFdResultAccepted)), AckStatus::ACCEPTED);
}

TEST(CanFdExchange, ReportsARejectedFdAckAsARejection)
{
  FdExchange link(7U, 0x2000U);

  link.send(link.command(1000));
  EXPECT_EQ(link.deliver_ack(link.acknowledge(kFdResultRejected)), AckStatus::REJECTED);
  EXPECT_TRUE(link.health().ack_fault());
}

TEST(CanFdExchange, TreatsAnAckForAnUnknownIdentityAsUnexpected)
{
  FdExchange link(7U, 0x3000U);

  link.send(link.command(1000));
  // An ACK whose widened id does not match the in-flight command.
  EXPECT_EQ(link.deliver_ack({{7U, 0x9999U}, kFdResultAccepted}), AckStatus::UNEXPECTED);
  EXPECT_TRUE(link.health().ack_fault());
}

TEST(CanFdExchange, TimesOutAnUnansweredCommand)
{
  FdExchange link(7U, 0x4000U);
  link.send(link.command(1000));

  const auto now = HardwareHealth::Clock::now();
  EXPECT_FALSE(link.health().ack_timed_out(0U, now, 200ms));
  EXPECT_TRUE(link.health().ack_timed_out(0U, now + 201ms, 200ms));
  EXPECT_TRUE(link.health().ack_timed_out(now + 201ms, 200ms));
}

TEST(CanFdExchange, RejectsMalformedAndMismatchedFrames)
{
  FdExchange link(7U, 0x5000U);
  const auto frame = link.send(link.command(1000));

  // A classic-length frame is not a valid FD command.
  EXPECT_FALSE(decode_fd_command(frame, 8U).has_value());
  // A dirty reserved tail is refused.
  auto dirty = frame;
  dirty[kFdCommandDlc - 1U] = 1U;
  EXPECT_FALSE(decode_fd_command(dirty).has_value());
  // A wrong version tag is refused.
  auto wrong_version = frame;
  wrong_version[1] = kFdProfileVersion + 1U;
  EXPECT_FALSE(decode_fd_command(wrong_version).has_value());

  // A frame arriving on an endpoint configured for the other profile is a
  // mismatch, not something to guess at.
  EXPECT_EQ(match_frame(TransportProfile::ClassicCan, true), FrameMatch::ProfileMismatch);
  EXPECT_EQ(match_frame(TransportProfile::CanFd, true), FrameMatch::Accepted);
  EXPECT_EQ(match_frame(TransportProfile::CanFd, false), FrameMatch::ProfileMismatch);
}

TEST(CanFdExchange, KeepsTheSafeStopToASingleIgnoredCommand)
{
  FdExchange link(7U, 0x6000U);

  // The FD safe stop mirrors the classic path: one disabled zero command per
  // motor, tracked outside the outstanding window, acknowledged exactly once.
  const auto stop = link.command(0, false);
  const auto decoded = decode_fd_command(link.send(stop, false));
  ASSERT_TRUE(decoded.has_value());
  EXPECT_FALSE(decoded->enabled);
  EXPECT_EQ(decoded->velocity_mrad_s, 0);
  EXPECT_EQ(link.health().pending_ack_count(0U), 0U);

  EXPECT_EQ(link.deliver_ack(link.acknowledge(kFdResultAccepted)), AckStatus::IGNORED);
  EXPECT_FALSE(link.health().ack_fault());
  // A second ACK for the same safe stop is no longer expected.
  EXPECT_EQ(link.deliver_ack(link.acknowledge(kFdResultAccepted)), AckStatus::UNEXPECTED);
}

TEST(CanFdExchange, ReactivationClearsTheTracker)
{
  FdExchange link(7U, 0x7000U);

  link.send(link.command(1000));
  link.deliver_ack(link.acknowledge(kFdResultRejected));
  EXPECT_TRUE(link.health().ack_fault());

  link.health().reset(HardwareHealth::Clock::now());
  EXPECT_FALSE(link.health().ack_fault());
  EXPECT_EQ(link.health().pending_ack_count(0U), 0U);
  EXPECT_EQ(link.deliver_ack({{7U, 0x7001U}, kFdResultAccepted}), AckStatus::UNEXPECTED);
}
