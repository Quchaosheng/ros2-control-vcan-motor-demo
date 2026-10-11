#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "vcan_diffbot_demo/can_protocol.hpp"
#include "vcan_diffbot_demo/canfd_protocol.hpp"

using namespace vcan_diffbot_demo::protocol;

namespace
{

constexpr FdRequestId kRequest{7U, 0x11223344U};

}  // namespace

TEST(CanFdProtocol, UsesAFixedVersionTag)
{
  EXPECT_EQ(kFdProfileVersion, 1U);

  const auto command = encode_fd_command({kRequest, true, 1000, 200});
  const auto feedback = encode_fd_feedback({kRequest, 3U, 1000, 10, 20U});
  const auto ack = encode_fd_ack({kRequest, kFdResultAccepted});

  EXPECT_EQ(command[1], kFdProfileVersion);
  EXPECT_EQ(feedback[1], kFdProfileVersion);
  EXPECT_EQ(ack[1], kFdProfileVersion);
}

TEST(CanFdProtocol, UsesDistinctFrameTypeTags)
{
  EXPECT_EQ(encode_fd_command({kRequest, true, 1000, 200})[13], 0x01U);
  EXPECT_EQ(encode_fd_feedback({kRequest, 3U, 1000, 10, 20U})[13], 0x02U);
  EXPECT_EQ(encode_fd_ack({kRequest, kFdResultAccepted})[13], 0x03U);
}

TEST(CanFdProtocol, EncodesAndDecodesCommand)
{
  const FdCommand input{kRequest, true, -1234, 200};
  const auto bytes = encode_fd_command(input);

  // The low byte mirrors classic byte 0 and the widened id spans bytes 4-11.
  EXPECT_EQ(bytes[0], 0x44U);
  EXPECT_EQ(bytes[2], 0x01U);
  EXPECT_EQ(bytes[4], 0x44U);
  EXPECT_EQ(bytes[5], 0x33U);
  EXPECT_EQ(bytes[6], 0x22U);
  EXPECT_EQ(bytes[7], 0x11U);
  EXPECT_EQ(bytes[8], 0x07U);

  const auto command = decode_fd_command(bytes);
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->request.node_id, input.request.node_id);
  EXPECT_EQ(command->request.command_id, input.request.command_id);
  EXPECT_TRUE(command->enabled);
  EXPECT_EQ(command->velocity_mrad_s, input.velocity_mrad_s);
  EXPECT_EQ(command->watchdog_ms, input.watchdog_ms);
}

TEST(CanFdProtocol, EncodesAndDecodesFeedback)
{
  const FdFeedback input{kRequest, 0x0003U, -1234, -123456, 999999U};
  const auto bytes = encode_fd_feedback(input);

  const auto feedback = decode_fd_feedback(bytes);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_EQ(feedback->request.node_id, input.request.node_id);
  EXPECT_EQ(feedback->request.command_id, input.request.command_id);
  EXPECT_EQ(feedback->status, input.status);
  EXPECT_EQ(feedback->velocity_mrad_s, input.velocity_mrad_s);
  EXPECT_EQ(feedback->encoder_count, input.encoder_count);
  EXPECT_EQ(feedback->timestamp_us, input.timestamp_us);
}

TEST(CanFdProtocol, EncodesAndDecodesAck)
{
  const FdAck input{kRequest, kFdResultRejected};
  const auto bytes = encode_fd_ack(input);

  const auto ack = decode_fd_ack(bytes);
  ASSERT_TRUE(ack.has_value());
  EXPECT_EQ(ack->request.command_id, input.request.command_id);
  EXPECT_EQ(ack->result, kFdResultRejected);
}

TEST(CanFdProtocol, WidenedRequestIdentitySurvivesSequenceWrap)
{
  // Two commands whose low byte is identical must stay distinguishable, which
  // is the whole point of the wider request identity.
  const auto first = encode_fd_ack({{7U, 300U}, kFdResultAccepted});
  const auto second = encode_fd_ack({{7U, 556U}, kFdResultAccepted});

  EXPECT_EQ(first[0], second[0]);
  EXPECT_NE(decode_fd_ack(first)->request.command_id, decode_fd_ack(second)->request.command_id);
}

TEST(CanFdProtocol, RejectsTruncatedAndOversizedFrames)
{
  const auto command = encode_fd_command({kRequest, true, 1000, 200});
  const auto feedback = encode_fd_feedback({kRequest, 3U, 1000, 10, 20U});
  const auto ack = encode_fd_ack({kRequest, kFdResultAccepted});

  // A classic DLC 8 frame is not a valid CAN-FD payload.
  EXPECT_FALSE(decode_fd_command(command, 8U).has_value());
  EXPECT_FALSE(decode_fd_feedback(feedback, 8U).has_value());
  EXPECT_FALSE(decode_fd_ack(ack, 8U).has_value());

  // A length the FD data phase cannot carry.
  EXPECT_FALSE(decode_fd_command(command, 10U).has_value());
  EXPECT_FALSE(decode_fd_command(command, 100U).has_value());

  // A legal FD length that does not match this frame type's layout.
  EXPECT_FALSE(decode_fd_command(command, kFdFeedbackDlc).has_value());
  EXPECT_FALSE(decode_fd_feedback(feedback, kFdCommandDlc).has_value());
}

TEST(CanFdProtocol, RejectsWrongVersionAndFrameType)
{
  auto command = encode_fd_command({kRequest, true, 1000, 200});
  command[1] = kFdProfileVersion + 1U;
  EXPECT_FALSE(decode_fd_command(command).has_value());

  auto feedback = encode_fd_feedback({kRequest, 3U, 1000, 10, 20U});
  feedback[13] = 0x01U;
  EXPECT_FALSE(decode_fd_feedback(feedback).has_value());
}

TEST(CanFdProtocol, RejectsDirtyReservedBytes)
{
  auto command = encode_fd_command({kRequest, true, 1000, 200});
  command[kFdCommandDlc - 1U] = 1U;
  EXPECT_FALSE(decode_fd_command(command).has_value());

  auto feedback = encode_fd_feedback({kRequest, 3U, 1000, 10, 20U});
  feedback[kFdFeedbackDlc - 1U] = 1U;
  EXPECT_FALSE(decode_fd_feedback(feedback).has_value());

  auto ack = encode_fd_ack({kRequest, kFdResultAccepted});
  ack[kFdAckDlc - 1U] = 1U;
  EXPECT_FALSE(decode_fd_ack(ack).has_value());
}

TEST(CanFdProtocol, RejectsFeedbackWithInconsistentStatusCopy)
{
  auto feedback = encode_fd_feedback({kRequest, 0x0003U, 1000, 10, 20U});
  // Byte 2 and the widened status field must agree.
  feedback[2] = 0x09U;
  EXPECT_FALSE(decode_fd_feedback(feedback).has_value());
}

TEST(CanFdProtocol, AcceptsEveryLegalFdLengthAndRejectsOthers)
{
  for (const std::size_t dlc : {8U, 9U, 10U, 11U, 13U, 63U, 65U, 0U}) {
    EXPECT_FALSE(fd_detail::valid_dlc(dlc)) << "dlc " << dlc;
  }
  for (const std::size_t dlc : {12U, 16U, 20U, 24U, 28U, 32U, 48U, 64U}) {
    EXPECT_TRUE(fd_detail::valid_dlc(dlc)) << "dlc " << dlc;
  }
  // The smallest payload a non-bit-rate-switched FD frame can carry.
  EXPECT_EQ(kFdUnswitchedMinDlc, 12U);
}

TEST(CanFdProtocol, ClassicAndFdProfilesAreNotInterchangeable)
{
  // A classic command has no version tag and DLC 8, so the FD decoder must
  // refuse it, and the classic decoder must refuse an FD frame.
  const auto classic = encode_command(7, true, 1000, 200);
  ASSERT_TRUE(decode_command(classic).has_value());

  FdFrameData widened{};
  std::copy(classic.begin(), classic.end(), widened.begin());
  EXPECT_FALSE(decode_fd_command(widened, classic.size()).has_value());

  const auto fd_command = encode_fd_command({kRequest, true, 1000, 200});
  FrameData narrowed{};
  std::copy_n(fd_command.begin(), narrowed.size(), narrowed.begin());
  EXPECT_FALSE(decode_command(narrowed, narrowed.size()).has_value());
}
