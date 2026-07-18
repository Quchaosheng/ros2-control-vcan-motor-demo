#include <gtest/gtest.h>

#include "vcan_diffbot_demo/can_protocol.hpp"

using namespace vcan_diffbot_demo::protocol;

TEST(CanProtocol, UsesExpectedStandardIdentifiers)
{
  EXPECT_EQ(command_id(1), 0x101U);
  EXPECT_EQ(feedback_id(2), 0x182U);
  EXPECT_EQ(ack_id(2), 0x282U);
}

TEST(CanProtocol, EncodesNegativeVelocityLittleEndian)
{
  const auto frame = encode_command(0x2a, true, -1234, 200);
  EXPECT_EQ(frame, (FrameData{0x2a, 0x01, 0x2e, 0xfb, 0xc8, 0x00, 0x00, 0x00}));
}

TEST(CanProtocol, SaturatesCommandVelocity)
{
  EXPECT_EQ(decode_command(encode_command(1, true, 40000, 200))->velocity_mrad_s, 32767);
  EXPECT_EQ(decode_command(encode_command(1, true, -40000, 200))->velocity_mrad_s, -32768);
}

TEST(CanProtocol, RejectsCommandWithNonzeroReservedBytes)
{
  auto frame = encode_command(1, true, 1000, 200);
  frame[6] = 1;
  EXPECT_FALSE(decode_command(frame).has_value());
}

TEST(CanProtocol, EncodesAndDecodesFeedback)
{
  const Feedback input{7, 3, 1234, 123456};
  const auto bytes = encode_feedback(input);
  EXPECT_EQ(bytes, (FrameData{7, 3, 0xd2, 0x04, 0x40, 0xe2, 0x01, 0x00}));

  const auto feedback = decode_feedback(bytes);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_EQ(feedback->sequence, input.sequence);
  EXPECT_EQ(feedback->status, input.status);
  EXPECT_EQ(feedback->velocity_mrad_s, input.velocity_mrad_s);
  EXPECT_EQ(feedback->encoder_count, input.encoder_count);
}

TEST(CanProtocol, EncodesAndDecodesAck)
{
  const auto bytes = encode_ack({255, 2});
  EXPECT_EQ(bytes, (FrameData{255, 2, 0, 0, 0, 0, 0, 0}));

  const auto ack = decode_ack(bytes);
  ASSERT_TRUE(ack.has_value());
  EXPECT_EQ(ack->sequence, 255);
  EXPECT_EQ(ack->result, 2);
}

TEST(CanProtocol, RejectsWrongDlc)
{
  EXPECT_FALSE(decode_command(FrameData{}, 7).has_value());
  EXPECT_FALSE(decode_feedback(FrameData{}, 7).has_value());
  EXPECT_FALSE(decode_ack(FrameData{}, 6).has_value());
}
