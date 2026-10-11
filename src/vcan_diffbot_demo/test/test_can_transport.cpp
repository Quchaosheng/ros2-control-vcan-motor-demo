#include <gtest/gtest.h>

#include <string>

#include "vcan_diffbot_demo/can_transport.hpp"

using namespace vcan_diffbot_demo;

TEST(CanTransport, DefaultsToTheClassicProfile)
{
  const TransportConfig config;

  EXPECT_EQ(config.profile, TransportProfile::ClassicCan);
  EXPECT_EQ(config.brs, BitRateSwitch::Auto);
  EXPECT_FALSE(bit_rate_switch_enabled(config));
  EXPECT_TRUE(payload_length_allowed(config, 8U));
  EXPECT_FALSE(payload_length_allowed(config, 24U));
}

TEST(CanTransport, ParsesOnlyKnownProfileNames)
{
  TransportProfile profile = TransportProfile::ClassicCan;
  EXPECT_TRUE(parse_transport_profile("canfd", profile));
  EXPECT_EQ(profile, TransportProfile::CanFd);
  EXPECT_TRUE(parse_transport_profile("classic_can", profile));
  EXPECT_EQ(profile, TransportProfile::ClassicCan);

  // A typo must never select a wire contract.
  EXPECT_FALSE(parse_transport_profile("can_fd", profile));
  EXPECT_FALSE(parse_transport_profile("CANFD", profile));
  EXPECT_FALSE(parse_transport_profile("", profile));
}

TEST(CanTransport, ParsesOnlyKnownBitRateSwitchPolicies)
{
  BitRateSwitch brs = BitRateSwitch::Auto;
  EXPECT_TRUE(parse_bit_rate_switch("enabled", brs));
  EXPECT_EQ(brs, BitRateSwitch::Enabled);
  EXPECT_TRUE(parse_bit_rate_switch("disabled", brs));
  EXPECT_EQ(brs, BitRateSwitch::Disabled);
  EXPECT_FALSE(parse_bit_rate_switch("on", brs));
}

TEST(CanTransport, RejectsFdWhenTheSocketApiCannotSupportIt)
{
  TransportConfig fd;
  fd.profile = TransportProfile::CanFd;
  std::string error;

  EXPECT_FALSE(validate_transport(fd, false, error));
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(validate_transport(fd, true, error));
  EXPECT_TRUE(error.empty());
}

TEST(CanTransport, RejectsFdDataRateThatIsNotFasterThanArbitration)
{
  TransportConfig fd;
  fd.profile = TransportProfile::CanFd;
  std::string error;

  fd.data_bitrate = fd.arbitration_bitrate;
  EXPECT_FALSE(validate_transport(fd, true, error));
  fd.data_bitrate = fd.arbitration_bitrate - 1U;
  EXPECT_FALSE(validate_transport(fd, true, error));
  fd.data_bitrate = fd.arbitration_bitrate + 1U;
  EXPECT_TRUE(validate_transport(fd, true, error));
}

TEST(CanTransport, RejectsFdOnlyOptionsOnTheClassicProfile)
{
  TransportConfig classic;
  std::string error;

  classic.brs = BitRateSwitch::Enabled;
  EXPECT_FALSE(validate_transport(classic, true, error));
  EXPECT_FALSE(error.empty());

  classic.brs = BitRateSwitch::Auto;
  classic.data_bitrate = 5000000U;
  EXPECT_FALSE(validate_transport(classic, true, error));
  EXPECT_TRUE(validate_transport(TransportConfig{}, true, error));
}

TEST(CanTransport, RejectsZeroBitrates)
{
  std::string error;
  TransportConfig classic;
  classic.arbitration_bitrate = 0U;
  EXPECT_FALSE(validate_transport(classic, true, error));
}

TEST(CanTransport, RejectsEndpointAndProfileMismatch)
{
  // A configured classic endpoint must not accept FD frames, and an FD
  // endpoint must not accept classic frames.
  EXPECT_TRUE(accepts_frame(TransportProfile::ClassicCan, false));
  EXPECT_FALSE(accepts_frame(TransportProfile::ClassicCan, true));
  EXPECT_TRUE(accepts_frame(TransportProfile::CanFd, true));
  EXPECT_FALSE(accepts_frame(TransportProfile::CanFd, false));

  EXPECT_EQ(
    match_frame(TransportProfile::ClassicCan, true), FrameMatch::ProfileMismatch);
  EXPECT_EQ(
    match_frame(TransportProfile::CanFd, false), FrameMatch::ProfileMismatch);
  EXPECT_EQ(
    match_frame(TransportProfile::CanFd, true), FrameMatch::Accepted);
}

TEST(CanTransport, MapsProfileToEndpoint)
{
  EXPECT_EQ(endpoint_of(TransportProfile::ClassicCan), TransportEndpoint::EndpointClassic);
  EXPECT_EQ(endpoint_of(TransportProfile::CanFd), TransportEndpoint::EndpointFd);
}

TEST(CanTransport, EnablesBitRateSwitchOnlyOnFd)
{
  TransportConfig config;
  config.brs = BitRateSwitch::Enabled;
  EXPECT_FALSE(bit_rate_switch_enabled(config));

  config.profile = TransportProfile::CanFd;
  EXPECT_TRUE(bit_rate_switch_enabled(config));
  config.brs = BitRateSwitch::Disabled;
  EXPECT_FALSE(bit_rate_switch_enabled(config));
}

TEST(CanTransport, ReportsTheConfiguredProfileForDiagnostics)
{
  TransportConfig config;
  EXPECT_EQ(describe_transport(config), "classic_can/v1/brs=auto");

  config.profile = TransportProfile::CanFd;
  config.brs = BitRateSwitch::Enabled;
  EXPECT_EQ(describe_transport(config), "canfd/v1/brs=enabled");
  EXPECT_STREQ(profile_name(config.profile), "canfd");
}

TEST(CanTransport, RejectsASocketOpenedForTheWrongProfile)
{
  std::string error;
  TransportConfig classic;
  TransportConfig fd;
  fd.profile = TransportProfile::CanFd;

  EXPECT_TRUE(check_socket_matches(classic, false, error));
  EXPECT_TRUE(error.empty());
  EXPECT_FALSE(check_socket_matches(classic, true, error));
  EXPECT_FALSE(error.empty());

  EXPECT_TRUE(check_socket_matches(fd, true, error));
  // An FD configuration on a classic socket must fail rather than fall back.
  EXPECT_FALSE(check_socket_matches(fd, false, error));
  EXPECT_FALSE(error.empty());
}

TEST(CanTransport, EnablesTheFdSocketOnlyForTheFdProfile)
{
  TransportConfig config;
  EXPECT_FALSE(fd_socket_enabled(config));
  config.profile = TransportProfile::CanFd;
  EXPECT_TRUE(fd_socket_enabled(config));
}

TEST(CanTransport, BuildsAStandardFrameIdentifierForBothProfiles)
{
  TransportConfig config;
  // The classic profile keeps the plain 11-bit standard identifier.
  const auto classic = transport_can_id(
    config, 0x101U, drivers::socketcan::FrameType::DATA);
  EXPECT_EQ(classic.identifier(), 0x101U);
  EXPECT_EQ(classic.frame_type(), drivers::socketcan::FrameType::DATA);
  EXPECT_FALSE(classic.is_extended());

  // ros2_socketcan 1.4.0 carries CAN-FD on the socket, not the identifier, so
  // the identifier itself stays a standard data frame for the FD profile too.
  config.profile = TransportProfile::CanFd;
  const auto fd = transport_can_id(
    config, 0x180U, drivers::socketcan::FrameType::DATA);
  EXPECT_EQ(fd.identifier(), 0x180U);
  EXPECT_FALSE(fd.is_extended());
}
