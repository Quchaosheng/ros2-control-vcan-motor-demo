#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <type_traits>

#include "vcan_diffbot_demo/can_sender.hpp"

namespace
{

// Mirrors the ros2_socketcan 1.4.0 signature, which takes `enable_loopback`
// between `enable_fd` and `default_id`.
struct ModernSender
{
  ModernSender(const std::string & interface, const bool enable_fd, const bool enable_loopback)
  : interface{interface}, enable_fd{enable_fd}, enable_loopback{enable_loopback} {}

  std::string interface;
  bool enable_fd;
  bool enable_loopback;
};

// Mirrors releases that predate the loopback argument.
struct LegacySender
{
  explicit LegacySender(const std::string & interface, const bool enable_fd = false)
  : interface{interface}, enable_fd{enable_fd} {}

  std::string interface;
  bool enable_fd;
};

}  // namespace

TEST(CanSender, DetectsLoopbackArgumentOnlyWhenUpstreamProvidesIt)
{
  EXPECT_TRUE(
    (vcan_diffbot_demo::sender_has_loopback_argument<ModernSender>::value));
  EXPECT_FALSE(
    (vcan_diffbot_demo::sender_has_loopback_argument<LegacySender>::value));
}

TEST(CanSender, RequestsLoopbackOnReleasesThatSupportIt)
{
  // vcan has no physical medium, so a sender that leaves CAN_RAW_LOOPBACK
  // disabled cannot deliver frames to the processes sharing the interface.
  const auto sender = vcan_diffbot_demo::make_loopback_sender<ModernSender>("vcantest");
  ASSERT_NE(sender, nullptr);
  EXPECT_EQ(sender->interface, "vcantest");
  EXPECT_FALSE(sender->enable_fd);
  EXPECT_TRUE(sender->enable_loopback);
}

TEST(CanSender, KeepsLegacyConstructionWhenLoopbackIsUnavailable)
{
  const auto sender = vcan_diffbot_demo::make_loopback_sender<LegacySender>("vcantest");
  ASSERT_NE(sender, nullptr);
  EXPECT_EQ(sender->interface, "vcantest");
  EXPECT_FALSE(sender->enable_fd);
}

TEST(CanSender, HelperReturnsTheUpstreamSenderType)
{
  // Compile-time only: the helper must select the request-loopback branch for
  // whichever ros2_socketcan release is installed. Constructing a real sender
  // here would need a live CAN interface, which unit tests must not require.
  static_assert(
    std::is_same_v<
      decltype(vcan_diffbot_demo::make_loopback_sender("vcantest")),
      std::unique_ptr<drivers::socketcan::SocketCanSender>>,
    "make_loopback_sender must return the upstream sender type");
}
