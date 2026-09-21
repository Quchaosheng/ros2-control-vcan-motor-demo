#ifndef VCAN_DIFFBOT_DEMO__CAN_SENDER_HPP_
#define VCAN_DIFFBOT_DEMO__CAN_SENDER_HPP_

#include <memory>
#include <string>
#include <type_traits>

#include "ros2_socketcan/socket_can_sender.hpp"

namespace vcan_diffbot_demo
{

// ros2_socketcan 1.4.0 inserted `enable_loopback` between `enable_fd` and
// `default_id`, defaulting it to false. A vcan interface has no physical
// medium, so local loopback is the only path that delivers a frame to the
// other processes sharing that interface. With loopback disabled the sender's
// frames never reach its peer and every vcan round trip stalls, while all pure
// policy tests keep passing.
//
// Older ros2_socketcan releases have no such argument, so detect it instead of
// pinning a single upstream API.
template<typename Sender, typename = void>
struct sender_has_loopback_argument : std::false_type {};

template<typename Sender>
struct sender_has_loopback_argument<Sender, std::void_t<decltype(
    Sender(
      std::declval<const std::string &>(), std::declval<const bool>(),
      std::declval<const bool>()))
  >> : std::true_type {};

/// Create a sender that keeps local loopback enabled on every supported
/// ros2_socketcan release.
template<typename Sender = drivers::socketcan::SocketCanSender>
std::unique_ptr<Sender> make_loopback_sender(const std::string & interface)
{
  if constexpr (sender_has_loopback_argument<Sender>::value) {
    return std::make_unique<Sender>(interface, false, true);
  } else {
    return std::make_unique<Sender>(interface);
  }
}

}  // namespace vcan_diffbot_demo

#endif  // VCAN_DIFFBOT_DEMO__CAN_SENDER_HPP_
