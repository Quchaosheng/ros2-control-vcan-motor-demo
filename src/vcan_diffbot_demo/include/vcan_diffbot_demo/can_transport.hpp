#ifndef VCAN_DIFFBOT_DEMO__CAN_TRANSPORT_HPP_
#define VCAN_DIFFBOT_DEMO__CAN_TRANSPORT_HPP_

#include <string>

#include "ros2_socketcan/socket_can_id.hpp"
#include "vcan_diffbot_demo/can_protocol.hpp"
#include "vcan_diffbot_demo/canfd_protocol.hpp"

namespace vcan_diffbot_demo
{

/// Transport profile is an explicit configuration choice, never inferred from
/// the interface: the classic 11-bit DLC-8 profile stays the default so an
/// existing deployment cannot be moved onto CAN-FD by accident.
enum class TransportProfile
{
  ClassicCan,
  CanFd,
};

/// Bit-rate-switching policy. `Auto` lets the kernel pick, which is what a
/// vcan interface does; the explicit values exist because BRS changes the data
/// phase timing and therefore what a peer may legally send.
enum class BitRateSwitch
{
  Auto,
  Disabled,
  Enabled,
};

struct TransportConfig
{
  TransportProfile profile{TransportProfile::ClassicCan};
  BitRateSwitch brs{BitRateSwitch::Auto};
  unsigned int arbitration_bitrate{500000U};
  unsigned int data_bitrate{2000000U};
};

/// The classic and FD profiles are told apart by two independent signals: the
/// interface configuration and the version tag inside the frame. `Endpoint`
/// records the configured signal so a mismatch is reported rather than
/// silently tolerated.
enum class TransportEndpoint
{
  EndpointClassic,
  EndpointFd,
};

constexpr TransportEndpoint endpoint_of(const TransportProfile profile)
{
  return profile == TransportProfile::CanFd ?
         TransportEndpoint::EndpointFd : TransportEndpoint::EndpointClassic;
}

constexpr const char * profile_name(const TransportProfile profile)
{
  return profile == TransportProfile::CanFd ? "canfd" : "classic_can";
}

constexpr const char * brs_name(const BitRateSwitch brs)
{
  switch (brs) {
    case BitRateSwitch::Auto:
      return "auto";
    case BitRateSwitch::Disabled:
      return "disabled";
    case BitRateSwitch::Enabled:
      return "enabled";
  }
  return "auto";
}

/// Parse an operator-supplied profile string. Anything unknown is an error: a
/// typo must not silently select the wrong wire contract.
inline bool parse_transport_profile(const std::string & text, TransportProfile & out)
{
  if (text == "classic_can") {
    out = TransportProfile::ClassicCan;
    return true;
  }
  if (text == "canfd") {
    out = TransportProfile::CanFd;
    return true;
  }
  return false;
}

inline bool parse_bit_rate_switch(const std::string & text, BitRateSwitch & out)
{
  if (text == "auto") {
    out = BitRateSwitch::Auto;
    return true;
  }
  if (text == "disabled") {
    out = BitRateSwitch::Disabled;
    return true;
  }
  if (text == "enabled") {
    out = BitRateSwitch::Enabled;
    return true;
  }
  return false;
}

/// Checks a decoded frame against the profile the endpoint is configured for.
/// A configured classic endpoint must not accept an FD frame, and an FD
/// endpoint must not accept a classic frame whose lack of a version tag cannot
/// be distinguished from a version mismatch.
inline bool accepts_frame(const TransportProfile profile, const bool frame_is_fd)
{
  return (profile == TransportProfile::CanFd) == frame_is_fd;
}

/// Result of checking a received frame against this endpoint's contract.
enum class FrameMatch
{
  Accepted,
  ProfileMismatch,
};

inline FrameMatch match_frame(const TransportProfile profile, const bool frame_is_fd)
{
  return accepts_frame(profile, frame_is_fd) ? FrameMatch::Accepted : FrameMatch::ProfileMismatch;
}

/// The CAN-FD socket option is only settable through the upstream API when the
/// sender was constructed for FD. Older ros2_socketcan releases have no FD
/// support, so an FD configuration there fails loudly instead of sending
/// classic frames to a peer that expects FD.
constexpr bool transport_supported(const TransportProfile profile, const bool fd_api_available)
{
  return profile == TransportProfile::ClassicCan || fd_api_available;
}

/// Reject a configuration the local socket cannot actually honour. Returning a
/// reason keeps the failure legible in a log instead of a bare false.
inline bool validate_transport(const TransportConfig & config, const bool fd_api_available,
  std::string & error)
{
  if (!transport_supported(config.profile, fd_api_available)) {
    error = "CAN-FD transport requires ros2_socketcan with CAN-FD support";
    return false;
  }
  if (config.profile != TransportProfile::CanFd &&
    (config.brs != BitRateSwitch::Auto || config.data_bitrate != 2000000U))
  {
    error = "bit-rate switching and data_bitrate only apply to the canfd profile";
    return false;
  }
  if (config.arbitration_bitrate == 0U || config.data_bitrate == 0U) {
    error = "bitrates must be non-zero";
    return false;
  }
  if (config.profile == TransportProfile::CanFd &&
    config.data_bitrate <= config.arbitration_bitrate)
  {
    error = "CAN-FD data_bitrate must exceed arbitration_bitrate";
    return false;
  }
  error.clear();
  return true;
}

/// Bit-rate switching is only meaningful on an FD endpoint.
constexpr bool bit_rate_switch_enabled(const TransportConfig & config)
{
  return config.profile == TransportProfile::CanFd && config.brs == BitRateSwitch::Enabled;
}

/// Payload lengths this endpoint may put on the wire. A classic endpoint is
/// fixed at DLC 8; an FD endpoint may use any length the FD data phase accepts.
/// shortcut: lengths legal for FD but larger than the defined layouts are not
/// used yet, upgrade when a layout needs them.
constexpr bool payload_length_allowed(const TransportConfig & config, const std::size_t dlc)
{
  return config.profile == TransportProfile::CanFd ?
         protocol::fd_detail::valid_dlc(dlc) : dlc == 8U;
}

/// Human-readable endpoint description used in diagnostics and error text.
inline std::string describe_transport(const TransportConfig & config)
{
  return std::string(profile_name(config.profile)) + "/v" +
         std::to_string(protocol::kFdProfileVersion) + "/brs=" + brs_name(config.brs);
}

/// Build the socket identifier for a frame. The classic profile keeps the plain
/// 11-bit standard identifier. ros2_socketcan 1.4.0 exposes CAN-FD only through
/// the socket (`enable_fd` at construction and `send_fd`), not through the
/// identifier, so no FD flag is attached here; the frame-type tag is the only
/// identifier-side signal both profiles share.
inline drivers::socketcan::CanId transport_can_id(
  const TransportConfig &, const uint32_t identifier,
  const drivers::socketcan::FrameType frame_type)
{
  return drivers::socketcan::CanId(
    identifier, 0U, frame_type, drivers::socketcan::StandardFrame);
}

/// True when the installed ros2_socketcan can enable a CAN-FD socket. The FD
/// capability is a constructor argument rather than a probeable runtime state,
/// so this records the contract the two callers must honour: the sender and the
/// receiver are both constructed with `enable_fd`.
constexpr bool fd_socket_enabled(const TransportConfig & config)
{
  return config.profile == TransportProfile::CanFd;
}

/// A CAN-FD socket must be opened for FD before the first frame, so an FD
/// configuration on a socket that was created classic is a hard error rather
/// than a silent downgrade.
inline bool check_socket_matches(const TransportConfig & config, const bool socket_is_fd,
  std::string & error)
{
  if (fd_socket_enabled(config) != socket_is_fd) {
    error = std::string("transport ") + profile_name(config.profile) +
      " does not match a " + (socket_is_fd ? "CAN-FD" : "classic CAN") + " socket";
    return false;
  }
  error.clear();
  return true;
}

}  // namespace vcan_diffbot_demo

#endif  // VCAN_DIFFBOT_DEMO__CAN_TRANSPORT_HPP_
