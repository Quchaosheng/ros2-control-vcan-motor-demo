#ifndef VCAN_DIFFBOT_DEMO__CANFD_PROTOCOL_HPP_
#define VCAN_DIFFBOT_DEMO__CANFD_PROTOCOL_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "vcan_diffbot_demo/can_protocol.hpp"

namespace vcan_diffbot_demo::protocol
{

/// CAN-FD is an opt-in transport profile. It shares the classic identifiers and
/// every field the classic profile already defined, so the two can be told
/// apart only by an explicit version tag and an explicit frame length.
///
/// Every CAN-FD frame carries the version in byte 1. The classic profile has no
/// version field, which is why the profiles cannot be silently mixed: an FD
/// frame never decodes as classic, and the hardware refuses a mismatched peer
/// instead of guessing.
///
/// Only the lengths an ISO 11898-1 CAN-FD data phase can actually carry are
/// accepted. Without bit-rate switching a CAN-FD frame must be at least
/// `kFdUnswitchedMinDlc` bytes, so 12 and 16 are the smallest legal payloads.
constexpr uint8_t kFdProfileVersion = 1U;
constexpr std::size_t kFdMinDlc = 12U;
constexpr std::size_t kFdMaxDlc = 64U;
constexpr std::size_t kFdUnswitchedMinDlc = 12U;
constexpr std::size_t kFdCommandDlc = 20U;
constexpr std::size_t kFdFeedbackDlc = 28U;
constexpr std::size_t kFdAckDlc = 16U;

using FdFrameData = std::array<uint8_t, kFdMaxDlc>;

/// Result codes. `kFdResultRejected` is classic result 2 (invalid reserved
/// bytes) carried over into the FD profile.
constexpr uint8_t kFdResultAccepted = 0U;
constexpr uint8_t kFdResultUnsupportedDlc = 1U;
constexpr uint8_t kFdResultProfileMismatch = 2U;
constexpr uint8_t kFdResultRejected = 3U;

/// `Feedback::sequence` is only 8 bits. The FD header widens the request
/// identity with a 32-bit command id so a late ACK is attributable to one
/// specific command instead of to a wrapped sequence number.
struct FdRequestId
{
  uint32_t node_id;
  uint32_t command_id;

  friend constexpr bool operator==(const FdRequestId & lhs, const FdRequestId & rhs)
  {
    return lhs.node_id == rhs.node_id && lhs.command_id == rhs.command_id;
  }
};

struct FdCommand
{
  FdRequestId request;
  bool enabled;
  int16_t velocity_mrad_s;
  uint16_t watchdog_ms;
};

struct FdFeedback
{
  FdRequestId request;
  uint16_t status;
  int16_t velocity_mrad_s;
  int32_t encoder_count;
  uint32_t timestamp_us;
};

struct FdAck
{
  FdRequestId request;
  uint8_t result;
};

namespace fd_detail
{

inline void put_u16(FdFrameData & data, const std::size_t offset, const uint16_t value)
{
  data[offset] = static_cast<uint8_t>(value & 0xffU);
  data[offset + 1] = static_cast<uint8_t>((value >> 8U) & 0xffU);
}

inline void put_u32(FdFrameData & data, const std::size_t offset, const uint32_t value)
{
  for (std::size_t index = 0; index < 4; ++index) {
    data[offset + index] = static_cast<uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

inline uint16_t get_u16(const FdFrameData & data, const std::size_t offset)
{
  return static_cast<uint16_t>(
    static_cast<uint16_t>(data[offset]) |
    (static_cast<uint16_t>(data[offset + 1]) << 8U));
}

inline uint32_t get_u32(const FdFrameData & data, const std::size_t offset)
{
  uint32_t raw = 0;
  for (std::size_t index = 0; index < 4; ++index) {
    raw |= static_cast<uint32_t>(data[offset + index]) << (8U * index);
  }
  return raw;
}

inline void put_i16(FdFrameData & data, const std::size_t offset, const int16_t value)
{
  put_u16(data, offset, static_cast<uint16_t>(value));
}

inline void put_i32(FdFrameData & data, const std::size_t offset, const int32_t value)
{
  put_u32(data, offset, static_cast<uint32_t>(value));
}

inline int16_t get_i16(const FdFrameData & data, const std::size_t offset)
{
  return static_cast<int16_t>(get_u16(data, offset));
}

inline int32_t get_i32(const FdFrameData & data, const std::size_t offset)
{
  return static_cast<int32_t>(get_u32(data, offset));
}

/// True for the payload lengths a CAN-FD data phase accepts.
inline bool valid_dlc(const std::size_t dlc)
{
  return dlc >= kFdMinDlc && dlc <= kFdMaxDlc && dlc % 4U == 0U;
}

inline bool zero_from(const FdFrameData & data, const std::size_t offset, const std::size_t dlc)
{
  for (std::size_t index = offset; index < dlc; ++index) {
    if (data[index] != 0U) {
      return false;
    }
  }
  return true;
}

/// Byte 0 mirrors classic byte 0 for the low 8 bits so an FD frame still shows
/// the sequence a human reads in candump; the widened id carries the full value.
/// The command id lives at byte 4 and the node id at byte 8, and no payload
/// field may reuse those offsets. Bytes 1 and 13 are the version and frame-type
/// tags, filled in by the caller.
inline void put_request(FdFrameData & data, const FdRequestId & request)
{
  data[0] = static_cast<uint8_t>(request.command_id & 0xffU);
  put_u32(data, 4, request.command_id);
  put_u32(data, 8, request.node_id);
}

/// Whole-frame consistency checks shared by every decoder: a legal FD length, a
/// declared length that is big enough for this frame type, the version tag, the
/// frame-type tag, and a zeroed reserved tail. These reject a truncated frame
/// and one padded past the layout with trailing garbage.
inline bool consistent_frame(
  const FdFrameData & data, const std::size_t dlc, const std::size_t expected_dlc,
  const std::size_t reserved_offset, const uint8_t frame_type, const uint8_t type_bitmap)
{
  return valid_dlc(dlc) && dlc == expected_dlc && data[1] == kFdProfileVersion &&
         data[13] == frame_type && (frame_type & type_bitmap) != 0U &&
         zero_from(data, reserved_offset, dlc);
}

}  // namespace fd_detail

inline FdFrameData encode_fd_command(const FdCommand & command)
{
  FdFrameData data{};
  fd_detail::put_request(data, command.request);
  data[1] = kFdProfileVersion;
  data[2] = command.enabled ? 0x01U : 0x00U;
  fd_detail::put_i16(data, 14, command.velocity_mrad_s);
  fd_detail::put_u16(data, 16, command.watchdog_ms);
  data[13] = 0x01U;
  return data;
}

inline std::optional<FdCommand> decode_fd_command(
  const FdFrameData & data, const std::size_t dlc = kFdCommandDlc)
{
  if (!fd_detail::consistent_frame(data, dlc, kFdCommandDlc, 18U, 0x01U, 0x01U)) {
    return std::nullopt;
  }
  return FdCommand{
    FdRequestId{fd_detail::get_u32(data, 8), fd_detail::get_u32(data, 4)},
    (data[2] & 0x01U) != 0U, fd_detail::get_i16(data, 14), fd_detail::get_u16(data, 16)};
}

inline FdFrameData encode_fd_feedback(const FdFeedback & feedback)
{
  FdFrameData data{};
  fd_detail::put_request(data, feedback.request);
  data[1] = kFdProfileVersion;
  data[2] = static_cast<uint8_t>(feedback.status & 0xffU);
  fd_detail::put_u16(data, 14, feedback.status);
  fd_detail::put_i16(data, 16, feedback.velocity_mrad_s);
  fd_detail::put_i32(data, 18, feedback.encoder_count);
  fd_detail::put_u32(data, 22, feedback.timestamp_us);
  data[13] = 0x02U;
  return data;
}

inline std::optional<FdFeedback> decode_fd_feedback(
  const FdFrameData & data, const std::size_t dlc = kFdFeedbackDlc)
{
  if (!fd_detail::consistent_frame(data, dlc, kFdFeedbackDlc, 26U, 0x02U, 0x02U)) {
    return std::nullopt;
  }
  const uint8_t low_status = static_cast<uint8_t>(data[2]);
  const uint16_t status = fd_detail::get_u16(data, 14);
  // Byte 2 is the classic status byte, so the two copies must agree.
  if (static_cast<uint8_t>(status & 0xffU) != low_status) {
    return std::nullopt;
  }
  return FdFeedback{
    FdRequestId{fd_detail::get_u32(data, 8), fd_detail::get_u32(data, 4)},
    status, fd_detail::get_i16(data, 16), fd_detail::get_i32(data, 18),
    fd_detail::get_u32(data, 22)};
}

inline FdFrameData encode_fd_ack(const FdAck & ack)
{
  FdFrameData data{};
  fd_detail::put_request(data, ack.request);
  data[1] = kFdProfileVersion;
  data[13] = 0x03U;
  data[14] = ack.result;
  return data;
}

inline std::optional<FdAck> decode_fd_ack(
  const FdFrameData & data, const std::size_t dlc = kFdAckDlc)
{
  if (!fd_detail::consistent_frame(data, dlc, kFdAckDlc, 15U, 0x03U, 0x03U)) {
    return std::nullopt;
  }
  return FdAck{FdRequestId{fd_detail::get_u32(data, 8), fd_detail::get_u32(data, 4)}, data[14]};
}

}  // namespace vcan_diffbot_demo::protocol

#endif  // VCAN_DIFFBOT_DEMO__CANFD_PROTOCOL_HPP_
