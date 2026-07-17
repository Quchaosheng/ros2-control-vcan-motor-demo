#ifndef VCAN_DIFFBOT_DEMO__CAN_PROTOCOL_HPP_
#define VCAN_DIFFBOT_DEMO__CAN_PROTOCOL_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace vcan_diffbot_demo::protocol
{

using FrameData = std::array<uint8_t, 8>;

constexpr uint32_t command_id(const uint8_t node_id)
{
  return 0x100U + node_id;
}

constexpr uint32_t feedback_id(const uint8_t node_id)
{
  return 0x180U + node_id;
}

constexpr uint32_t ack_id(const uint8_t node_id)
{
  return 0x280U + node_id;
}

struct Command
{
  uint8_t sequence;
  bool enabled;
  int16_t velocity_mrad_s;
  uint16_t watchdog_ms;
};

struct Feedback
{
  uint8_t sequence;
  uint8_t status;
  int16_t velocity_mrad_s;
  int32_t encoder_count;
};

struct Ack
{
  uint8_t sequence;
  uint8_t result;
};

namespace detail
{

inline void put_u16(FrameData & data, const std::size_t offset, const uint16_t value)
{
  data[offset] = static_cast<uint8_t>(value & 0xffU);
  data[offset + 1] = static_cast<uint8_t>((value >> 8U) & 0xffU);
}

inline void put_i16(FrameData & data, const std::size_t offset, const int16_t value)
{
  put_u16(data, offset, static_cast<uint16_t>(value));
}

inline void put_i32(FrameData & data, const std::size_t offset, const int32_t value)
{
  const auto raw = static_cast<uint32_t>(value);
  for (std::size_t index = 0; index < 4; ++index) {
    data[offset + index] = static_cast<uint8_t>((raw >> (8U * index)) & 0xffU);
  }
}

inline uint16_t get_u16(const FrameData & data, const std::size_t offset)
{
  return static_cast<uint16_t>(
    static_cast<uint16_t>(data[offset]) |
    (static_cast<uint16_t>(data[offset + 1]) << 8U));
}

inline int16_t get_i16(const FrameData & data, const std::size_t offset)
{
  return static_cast<int16_t>(get_u16(data, offset));
}

inline int32_t get_i32(const FrameData & data, const std::size_t offset)
{
  uint32_t raw = 0;
  for (std::size_t index = 0; index < 4; ++index) {
    raw |= static_cast<uint32_t>(data[offset + index]) << (8U * index);
  }
  return static_cast<int32_t>(raw);
}

}  // namespace detail

inline FrameData encode_command(
  const uint8_t sequence, const bool enabled, const int32_t velocity_mrad_s,
  const uint16_t watchdog_ms)
{
  FrameData data{};
  data[0] = sequence;
  data[1] = enabled ? 0x01U : 0x00U;
  const auto velocity = std::clamp(
    velocity_mrad_s,
    static_cast<int32_t>(std::numeric_limits<int16_t>::min()),
    static_cast<int32_t>(std::numeric_limits<int16_t>::max()));
  detail::put_i16(data, 2, static_cast<int16_t>(velocity));
  detail::put_u16(data, 4, watchdog_ms);
  return data;
}

inline std::optional<Command> decode_command(
  const FrameData & data, const std::size_t dlc = 8U)
{
  if (dlc != data.size() || data[6] != 0U || data[7] != 0U) {
    return std::nullopt;
  }
  return Command{data[0], (data[1] & 0x01U) != 0U, detail::get_i16(data, 2),
    detail::get_u16(data, 4)};
}

inline FrameData encode_feedback(const Feedback & feedback)
{
  FrameData data{};
  data[0] = feedback.sequence;
  data[1] = feedback.status;
  detail::put_i16(data, 2, feedback.velocity_mrad_s);
  detail::put_i32(data, 4, feedback.encoder_count);
  return data;
}

inline std::optional<Feedback> decode_feedback(
  const FrameData & data, const std::size_t dlc = 8U)
{
  if (dlc != data.size()) {
    return std::nullopt;
  }
  return Feedback{data[0], data[1], detail::get_i16(data, 2), detail::get_i32(data, 4)};
}

inline FrameData encode_ack(const Ack & ack)
{
  FrameData data{};
  data[0] = ack.sequence;
  data[1] = ack.result;
  return data;
}

inline std::optional<Ack> decode_ack(
  const FrameData & data, const std::size_t dlc = 8U)
{
  if (dlc != data.size()) {
    return std::nullopt;
  }
  for (std::size_t index = 2; index < data.size(); ++index) {
    if (data[index] != 0U) {
      return std::nullopt;
    }
  }
  return Ack{data[0], data[1]};
}

}  // namespace vcan_diffbot_demo::protocol

#endif  // VCAN_DIFFBOT_DEMO__CAN_PROTOCOL_HPP_
