#ifndef VCAN_DIFFBOT_DEMO__HARDWARE_HEALTH_HPP_
#define VCAN_DIFFBOT_DEMO__HARDWARE_HEALTH_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace vcan_diffbot_demo
{

enum class AckStatus
{
  NONE,
  ACCEPTED,
  REJECTED,
  UNEXPECTED,
  IGNORED,
};

class HardwareHealth
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  void reset(const TimePoint now)
  {
    last_feedback_.fill(now);
    for (auto & pending : pending_acks_) {
      pending.clear();
    }
    ignored_ack_sequences_.fill(std::nullopt);
    last_ack_status_.fill(AckStatus::NONE);
    ack_fault_ = false;
    stop_sent_ = false;
  }

  static constexpr std::size_t kMaxOutstandingCommands = 4U;

  /// True when another in-flight command fits, so the caller can reject an
  /// overflow before it advances the sequence and puts a frame on the bus.
  /// An ambiguous reuse of a live sequence is rejected for the same reason:
  /// the ACK for the older command would be indistinguishable from the newer.
  bool can_accept_command(const std::size_t index, const uint8_t sequence) const
  {
    if (index >= pending_acks_.size() ||
      pending_acks_[index].size >= kMaxOutstandingCommands)
    {
      return false;
    }
    for (std::size_t offset = 0; offset < pending_acks_[index].size; ++offset) {
      if (pending_acks_[index].sequence(offset) == sequence) {
        return false;
      }
    }
    return true;
  }

  void command_sent(const std::size_t index, const uint8_t sequence, const TimePoint now)
  {
    if (index < pending_acks_.size()) {
      pending_acks_[index].push({sequence, now});
    }
  }

  void safe_stop_sent(const std::size_t index, const uint8_t sequence)
  {
    if (index < ignored_ack_sequences_.size()) {
      ignored_ack_sequences_[index] = sequence;
    }
  }

  AckStatus ack_received(
    const std::size_t index, const uint8_t sequence, const uint8_t result)
  {
    if (index >= pending_acks_.size()) {
      ack_fault_ = true;
      return AckStatus::UNEXPECTED;
    }

    auto & pending = pending_acks_[index];
    if (!pending.empty() && pending.front().sequence == sequence) {
      pending.pop();
      if (result == 0U) {
        last_ack_status_[index] = AckStatus::ACCEPTED;
        return AckStatus::ACCEPTED;
      }
      ack_fault_ = true;
      last_ack_status_[index] = AckStatus::REJECTED;
      return AckStatus::REJECTED;
    }

    if (ignored_ack_sequences_[index] == sequence) {
      ignored_ack_sequences_[index].reset();
      last_ack_status_[index] = AckStatus::IGNORED;
      return AckStatus::IGNORED;
    }

    ack_fault_ = true;
    last_ack_status_[index] = AckStatus::UNEXPECTED;
    return AckStatus::UNEXPECTED;
  }

  std::size_t pending_ack_count(const std::size_t index) const
  {
    return index < pending_acks_.size() ? pending_acks_[index].size : 0U;
  }

  AckStatus last_ack_status(const std::size_t index) const
  {
    return index < last_ack_status_.size() ?
           last_ack_status_[index] : AckStatus::UNEXPECTED;
  }

  bool ack_timed_out(
    const TimePoint now, const std::chrono::milliseconds timeout) const
  {
    for (std::size_t index = 0; index < pending_acks_.size(); ++index) {
      if (ack_timed_out(index, now, timeout)) {
        return true;
      }
    }
    return false;
  }

  bool ack_timed_out(
    const std::size_t index, const TimePoint now,
    const std::chrono::milliseconds timeout) const
  {
    return index < pending_acks_.size() && !pending_acks_[index].empty() &&
           now - pending_acks_[index].front().sent_at > timeout;
  }

  bool ack_fault() const
  {
    return ack_fault_;
  }

  void feedback_received(const std::size_t index, const TimePoint now)
  {
    if (index < last_feedback_.size()) {
      last_feedback_[index] = now;
    }
  }

  bool feedback_timed_out(
    const TimePoint now, const std::chrono::milliseconds timeout) const
  {
    for (std::size_t index = 0; index < last_feedback_.size(); ++index) {
      if (feedback_timed_out(index, now, timeout)) {
        return true;
      }
    }
    return false;
  }

  bool feedback_timed_out(
    const std::size_t index, const TimePoint now,
    const std::chrono::milliseconds timeout) const
  {
    return index < last_feedback_.size() && now - last_feedback_[index] > timeout;
  }

  std::chrono::milliseconds feedback_age(
    const std::size_t index, const TimePoint now) const
  {
    return index < last_feedback_.size() ?
           std::chrono::duration_cast<std::chrono::milliseconds>(now - last_feedback_[index]) :
           std::chrono::milliseconds(0);
  }

  void mark_stop_sent()
  {
    stop_sent_ = true;
  }

  bool stop_sent() const
  {
    return stop_sent_;
  }

private:
  struct PendingAck
  {
    uint8_t sequence;
    TimePoint sent_at;
  };

  // Fixed-capacity FIFO over a preallocated array: the control loop must not
  // allocate, so the window is a ring rather than a growing container.
  struct AckWindow
  {
    std::array<PendingAck, kMaxOutstandingCommands> slots{};
    std::size_t head{0U};
    std::size_t size{0U};

    bool empty() const {return size == 0U;}

    PendingAck & front() {return slots[head];}

    const PendingAck & front() const {return slots[head];}

    uint8_t sequence(const std::size_t offset) const
    {
      return slots[(head + offset) % kMaxOutstandingCommands].sequence;
    }

    void push(const PendingAck & ack)
    {
      if (size == kMaxOutstandingCommands) {
        return;
      }
      slots[(head + size) % kMaxOutstandingCommands] = ack;
      ++size;
    }

    void pop()
    {
      if (size == 0U) {
        return;
      }
      head = (head + 1U) % kMaxOutstandingCommands;
      --size;
    }

    void clear()
    {
      head = 0U;
      size = 0U;
    }
  };

  std::array<AckWindow, 2> pending_acks_;
  std::array<std::optional<uint8_t>, 2> ignored_ack_sequences_{};
  std::array<AckStatus, 2> last_ack_status_{};
  std::array<TimePoint, 2> last_feedback_{};
  bool ack_fault_{false};
  bool stop_sent_{false};
};

}  // namespace vcan_diffbot_demo

#endif  // VCAN_DIFFBOT_DEMO__HARDWARE_HEALTH_HPP_
