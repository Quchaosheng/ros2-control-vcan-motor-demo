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

  /** How many commands may be in flight per motor.
   *
   * The sequence number is 8 bits, so an unbounded queue would eventually hold
   * two entries with the same sequence and an ACK could not be attributed to
   * either. Eight is far below that and is a number the write path can be held
   * to: the control loop never has more than one command outstanding per motor
   * unless ACKs are being lost, and a lost ACK is a fault, not a backlog.
   */
  static constexpr std::size_t kMaxOutstandingAcks = 8U;

  /** Whether a command with this sequence may be tracked at all.
   *
   * False when the window is full, and false when the sequence is already in
   * flight: reusing one inside the window makes the next ACK ambiguous, so the
   * caller must not send it rather than discover that afterwards.
   */
  bool can_track(const std::size_t index, const uint8_t sequence) const
  {
    if (index >= pending_acks_.size()) {
      return false;
    }
    const auto & window = pending_acks_[index];
    return window.size() < kMaxOutstandingAcks && !window.contains(sequence);
  }

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

  /** Reserve a slot for a command that has just been sent.
   *
   * Returns false when it was refused. The caller is expected to have asked
   * can_track() before sending; this repeats the check so a caller that did not
   * cannot grow the window past its bound.
   */
  bool command_sent(const std::size_t index, const uint8_t sequence, const TimePoint now)
  {
    if (!can_track(index, sequence)) {
      return false;
    }
    pending_acks_[index].push({sequence, now});
    return true;
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
    return index < pending_acks_.size() ? pending_acks_[index].size() : 0U;
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

  /** Fixed-capacity ring of in-flight commands.
   *
   * A ring rather than a deque because command_sent() runs in the write path,
   * and a deque allocates on push. Capacity is a compile-time constant, so
   * push and pop are index arithmetic on storage that already exists.
   */
  class AckWindow
  {
public:
    void clear()
    {
      head_ = 0;
      size_ = 0;
    }

    [[nodiscard]] bool empty() const {return size_ == 0U;}
    [[nodiscard]] std::size_t size() const {return size_;}

    [[nodiscard]] const PendingAck & front() const {return slots_[head_];}

    [[nodiscard]] bool contains(const uint8_t sequence) const
    {
      for (std::size_t offset = 0; offset < size_; ++offset) {
        if (slots_[(head_ + offset) % kMaxOutstandingAcks].sequence == sequence) {
          return true;
        }
      }
      return false;
    }

    void push(const PendingAck & pending)
    {
      slots_[(head_ + size_) % kMaxOutstandingAcks] = pending;
      ++size_;
    }

    void pop()
    {
      head_ = (head_ + 1U) % kMaxOutstandingAcks;
      --size_;
    }

private:
    std::array<PendingAck, kMaxOutstandingAcks> slots_{};
    std::size_t head_{0};
    std::size_t size_{0};
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
