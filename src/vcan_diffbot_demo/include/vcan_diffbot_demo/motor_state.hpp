#ifndef VCAN_DIFFBOT_DEMO__MOTOR_STATE_HPP_
#define VCAN_DIFFBOT_DEMO__MOTOR_STATE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace vcan_diffbot_demo
{

class MotorState
{
public:
  MotorState(const int32_t counts_per_revolution, const double max_acceleration_rad_s2)
  : counts_per_revolution_(counts_per_revolution),
    max_acceleration_rad_s2_(max_acceleration_rad_s2)
  {
    if (counts_per_revolution_ <= 0) {
      throw std::invalid_argument("counts_per_revolution must be positive");
    }
    if (max_acceleration_rad_s2_ <= 0.0) {
      throw std::invalid_argument("max_acceleration_rad_s2 must be positive");
    }
  }

  void accept_command(
    const uint8_t sequence, const double velocity_rad_s,
    const std::chrono::milliseconds watchdog)
  {
    last_sequence_ = sequence;
    target_velocity_rad_s_ = velocity_rad_s;
    watchdog_ = watchdog;
    enabled_ = true;
    watchdog_stopped_ = false;
  }

  void disable(const uint8_t sequence, const std::chrono::milliseconds watchdog)
  {
    last_sequence_ = sequence;
    target_velocity_rad_s_ = 0.0;
    watchdog_ = watchdog;
    enabled_ = false;
    watchdog_stopped_ = false;
  }

  void update(
    const double dt_seconds, const std::chrono::milliseconds since_last_command)
  {
    if (enabled_ && since_last_command > watchdog_) {
      target_velocity_rad_s_ = 0.0;
      watchdog_stopped_ = true;
    }

    if (dt_seconds <= 0.0) {
      return;
    }

    const double max_delta = max_acceleration_rad_s2_ * dt_seconds;
    const double delta = std::clamp(
      target_velocity_rad_s_ - velocity_rad_s_, -max_delta, max_delta);
    velocity_rad_s_ += delta;
    position_rad_ += velocity_rad_s_ * dt_seconds;
  }

  double target_velocity_rad_s() const
  {
    return target_velocity_rad_s_;
  }

  double velocity_rad_s() const
  {
    return velocity_rad_s_;
  }

  double position_rad() const
  {
    return position_rad_;
  }

  int32_t encoder_count() const
  {
    constexpr double kTwoPi = 6.28318530717958647692;
    const double count = std::round(
      position_rad_ * static_cast<double>(counts_per_revolution_) / kTwoPi);
    const double clamped = std::clamp(
      count,
      static_cast<double>(std::numeric_limits<int32_t>::min()),
      static_cast<double>(std::numeric_limits<int32_t>::max()));
    return static_cast<int32_t>(clamped);
  }

  uint8_t last_sequence() const
  {
    return last_sequence_;
  }

  uint8_t status() const
  {
    uint8_t status = enabled_ ? 0x01U : 0x00U;
    if (watchdog_stopped_) {
      status |= 0x02U;
    }
    if (protocol_fault_) {
      status |= 0x04U;
    }
    return status;
  }

  bool watchdog_stopped() const
  {
    return watchdog_stopped_;
  }

  void set_protocol_fault(const bool fault)
  {
    protocol_fault_ = fault;
  }

private:
  int32_t counts_per_revolution_;
  double max_acceleration_rad_s2_;
  double target_velocity_rad_s_{0.0};
  double velocity_rad_s_{0.0};
  double position_rad_{0.0};
  std::chrono::milliseconds watchdog_{0};
  uint8_t last_sequence_{0};
  bool enabled_{false};
  bool watchdog_stopped_{false};
  bool protocol_fault_{false};
};

}  // namespace vcan_diffbot_demo

#endif  // VCAN_DIFFBOT_DEMO__MOTOR_STATE_HPP_
