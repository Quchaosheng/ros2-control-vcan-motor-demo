#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_id.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
#include "vcan_diffbot_demo/can_filters.hpp"
#include "vcan_diffbot_demo/can_protocol.hpp"
#include "vcan_diffbot_demo/motor_state.hpp"

namespace vcan_diffbot_demo
{
namespace
{

constexpr auto kCanTimeout = std::chrono::milliseconds(10);
constexpr auto kSendTimeout = std::chrono::milliseconds(1);

bool every_n(const int64_t interval, const uint64_t count)
{
  return interval > 0 && count % static_cast<uint64_t>(interval) == 0U;
}

std::chrono::nanoseconds period_from_rate(const double rate_hz, const std::string & name)
{
  if (rate_hz <= 0.0) {
    throw std::invalid_argument(name + " must be positive");
  }
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate_hz));
}

}  // namespace

class VirtualMotorNode final : public rclcpp::Node
{
public:
  VirtualMotorNode()
  : Node("virtual_motor")
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "vcan0");
    const auto encoder_counts = declare_parameter<int64_t>(
      "encoder_counts_per_revolution", 4096);
    const double max_acceleration = declare_parameter<double>(
      "max_acceleration_rad_s2", 20.0);
    const double update_rate = declare_parameter<double>("update_rate_hz", 100.0);
    const double feedback_rate = declare_parameter<double>("feedback_rate_hz", 50.0);
    drop_command_every_n_ = nonnegative_parameter("drop_command_every_n", 0);
    drop_feedback_every_n_ = nonnegative_parameter("drop_feedback_every_n", 0);
    feedback_delay_ms_ = nonnegative_parameter("feedback_delay_ms", 0);
    malformed_feedback_every_n_ = nonnegative_parameter("malformed_feedback_every_n", 0);
    error_frame_every_n_ = nonnegative_parameter("error_frame_every_n", 0);

    if (can_interface_.empty()) {
      throw std::invalid_argument("can_interface must not be empty");
    }
    if (encoder_counts <= 0 || encoder_counts > std::numeric_limits<int32_t>::max()) {
      throw std::invalid_argument("encoder_counts_per_revolution is invalid");
    }

    motors_[0] = std::make_unique<MotorState>(
      static_cast<int32_t>(encoder_counts), max_acceleration);
    motors_[1] = std::make_unique<MotorState>(
      static_cast<int32_t>(encoder_counts), max_acceleration);

    sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface_);
    receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_interface_);
    receiver_->SetCanFilters(virtual_motor_can_filters());

    const auto now = std::chrono::steady_clock::now();
    last_command_.fill(now);
    last_update_ = now;

    update_timer_ = create_wall_timer(
      period_from_rate(update_rate, "update_rate_hz"),
      std::bind(&VirtualMotorNode::update, this));
    feedback_timer_ = create_wall_timer(
      period_from_rate(feedback_rate, "feedback_rate_hz"),
      std::bind(&VirtualMotorNode::publish_feedback, this));

    running_.store(true);
    receiver_thread_ = std::thread(&VirtualMotorNode::receive_loop, this);
    RCLCPP_INFO(get_logger(), "Virtual motors listening on %s", can_interface_.c_str());
  }

  ~VirtualMotorNode() override
  {
    running_.store(false);
    if (receiver_thread_.joinable()) {
      receiver_thread_.join();
    }
  }

private:
  struct DelayedFrame
  {
    protocol::FrameData data;
    drivers::socketcan::CanId id;
    std::size_t length;
    std::chrono::steady_clock::time_point due;
  };

  int64_t nonnegative_parameter(const std::string & name, const int64_t default_value)
  {
    const auto value = declare_parameter<int64_t>(name, default_value);
    if (value < 0) {
      throw std::invalid_argument(name + " must not be negative");
    }
    return value;
  }

  void receive_loop()
  {
    while (running_.load()) {
      protocol::FrameData data{};
      try {
        const auto can_id = receiver_->receive(data.data(), kCanTimeout);
        if (can_id.frame_type() != drivers::socketcan::FrameType::DATA) {
          continue;
        }

        std::size_t index = motors_.size();
        if (can_id.identifier() == protocol::command_id(1U)) {
          index = 0U;
        } else if (can_id.identifier() == protocol::command_id(2U)) {
          index = 1U;
        } else {
          continue;
        }

        ++command_count_;
        if (every_n(drop_command_every_n_, command_count_)) {
          continue;
        }

        const auto command = protocol::decode_command(data, can_id.length());
        if (!command) {
          {
            std::lock_guard<std::mutex> lock(motor_mutex_);
            motors_[index]->set_protocol_fault(true);
          }
          send_ack(index, data[0], can_id.length() == 8U ? 2U : 1U);
          continue;
        }

        {
          std::lock_guard<std::mutex> lock(motor_mutex_);
          motors_[index]->set_protocol_fault(false);
          const auto watchdog = std::chrono::milliseconds(command->watchdog_ms);
          if (command->enabled) {
            motors_[index]->accept_command(
              command->sequence,
              static_cast<double>(command->velocity_mrad_s) / 1000.0,
              watchdog);
          } else {
            motors_[index]->disable(command->sequence, watchdog);
          }
          last_command_[index] = std::chrono::steady_clock::now();
        }
        send_ack(index, command->sequence, 0U);
      } catch (const drivers::socketcan::SocketCanTimeout &) {
        continue;
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "CAN receive loop stopped: %s", error.what());
        running_.store(false);
      }
    }
  }

  void update()
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_update_).count();
    last_update_ = now;
    {
      std::lock_guard<std::mutex> lock(motor_mutex_);
      for (std::size_t index = 0; index < motors_.size(); ++index) {
        motors_[index]->update(
          dt,
          std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_[index]));
      }
    }

    while (!delayed_frames_.empty() && delayed_frames_.front().due <= now) {
      const auto frame = std::move(delayed_frames_.front());
      delayed_frames_.pop_front();
      send_frame(frame.data, frame.id, frame.length);
    }
  }

  void publish_feedback()
  {
    std::array<protocol::Feedback, 2> feedback{};
    {
      std::lock_guard<std::mutex> lock(motor_mutex_);
      for (std::size_t index = 0; index < motors_.size(); ++index) {
        const double raw_velocity = std::round(motors_[index]->velocity_rad_s() * 1000.0);
        const double clamped_velocity = std::clamp(
          raw_velocity,
          static_cast<double>(std::numeric_limits<int16_t>::min()),
          static_cast<double>(std::numeric_limits<int16_t>::max()));
        feedback[index] = protocol::Feedback{
          motors_[index]->last_sequence(),
          motors_[index]->status(),
          static_cast<int16_t>(clamped_velocity),
          motors_[index]->encoder_count()};
      }
    }

    for (std::size_t index = 0; index < feedback.size(); ++index) {
      ++feedback_count_;
      if (every_n(drop_feedback_every_n_, feedback_count_)) {
        continue;
      }

      if (every_n(error_frame_every_n_, feedback_count_)) {
        protocol::FrameData error_data{};
        const drivers::socketcan::CanId error_id(
          0x04U, 0U, drivers::socketcan::FrameType::ERROR,
          drivers::socketcan::StandardFrame);
        send_frame(error_data, error_id, error_data.size());
      }

      const auto data = protocol::encode_feedback(feedback[index]);
      const drivers::socketcan::CanId can_id(
        protocol::feedback_id(static_cast<uint8_t>(index + 1U)), 0U,
        drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
      const std::size_t length = every_n(malformed_feedback_every_n_, feedback_count_) ? 7U : 8U;
      if (feedback_delay_ms_ == 0) {
        send_frame(data, can_id, length);
      } else {
        delayed_frames_.push_back(DelayedFrame{
          data, can_id, length,
          std::chrono::steady_clock::now() + std::chrono::milliseconds(feedback_delay_ms_)});
      }
    }
  }

  void send_ack(const std::size_t index, const uint8_t sequence, const uint8_t result)
  {
    const auto data = protocol::encode_ack({sequence, result});
    const drivers::socketcan::CanId can_id(
      protocol::ack_id(static_cast<uint8_t>(index + 1U)), 0U,
      drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
    send_frame(data, can_id, data.size());
  }

  void send_frame(
    const protocol::FrameData & data, const drivers::socketcan::CanId & can_id,
    const std::size_t length)
  {
    try {
      std::lock_guard<std::mutex> lock(sender_mutex_);
      sender_->send(data.data(), length, can_id, kSendTimeout);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "CAN send failed: %s", error.what());
    }
  }

  std::string can_interface_;
  std::array<std::unique_ptr<MotorState>, 2> motors_;
  std::array<std::chrono::steady_clock::time_point, 2> last_command_{};
  std::chrono::steady_clock::time_point last_update_{};

  int64_t drop_command_every_n_{0};
  int64_t drop_feedback_every_n_{0};
  int64_t feedback_delay_ms_{0};
  int64_t malformed_feedback_every_n_{0};
  int64_t error_frame_every_n_{0};
  uint64_t command_count_{0};
  uint64_t feedback_count_{0};

  std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;
  std::atomic<bool> running_{false};
  std::thread receiver_thread_;
  std::mutex motor_mutex_;
  std::mutex sender_mutex_;
  std::deque<DelayedFrame> delayed_frames_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
};

}  // namespace vcan_diffbot_demo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<vcan_diffbot_demo::VirtualMotorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("virtual_motor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
