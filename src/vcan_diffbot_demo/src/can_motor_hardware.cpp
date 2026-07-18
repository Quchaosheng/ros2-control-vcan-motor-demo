#include "vcan_diffbot_demo/can_motor_hardware.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_id.hpp"
#include "vcan_diffbot_demo/can_error_policy.hpp"
#include "vcan_diffbot_demo/can_filters.hpp"
#include "vcan_diffbot_demo/can_protocol.hpp"

namespace vcan_diffbot_demo
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;
constexpr auto kSendTimeout = std::chrono::milliseconds(1);
constexpr auto kReceivePollTimeout = std::chrono::microseconds(50);

long parse_positive(const std::string & value, const std::string & name)
{
  std::size_t parsed = 0;
  const long result = std::stol(value, &parsed, 10);
  if (parsed != value.size() || result <= 0) {
    throw std::invalid_argument(name + " must be a positive integer");
  }
  return result;
}

void validate_joint(const hardware_interface::ComponentInfo & joint)
{
  if (joint.command_interfaces.size() != 1U ||
    joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
  {
    throw std::invalid_argument(joint.name + " must expose one velocity command interface");
  }
  if (joint.state_interfaces.size() != 2U ||
    joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
    joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
  {
    throw std::invalid_argument(
            joint.name + " must expose position then velocity state interfaces");
  }
}

}  // namespace

hardware_interface::CallbackReturn CanMotorHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    if (info_.joints.size() != 2U) {
      throw std::invalid_argument("exactly two wheel joints are required");
    }
    validate_joint(info_.joints[0]);
    validate_joint(info_.joints[1]);

    can_interface_ = info_.hardware_parameters.at("can_interface");
    if (can_interface_.empty()) {
      throw std::invalid_argument("can_interface must not be empty");
    }

    const long left_node = parse_positive(
      info_.hardware_parameters.at("left_node_id"), "left_node_id");
    const long right_node = parse_positive(
      info_.hardware_parameters.at("right_node_id"), "right_node_id");
    if (left_node > 127 || right_node > 127 || left_node == right_node) {
      throw std::invalid_argument("motor node IDs must be distinct values from 1 to 127");
    }
    node_ids_ = {static_cast<uint8_t>(left_node), static_cast<uint8_t>(right_node)};

    const long encoder_counts = parse_positive(
      info_.hardware_parameters.at("encoder_counts_per_revolution"),
      "encoder_counts_per_revolution");
    if (encoder_counts > std::numeric_limits<int32_t>::max()) {
      throw std::invalid_argument("encoder_counts_per_revolution is too large");
    }
    encoder_counts_per_revolution_ = static_cast<int32_t>(encoder_counts);

    const long watchdog = parse_positive(
      info_.hardware_parameters.at("command_watchdog_ms"), "command_watchdog_ms");
    if (watchdog > std::numeric_limits<uint16_t>::max()) {
      throw std::invalid_argument("command_watchdog_ms is too large");
    }
    command_watchdog_ms_ = static_cast<uint16_t>(watchdog);
    ack_timeout_ = std::chrono::milliseconds(command_watchdog_ms_);

    feedback_timeout_ = std::chrono::milliseconds(parse_positive(
        info_.hardware_parameters.at("feedback_timeout_ms"), "feedback_timeout_ms"));
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "Invalid hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  commands_.fill(0.0);
  positions_.fill(0.0);
  velocities_.fill(0.0);
  sequences_.fill(0U);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CanMotorHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(4U);
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &positions_[index]);
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &velocities_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> CanMotorHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(2U);
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &commands_[index]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn CanMotorHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  commands_.fill(0.0);
  try {
    sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface_);
    receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_interface_);
    receiver_->SetCanFilters(hardware_can_filters(node_ids_));
    const auto now = std::chrono::steady_clock::now();
    health_.reset(now);
    last_can_error_.clear();
    stop_reason_.clear();
    if (!send_safe_stop()) {
      throw std::runtime_error("failed to send activation safe stop");
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "Failed to activate CAN hardware on %s: %s",
      can_interface_.c_str(), error.what());
    sender_.reset();
    receiver_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "CAN motor hardware active on %s", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanMotorHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  commands_.fill(0.0);
  const bool stopped = send_safe_stop();
  receiver_.reset();
  sender_.reset();
  return stopped ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type CanMotorHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!receiver_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    for (std::size_t count = 0; count < 64U; ++count) {
      protocol::FrameData data{};
      drivers::socketcan::CanId can_id;
      try {
        can_id = receiver_->receive(data.data(), kReceivePollTimeout);
      } catch (const drivers::socketcan::SocketCanTimeout &) {
        break;
      }

      if (can_id.frame_type() == drivers::socketcan::FrameType::ERROR) {
        const uint32_t error_mask = can_id.identifier() & CAN_ERR_MASK;
        const auto severity = classify_can_error(error_mask);
        if (severity == CanErrorSeverity::NONE) {
          continue;
        }

        last_can_error_ = describe_can_error(error_mask);
        if (severity == CanErrorSeverity::WARNING) {
          RCLCPP_WARN(logger_, "SocketCAN warning: %s", last_can_error_.c_str());
          continue;
        }

        stop_reason_ = (error_mask & CAN_ERR_BUSOFF) != 0U ?
          "can_bus_off" : "can_tx_timeout";
        RCLCPP_ERROR(logger_, "Fatal SocketCAN error: %s", last_can_error_.c_str());
        if (!health_.stop_sent() && send_safe_stop()) {
          health_.mark_stop_sent();
        }
        return hardware_interface::return_type::ERROR;
      }
      if (can_id.frame_type() != drivers::socketcan::FrameType::DATA) {
        continue;
      }

      for (std::size_t index = 0; index < node_ids_.size(); ++index) {
        if (can_id.identifier() == protocol::feedback_id(node_ids_[index])) {
          const auto feedback = protocol::decode_feedback(data, can_id.length());
          if (!feedback) {
            RCLCPP_WARN(logger_, "Ignoring malformed feedback for node %u", node_ids_[index]);
            break;
          }
          velocities_[index] = static_cast<double>(feedback->velocity_mrad_s) / 1000.0;
          positions_[index] = static_cast<double>(feedback->encoder_count) * kTwoPi /
            static_cast<double>(encoder_counts_per_revolution_);
          health_.feedback_received(index, std::chrono::steady_clock::now());
          if ((feedback->status & 0x04U) != 0U) {
            RCLCPP_WARN(logger_, "Motor node %u reports a protocol fault", node_ids_[index]);
          }
          break;
        }

        if (can_id.identifier() == protocol::ack_id(node_ids_[index])) {
          const auto ack = protocol::decode_ack(data, can_id.length());
          if (!ack) {
            RCLCPP_WARN(logger_, "Ignoring malformed ACK for node %u", node_ids_[index]);
            break;
          }
          const auto ack_status = health_.ack_received(index, ack->sequence, ack->result);
          if (ack_status == AckStatus::REJECTED) {
            RCLCPP_WARN(logger_, "Motor node %u rejected command %u with result %u",
              node_ids_[index], ack->sequence, ack->result);
          } else if (ack_status == AckStatus::UNEXPECTED) {
            RCLCPP_WARN(logger_, "Motor node %u returned unexpected ACK sequence %u",
              node_ids_[index], ack->sequence);
          }
          break;
        }
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "SocketCAN receive failed: %s", error.what());
    send_safe_stop();
    return hardware_interface::return_type::ERROR;
  }

  const auto now = std::chrono::steady_clock::now();
  health_.recover_if_healthy(now, feedback_timeout_, ack_timeout_);
  bool fault = health_.ack_fault();
  for (std::size_t index = 0; index < node_ids_.size(); ++index) {
    if (health_.feedback_timed_out(index, now, feedback_timeout_)) {
      fault = true;
      if (!health_.stop_sent()) {
        RCLCPP_ERROR(logger_, "Feedback timeout for motor node %u", node_ids_[index]);
      }
    }
    if (health_.ack_timed_out(index, now, ack_timeout_)) {
      fault = true;
      if (!health_.stop_sent()) {
        RCLCPP_ERROR(logger_, "ACK timeout for motor node %u", node_ids_[index]);
      }
    }
  }

  if (fault) {
    if (!health_.stop_sent() && send_safe_stop()) {
      health_.mark_stop_sent();
    }
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanMotorHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!sender_) {
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t index = 0; index < commands_.size(); ++index) {
    if (!send_command(index, true, commands_[index])) {
      send_safe_stop();
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

bool CanMotorHardware::send_command(
  const std::size_t index, const bool enabled, const double velocity_rad_s,
  const bool track_ack)
{
  if (!sender_ || index >= node_ids_.size()) {
    return false;
  }

  try {
    const double clamped_velocity_mrad_s = std::clamp(
      velocity_rad_s * 1000.0,
      static_cast<double>(std::numeric_limits<int16_t>::min()),
      static_cast<double>(std::numeric_limits<int16_t>::max()));
    const int32_t velocity_mrad_s = static_cast<int32_t>(std::llround(clamped_velocity_mrad_s));
    const auto data = protocol::encode_command(
      ++sequences_[index], enabled, velocity_mrad_s, command_watchdog_ms_);
    const drivers::socketcan::CanId can_id(
      protocol::command_id(node_ids_[index]), 0U,
      drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
    sender_->send(data, can_id, kSendTimeout);
    if (track_ack) {
      health_.command_sent(index, sequences_[index], std::chrono::steady_clock::now());
    } else {
      health_.safe_stop_sent(index, sequences_[index]);
    }
    return true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "Failed to send command to motor node %u: %s",
      node_ids_[index], error.what());
    return false;
  }
}

bool CanMotorHardware::send_safe_stop()
{
  if (!sender_) {
    return false;
  }
  bool success = true;
  for (std::size_t index = 0; index < node_ids_.size(); ++index) {
    success = send_command(index, false, 0.0, false) && success;
  }
  return success;
}

}  // namespace vcan_diffbot_demo

PLUGINLIB_EXPORT_CLASS(
  vcan_diffbot_demo::CanMotorHardware, hardware_interface::SystemInterface)
