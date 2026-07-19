#ifndef VCAN_DIFFBOT_DEMO__CAN_MOTOR_HARDWARE_HPP_
#define VCAN_DIFFBOT_DEMO__CAN_MOTOR_HARDWARE_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
#include "vcan_diffbot_demo/hardware_health.hpp"

namespace vcan_diffbot_demo
{

class CanMotorHardware final : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool send_command(
    std::size_t index, bool enabled, double velocity_rad_s, bool track_ack = true);
  bool send_safe_stop();
  bool attempt_fault_safe_stop();
  void publish_diagnostics(bool force = false);

  rclcpp::Logger logger_{rclcpp::get_logger("vcan_diffbot_demo.CanMotorHardware")};
  std::string can_interface_;
  std::array<uint8_t, 2> node_ids_{};
  int32_t encoder_counts_per_revolution_{0};
  uint16_t command_watchdog_ms_{0};
  std::chrono::milliseconds feedback_timeout_{0};

  std::array<double, 2> commands_{};
  std::array<double, 2> positions_{};
  std::array<double, 2> velocities_{};
  std::array<uint8_t, 2> sequences_{};
  std::chrono::milliseconds ack_timeout_{0};
  HardwareHealth health_;
  bool fatal_fault_latched_{false};
  std::string last_can_error_;
  std::string stop_reason_{"none"};
  std::string diagnostic_state_{"inactive"};
  HardwareHealth::TimePoint last_diagnostics_publish_{};
  bool diagnostics_published_{false};
  bool diagnostics_pending_{false};

  std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;
  rclcpp::Node::SharedPtr diagnostics_node_;
  std::unique_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>
    diagnostics_publisher_;
};

}  // namespace vcan_diffbot_demo

#endif  // VCAN_DIFFBOT_DEMO__CAN_MOTOR_HARDWARE_HPP_
