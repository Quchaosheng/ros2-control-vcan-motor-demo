#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "vcan_diffbot_demo/can_motor_hardware.hpp"

namespace
{

hardware_interface::HardwareInfo make_hardware_info()
{
  hardware_interface::HardwareInfo info;
  info.name = "TestHardware";
  info.type = "system";
  info.hardware_parameters = {
    {"can_interface", "interface_that_does_not_exist"},
    {"left_node_id", "1"},
    {"right_node_id", "2"},
    {"encoder_counts_per_revolution", "4096"},
    {"command_watchdog_ms", "200"},
    {"feedback_timeout_ms", "500"},
  };

  for (const std::string name : {"left_wheel_joint", "right_wheel_joint"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    joint.command_interfaces.resize(1);
    joint.command_interfaces[0].name = "velocity";
    joint.state_interfaces.resize(2);
    joint.state_interfaces[0].name = "position";
    joint.state_interfaces[1].name = "velocity";
    info.joints.push_back(joint);
  }
  return info;
}

}  // namespace

class CanMotorHardwareTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(CanMotorHardwareTest, LoadsThroughPluginlib)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  const auto hardware = loader.createSharedInstance("vcan_diffbot_demo/CanMotorHardware");
  ASSERT_NE(hardware, nullptr);
}

TEST_F(CanMotorHardwareTest, ClearsCommandsBeforeFailedActivation)
{
  vcan_diffbot_demo::CanMotorHardware hardware;
  ASSERT_EQ(hardware.on_init(make_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = hardware.export_command_interfaces();
  ASSERT_EQ(commands.size(), 2U);
  commands[0].set_value(3.0);
  commands[1].set_value(-4.0);

  const rclcpp_lifecycle::State state;
  EXPECT_EQ(hardware.on_activate(state), hardware_interface::CallbackReturn::ERROR);
  EXPECT_DOUBLE_EQ(commands[0].get_value(), 0.0);
  EXPECT_DOUBLE_EQ(commands[1].get_value(), 0.0);
}

TEST_F(CanMotorHardwareTest, ReportsDeactivationFailureWithoutSender)
{
  vcan_diffbot_demo::CanMotorHardware hardware;
  const rclcpp_lifecycle::State state;
  EXPECT_EQ(hardware.on_deactivate(state), hardware_interface::CallbackReturn::ERROR);
}
