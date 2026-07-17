#include <gtest/gtest.h>

#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

TEST(CanMotorHardware, LoadsThroughPluginlib)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  const auto hardware = loader.createSharedInstance("vcan_diffbot_demo/CanMotorHardware");
  ASSERT_NE(hardware, nullptr);
}
