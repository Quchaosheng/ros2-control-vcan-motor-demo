#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

#include "vcan_diffbot_demo/motor_state.hpp"

using vcan_diffbot_demo::MotorState;

TEST(MotorState, RejectsInvalidConfiguration)
{
  EXPECT_THROW(MotorState(0, 20.0), std::invalid_argument);
  EXPECT_THROW(MotorState(4096, 0.0), std::invalid_argument);
}

TEST(MotorState, AppliesAccelerationLimitAndIntegratesEncoder)
{
  MotorState motor(4096, 20.0);
  motor.accept_command(5, 10.0, std::chrono::milliseconds(200));
  motor.update(0.1, std::chrono::milliseconds(100));

  EXPECT_DOUBLE_EQ(motor.velocity_rad_s(), 2.0);
  EXPECT_NEAR(motor.position_rad(), 0.2, 1e-9);
  EXPECT_EQ(motor.encoder_count(), 130);
  EXPECT_EQ(motor.last_sequence(), 5);
  EXPECT_EQ(motor.status(), 0x01U);
}

TEST(MotorState, RampsDownForDisabledCommand)
{
  MotorState motor(4096, 20.0);
  motor.accept_command(1, 4.0, std::chrono::milliseconds(200));
  motor.update(0.1, std::chrono::milliseconds(100));
  motor.disable(2, std::chrono::milliseconds(200));
  motor.update(0.05, std::chrono::milliseconds(50));

  EXPECT_DOUBLE_EQ(motor.target_velocity_rad_s(), 0.0);
  EXPECT_DOUBLE_EQ(motor.velocity_rad_s(), 1.0);
  EXPECT_EQ(motor.status(), 0x00U);
  EXPECT_EQ(motor.last_sequence(), 2);
}

TEST(MotorState, WatchdogStopsTarget)
{
  MotorState motor(4096, 100.0);
  motor.accept_command(1, 5.0, std::chrono::milliseconds(200));
  motor.update(0.1, std::chrono::milliseconds(250));

  EXPECT_TRUE(motor.watchdog_stopped());
  EXPECT_DOUBLE_EQ(motor.target_velocity_rad_s(), 0.0);
  EXPECT_EQ(motor.status(), 0x03U);
}

TEST(MotorState, ExposesProtocolFaultStatus)
{
  MotorState motor(4096, 20.0);
  motor.set_protocol_fault(true);
  EXPECT_EQ(motor.status(), 0x04U);
  motor.set_protocol_fault(false);
  EXPECT_EQ(motor.status(), 0x00U);
}
