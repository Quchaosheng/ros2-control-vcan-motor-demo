#include <gtest/gtest.h>

#include <linux/can/error.h>

#include "vcan_diffbot_demo/can_error_policy.hpp"

using vcan_diffbot_demo::CanErrorSeverity;
using vcan_diffbot_demo::classify_can_error;
using vcan_diffbot_demo::describe_can_controller_state;
using vcan_diffbot_demo::describe_can_error;

TEST(CanErrorPolicy, ClassifiesSocketCanErrorMasks)
{
  EXPECT_EQ(classify_can_error(0U), CanErrorSeverity::NONE);
  EXPECT_EQ(classify_can_error(CAN_ERR_CRTL), CanErrorSeverity::WARNING);
  EXPECT_EQ(
    classify_can_error(CAN_ERR_ACK | CAN_ERR_PROT),
    CanErrorSeverity::WARNING);
  EXPECT_EQ(classify_can_error(CAN_ERR_BUSOFF), CanErrorSeverity::FATAL);
  EXPECT_EQ(classify_can_error(CAN_ERR_TX_TIMEOUT), CanErrorSeverity::FATAL);
  EXPECT_EQ(
    classify_can_error(CAN_ERR_CRTL | CAN_ERR_BUSOFF),
    CanErrorSeverity::FATAL);
}

TEST(CanErrorPolicy, DescribesKnownBitsInStableOrder)
{
  EXPECT_EQ(describe_can_error(CAN_ERR_BUSOFF), "bus_off");
  EXPECT_EQ(describe_can_error(CAN_ERR_CRTL), "controller");
  EXPECT_EQ(
    describe_can_error(
      CAN_ERR_RESTARTED | CAN_ERR_BUSERROR | CAN_ERR_BUSOFF | CAN_ERR_ACK |
      CAN_ERR_TRX | CAN_ERR_PROT | CAN_ERR_CRTL | CAN_ERR_LOSTARB |
      CAN_ERR_TX_TIMEOUT),
    "tx_timeout,lost_arbitration,controller,protocol,transceiver,ack,bus_off,bus_error,"
    "restarted");
  EXPECT_EQ(describe_can_error(0x40000000U), "unknown");
}

TEST(CanErrorPolicy, DescribesControllerStateBits)
{
  EXPECT_EQ(describe_can_controller_state(0U), "");
  EXPECT_EQ(
    describe_can_controller_state(CAN_ERR_CRTL_RX_OVERFLOW), "rx_overflow");
  EXPECT_EQ(
    describe_can_controller_state(CAN_ERR_CRTL_TX_OVERFLOW), "tx_overflow");
  EXPECT_EQ(
    describe_can_controller_state(CAN_ERR_CRTL_RX_WARNING), "rx_warning");
  EXPECT_EQ(
    describe_can_controller_state(CAN_ERR_CRTL_TX_WARNING), "tx_warning");
  EXPECT_EQ(
    describe_can_controller_state(CAN_ERR_CRTL_RX_PASSIVE), "rx_passive");
  EXPECT_EQ(
    describe_can_controller_state(CAN_ERR_CRTL_TX_PASSIVE), "tx_passive");
  EXPECT_EQ(describe_can_controller_state(CAN_ERR_CRTL_ACTIVE), "active");
}

TEST(CanErrorPolicy, DescribesControllerStateInStableOrder)
{
  EXPECT_EQ(
    describe_can_controller_state(
      CAN_ERR_CRTL_ACTIVE | CAN_ERR_CRTL_TX_PASSIVE |
      CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_WARNING |
      CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_OVERFLOW |
      CAN_ERR_CRTL_RX_OVERFLOW),
    "rx_overflow,tx_overflow,rx_warning,tx_warning,rx_passive,tx_passive,"
    "active");
  EXPECT_EQ(
    describe_can_controller_state(
      CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE),
    "rx_passive,tx_passive");
}

TEST(CanErrorPolicy, DistinguishesPassiveFromWarning)
{
  const auto warning =
    describe_can_controller_state(CAN_ERR_CRTL_RX_WARNING);
  const auto passive =
    describe_can_controller_state(CAN_ERR_CRTL_RX_PASSIVE);
  EXPECT_NE(warning, passive);
  EXPECT_EQ(describe_can_controller_state(0x80U), "unknown");
}
