#include <gtest/gtest.h>

#include <linux/can/error.h>

#include "vcan_diffbot_demo/can_error_policy.hpp"

using vcan_diffbot_demo::CanErrorSeverity;
using vcan_diffbot_demo::classify_can_error;
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
