#include <gtest/gtest.h>

#include <linux/can.h>
#include <linux/can/error.h>

#include <array>

#include "vcan_diffbot_demo/can_filters.hpp"

using vcan_diffbot_demo::hardware_can_filters;
using vcan_diffbot_demo::virtual_motor_can_filters;

TEST(CanFilters, HardwareAcceptsOnlyFeedbackAckAndErrors)
{
  const auto filters = hardware_can_filters({1U, 2U});
  ASSERT_EQ(filters.filters.size(), 4U);
  EXPECT_EQ(filters.filters[0].can_id, 0x181U);
  EXPECT_EQ(filters.filters[1].can_id, 0x281U);
  EXPECT_EQ(filters.filters[2].can_id, 0x182U);
  EXPECT_EQ(filters.filters[3].can_id, 0x282U);
  for (const auto & filter : filters.filters) {
    EXPECT_EQ(filter.can_mask, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG);
  }
  EXPECT_EQ(filters.error_mask, CAN_ERR_MASK);
}
TEST(CanFilters, VirtualMotorAcceptsOnlyCommands)
{
  const auto filters = virtual_motor_can_filters();
  ASSERT_EQ(filters.filters.size(), 2U);
  EXPECT_EQ(filters.filters[0].can_id, 0x101U);
  EXPECT_EQ(filters.filters[1].can_id, 0x102U);
  for (const auto & filter : filters.filters) {
    EXPECT_EQ(filter.can_mask, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG);
  }
  EXPECT_EQ(filters.error_mask, 0U);
}
