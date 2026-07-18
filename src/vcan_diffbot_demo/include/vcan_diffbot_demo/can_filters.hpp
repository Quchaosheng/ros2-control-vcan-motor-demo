#ifndef VCAN_DIFFBOT_DEMO__CAN_FILTERS_HPP_
#define VCAN_DIFFBOT_DEMO__CAN_FILTERS_HPP_

#include <linux/can.h>
#include <linux/can/error.h>

#include <array>
#include <cstdint>

#include "ros2_socketcan/socket_can_receiver.hpp"
#include "vcan_diffbot_demo/can_protocol.hpp"

namespace vcan_diffbot_demo
{

inline can_filter exact_standard_filter(const uint32_t identifier)
{
  return can_filter{
    identifier,
    CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG,
  };
}

inline drivers::socketcan::SocketCanReceiver::CanFilterList hardware_can_filters(
  const std::array<uint8_t, 2> node_ids)
{
  drivers::socketcan::SocketCanReceiver::CanFilterList filters;
  for (const auto node_id : node_ids) {
    filters.filters.push_back(exact_standard_filter(protocol::feedback_id(node_id)));
    filters.filters.push_back(exact_standard_filter(protocol::ack_id(node_id)));
  }
  filters.error_mask = CAN_ERR_MASK;
  return filters;
}

inline drivers::socketcan::SocketCanReceiver::CanFilterList virtual_motor_can_filters()
{
  drivers::socketcan::SocketCanReceiver::CanFilterList filters;
  filters.filters.push_back(exact_standard_filter(protocol::command_id(1U)));
  filters.filters.push_back(exact_standard_filter(protocol::command_id(2U)));
  return filters;
}

}  // namespace vcan_diffbot_demo

#endif  // VCAN_DIFFBOT_DEMO__CAN_FILTERS_HPP_
