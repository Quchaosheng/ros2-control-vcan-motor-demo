#ifndef VCAN_DIFFBOT_DEMO__CAN_ERROR_POLICY_HPP_
#define VCAN_DIFFBOT_DEMO__CAN_ERROR_POLICY_HPP_

#include <cstdint>
#include <string>

#include <linux/can/error.h>

namespace vcan_diffbot_demo
{

enum class CanErrorSeverity
{
  NONE,
  WARNING,
  FATAL,
};

inline CanErrorSeverity classify_can_error(const uint32_t error_mask)
{
  if ((error_mask & (CAN_ERR_BUSOFF | CAN_ERR_TX_TIMEOUT)) != 0U) {
    return CanErrorSeverity::FATAL;
  }
  return error_mask == 0U ? CanErrorSeverity::NONE : CanErrorSeverity::WARNING;
}

inline std::string describe_can_controller_state(const uint8_t status_byte)
{
  struct StateName
  {
    uint8_t bit;
    const char * name;
  };
  constexpr StateName state_names[] = {
    {CAN_ERR_CRTL_RX_OVERFLOW, "rx_overflow"},
    {CAN_ERR_CRTL_TX_OVERFLOW, "tx_overflow"},
    {CAN_ERR_CRTL_RX_WARNING, "rx_warning"},
    {CAN_ERR_CRTL_TX_WARNING, "tx_warning"},
    {CAN_ERR_CRTL_RX_PASSIVE, "rx_passive"},
    {CAN_ERR_CRTL_TX_PASSIVE, "tx_passive"},
    {CAN_ERR_CRTL_ACTIVE, "active"},
  };

  std::string description;
  for (const auto & state_name : state_names) {
    if ((status_byte & state_name.bit) == 0U) {
      continue;
    }
    if (!description.empty()) {
      description += ',';
    }
    description += state_name.name;
  }
  return description.empty() && status_byte != 0U ? "unknown" : description;
}

inline std::string describe_can_error(const uint32_t error_mask)
{
  struct ErrorName
  {
    uint32_t bit;
    const char * name;
  };
  constexpr ErrorName error_names[] = {
    {CAN_ERR_TX_TIMEOUT, "tx_timeout"},
    {CAN_ERR_LOSTARB, "lost_arbitration"},
    {CAN_ERR_CRTL, "controller"},
    {CAN_ERR_PROT, "protocol"},
    {CAN_ERR_TRX, "transceiver"},
    {CAN_ERR_ACK, "ack"},
    {CAN_ERR_BUSOFF, "bus_off"},
    {CAN_ERR_BUSERROR, "bus_error"},
    {CAN_ERR_RESTARTED, "restarted"},
  };

  std::string description;
  for (const auto & error_name : error_names) {
    if ((error_mask & error_name.bit) == 0U) {
      continue;
    }
    if (!description.empty()) {
      description += ',';
    }
    description += error_name.name;
  }
  return description.empty() && error_mask != 0U ? "unknown" : description;
}

}  // namespace vcan_diffbot_demo

#endif  // VCAN_DIFFBOT_DEMO__CAN_ERROR_POLICY_HPP_
