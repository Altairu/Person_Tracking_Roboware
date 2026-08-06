// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from altair_interfaces:msg/CanStatus.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__CAN_STATUS__BUILDER_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__CAN_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "altair_interfaces/msg/detail/can_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace altair_interfaces
{

namespace msg
{

namespace builder
{

class Init_CanStatus_error_message
{
public:
  explicit Init_CanStatus_error_message(::altair_interfaces::msg::CanStatus & msg)
  : msg_(msg)
  {}
  ::altair_interfaces::msg::CanStatus error_message(::altair_interfaces::msg::CanStatus::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::altair_interfaces::msg::CanStatus msg_;
};

class Init_CanStatus_bitrate
{
public:
  explicit Init_CanStatus_bitrate(::altair_interfaces::msg::CanStatus & msg)
  : msg_(msg)
  {}
  Init_CanStatus_error_message bitrate(::altair_interfaces::msg::CanStatus::_bitrate_type arg)
  {
    msg_.bitrate = std::move(arg);
    return Init_CanStatus_error_message(msg_);
  }

private:
  ::altair_interfaces::msg::CanStatus msg_;
};

class Init_CanStatus_active_port
{
public:
  explicit Init_CanStatus_active_port(::altair_interfaces::msg::CanStatus & msg)
  : msg_(msg)
  {}
  Init_CanStatus_bitrate active_port(::altair_interfaces::msg::CanStatus::_active_port_type arg)
  {
    msg_.active_port = std::move(arg);
    return Init_CanStatus_bitrate(msg_);
  }

private:
  ::altair_interfaces::msg::CanStatus msg_;
};

class Init_CanStatus_is_connected
{
public:
  Init_CanStatus_is_connected()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CanStatus_active_port is_connected(::altair_interfaces::msg::CanStatus::_is_connected_type arg)
  {
    msg_.is_connected = std::move(arg);
    return Init_CanStatus_active_port(msg_);
  }

private:
  ::altair_interfaces::msg::CanStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::altair_interfaces::msg::CanStatus>()
{
  return altair_interfaces::msg::builder::Init_CanStatus_is_connected();
}

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__CAN_STATUS__BUILDER_HPP_
