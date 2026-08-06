// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from altair_interfaces:srv/GetAvailablePorts.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__GET_AVAILABLE_PORTS__BUILDER_HPP_
#define ALTAIR_INTERFACES__SRV__DETAIL__GET_AVAILABLE_PORTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "altair_interfaces/srv/detail/get_available_ports__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace altair_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::altair_interfaces::srv::GetAvailablePorts_Request>()
{
  return ::altair_interfaces::srv::GetAvailablePorts_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace altair_interfaces


namespace altair_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetAvailablePorts_Response_ports
{
public:
  Init_GetAvailablePorts_Response_ports()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::altair_interfaces::srv::GetAvailablePorts_Response ports(::altair_interfaces::srv::GetAvailablePorts_Response::_ports_type arg)
  {
    msg_.ports = std::move(arg);
    return std::move(msg_);
  }

private:
  ::altair_interfaces::srv::GetAvailablePorts_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::altair_interfaces::srv::GetAvailablePorts_Response>()
{
  return altair_interfaces::srv::builder::Init_GetAvailablePorts_Response_ports();
}

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__GET_AVAILABLE_PORTS__BUILDER_HPP_
