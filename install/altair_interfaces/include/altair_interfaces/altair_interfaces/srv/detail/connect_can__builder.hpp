// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from altair_interfaces:srv/ConnectCan.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__BUILDER_HPP_
#define ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "altair_interfaces/srv/detail/connect_can__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace altair_interfaces
{

namespace srv
{

namespace builder
{

class Init_ConnectCan_Request_auto_connect
{
public:
  explicit Init_ConnectCan_Request_auto_connect(::altair_interfaces::srv::ConnectCan_Request & msg)
  : msg_(msg)
  {}
  ::altair_interfaces::srv::ConnectCan_Request auto_connect(::altair_interfaces::srv::ConnectCan_Request::_auto_connect_type arg)
  {
    msg_.auto_connect = std::move(arg);
    return std::move(msg_);
  }

private:
  ::altair_interfaces::srv::ConnectCan_Request msg_;
};

class Init_ConnectCan_Request_port
{
public:
  Init_ConnectCan_Request_port()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConnectCan_Request_auto_connect port(::altair_interfaces::srv::ConnectCan_Request::_port_type arg)
  {
    msg_.port = std::move(arg);
    return Init_ConnectCan_Request_auto_connect(msg_);
  }

private:
  ::altair_interfaces::srv::ConnectCan_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::altair_interfaces::srv::ConnectCan_Request>()
{
  return altair_interfaces::srv::builder::Init_ConnectCan_Request_port();
}

}  // namespace altair_interfaces


namespace altair_interfaces
{

namespace srv
{

namespace builder
{

class Init_ConnectCan_Response_message
{
public:
  explicit Init_ConnectCan_Response_message(::altair_interfaces::srv::ConnectCan_Response & msg)
  : msg_(msg)
  {}
  ::altair_interfaces::srv::ConnectCan_Response message(::altair_interfaces::srv::ConnectCan_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::altair_interfaces::srv::ConnectCan_Response msg_;
};

class Init_ConnectCan_Response_success
{
public:
  Init_ConnectCan_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConnectCan_Response_message success(::altair_interfaces::srv::ConnectCan_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ConnectCan_Response_message(msg_);
  }

private:
  ::altair_interfaces::srv::ConnectCan_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::altair_interfaces::srv::ConnectCan_Response>()
{
  return altair_interfaces::srv::builder::Init_ConnectCan_Response_success();
}

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__BUILDER_HPP_
