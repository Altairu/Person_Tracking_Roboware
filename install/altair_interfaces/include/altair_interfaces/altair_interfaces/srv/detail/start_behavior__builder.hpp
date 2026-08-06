// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from altair_interfaces:srv/StartBehavior.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__START_BEHAVIOR__BUILDER_HPP_
#define ALTAIR_INTERFACES__SRV__DETAIL__START_BEHAVIOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "altair_interfaces/srv/detail/start_behavior__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace altair_interfaces
{

namespace srv
{

namespace builder
{

class Init_StartBehavior_Request_behavior_name
{
public:
  Init_StartBehavior_Request_behavior_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::altair_interfaces::srv::StartBehavior_Request behavior_name(::altair_interfaces::srv::StartBehavior_Request::_behavior_name_type arg)
  {
    msg_.behavior_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::altair_interfaces::srv::StartBehavior_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::altair_interfaces::srv::StartBehavior_Request>()
{
  return altair_interfaces::srv::builder::Init_StartBehavior_Request_behavior_name();
}

}  // namespace altair_interfaces


namespace altair_interfaces
{

namespace srv
{

namespace builder
{

class Init_StartBehavior_Response_message
{
public:
  explicit Init_StartBehavior_Response_message(::altair_interfaces::srv::StartBehavior_Response & msg)
  : msg_(msg)
  {}
  ::altair_interfaces::srv::StartBehavior_Response message(::altair_interfaces::srv::StartBehavior_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::altair_interfaces::srv::StartBehavior_Response msg_;
};

class Init_StartBehavior_Response_success
{
public:
  Init_StartBehavior_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StartBehavior_Response_message success(::altair_interfaces::srv::StartBehavior_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_StartBehavior_Response_message(msg_);
  }

private:
  ::altair_interfaces::srv::StartBehavior_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::altair_interfaces::srv::StartBehavior_Response>()
{
  return altair_interfaces::srv::builder::Init_StartBehavior_Response_success();
}

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__START_BEHAVIOR__BUILDER_HPP_
