// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from altair_interfaces:msg/MddFeedback.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__BUILDER_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "altair_interfaces/msg/detail/mdd_feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace altair_interfaces
{

namespace msg
{

namespace builder
{

class Init_MddFeedback_speeds
{
public:
  explicit Init_MddFeedback_speeds(::altair_interfaces::msg::MddFeedback & msg)
  : msg_(msg)
  {}
  ::altair_interfaces::msg::MddFeedback speeds(::altair_interfaces::msg::MddFeedback::_speeds_type arg)
  {
    msg_.speeds = std::move(arg);
    return std::move(msg_);
  }

private:
  ::altair_interfaces::msg::MddFeedback msg_;
};

class Init_MddFeedback_angles
{
public:
  explicit Init_MddFeedback_angles(::altair_interfaces::msg::MddFeedback & msg)
  : msg_(msg)
  {}
  Init_MddFeedback_speeds angles(::altair_interfaces::msg::MddFeedback::_angles_type arg)
  {
    msg_.angles = std::move(arg);
    return Init_MddFeedback_speeds(msg_);
  }

private:
  ::altair_interfaces::msg::MddFeedback msg_;
};

class Init_MddFeedback_system_status
{
public:
  explicit Init_MddFeedback_system_status(::altair_interfaces::msg::MddFeedback & msg)
  : msg_(msg)
  {}
  Init_MddFeedback_angles system_status(::altair_interfaces::msg::MddFeedback::_system_status_type arg)
  {
    msg_.system_status = std::move(arg);
    return Init_MddFeedback_angles(msg_);
  }

private:
  ::altair_interfaces::msg::MddFeedback msg_;
};

class Init_MddFeedback_error_code
{
public:
  explicit Init_MddFeedback_error_code(::altair_interfaces::msg::MddFeedback & msg)
  : msg_(msg)
  {}
  Init_MddFeedback_system_status error_code(::altair_interfaces::msg::MddFeedback::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_MddFeedback_system_status(msg_);
  }

private:
  ::altair_interfaces::msg::MddFeedback msg_;
};

class Init_MddFeedback_limit_switches
{
public:
  Init_MddFeedback_limit_switches()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MddFeedback_error_code limit_switches(::altair_interfaces::msg::MddFeedback::_limit_switches_type arg)
  {
    msg_.limit_switches = std::move(arg);
    return Init_MddFeedback_error_code(msg_);
  }

private:
  ::altair_interfaces::msg::MddFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::altair_interfaces::msg::MddFeedback>()
{
  return altair_interfaces::msg::builder::Init_MddFeedback_limit_switches();
}

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__BUILDER_HPP_
