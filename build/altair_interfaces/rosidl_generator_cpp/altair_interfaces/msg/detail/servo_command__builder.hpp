// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from altair_interfaces:msg/ServoCommand.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__SERVO_COMMAND__BUILDER_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__SERVO_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "altair_interfaces/msg/detail/servo_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace altair_interfaces
{

namespace msg
{

namespace builder
{

class Init_ServoCommand_angles
{
public:
  Init_ServoCommand_angles()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::altair_interfaces::msg::ServoCommand angles(::altair_interfaces::msg::ServoCommand::_angles_type arg)
  {
    msg_.angles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::altair_interfaces::msg::ServoCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::altair_interfaces::msg::ServoCommand>()
{
  return altair_interfaces::msg::builder::Init_ServoCommand_angles();
}

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__SERVO_COMMAND__BUILDER_HPP_
