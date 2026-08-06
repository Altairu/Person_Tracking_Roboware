// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from altair_interfaces:msg/SolenoidCommand.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__SOLENOID_COMMAND__BUILDER_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__SOLENOID_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "altair_interfaces/msg/detail/solenoid_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace altair_interfaces
{

namespace msg
{

namespace builder
{

class Init_SolenoidCommand_valves
{
public:
  Init_SolenoidCommand_valves()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::altair_interfaces::msg::SolenoidCommand valves(::altair_interfaces::msg::SolenoidCommand::_valves_type arg)
  {
    msg_.valves = std::move(arg);
    return std::move(msg_);
  }

private:
  ::altair_interfaces::msg::SolenoidCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::altair_interfaces::msg::SolenoidCommand>()
{
  return altair_interfaces::msg::builder::Init_SolenoidCommand_valves();
}

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__SOLENOID_COMMAND__BUILDER_HPP_
