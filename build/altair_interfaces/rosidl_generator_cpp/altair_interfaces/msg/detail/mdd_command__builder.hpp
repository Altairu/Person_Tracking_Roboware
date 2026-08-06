// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from altair_interfaces:msg/MddCommand.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__MDD_COMMAND__BUILDER_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__MDD_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "altair_interfaces/msg/detail/mdd_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace altair_interfaces
{

namespace msg
{

namespace builder
{

class Init_MddCommand_targets
{
public:
  Init_MddCommand_targets()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::altair_interfaces::msg::MddCommand targets(::altair_interfaces::msg::MddCommand::_targets_type arg)
  {
    msg_.targets = std::move(arg);
    return std::move(msg_);
  }

private:
  ::altair_interfaces::msg::MddCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::altair_interfaces::msg::MddCommand>()
{
  return altair_interfaces::msg::builder::Init_MddCommand_targets();
}

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__MDD_COMMAND__BUILDER_HPP_
