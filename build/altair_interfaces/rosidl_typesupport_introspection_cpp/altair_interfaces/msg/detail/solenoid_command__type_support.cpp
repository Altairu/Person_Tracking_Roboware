// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from altair_interfaces:msg/SolenoidCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "altair_interfaces/msg/detail/solenoid_command__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace altair_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SolenoidCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) altair_interfaces::msg::SolenoidCommand(_init);
}

void SolenoidCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<altair_interfaces::msg::SolenoidCommand *>(message_memory);
  typed_message->~SolenoidCommand();
}

size_t size_function__SolenoidCommand__valves(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__SolenoidCommand__valves(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<bool, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__SolenoidCommand__valves(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<bool, 12> *>(untyped_member);
  return &member[index];
}

void fetch_function__SolenoidCommand__valves(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const bool *>(
    get_const_function__SolenoidCommand__valves(untyped_member, index));
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = item;
}

void assign_function__SolenoidCommand__valves(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<bool *>(
    get_function__SolenoidCommand__valves(untyped_member, index));
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SolenoidCommand_message_member_array[1] = {
  {
    "valves",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces::msg::SolenoidCommand, valves),  // bytes offset in struct
    nullptr,  // default value
    size_function__SolenoidCommand__valves,  // size() function pointer
    get_const_function__SolenoidCommand__valves,  // get_const(index) function pointer
    get_function__SolenoidCommand__valves,  // get(index) function pointer
    fetch_function__SolenoidCommand__valves,  // fetch(index, &value) function pointer
    assign_function__SolenoidCommand__valves,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SolenoidCommand_message_members = {
  "altair_interfaces::msg",  // message namespace
  "SolenoidCommand",  // message name
  1,  // number of fields
  sizeof(altair_interfaces::msg::SolenoidCommand),
  SolenoidCommand_message_member_array,  // message members
  SolenoidCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  SolenoidCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SolenoidCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SolenoidCommand_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace altair_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<altair_interfaces::msg::SolenoidCommand>()
{
  return &::altair_interfaces::msg::rosidl_typesupport_introspection_cpp::SolenoidCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, altair_interfaces, msg, SolenoidCommand)() {
  return &::altair_interfaces::msg::rosidl_typesupport_introspection_cpp::SolenoidCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
