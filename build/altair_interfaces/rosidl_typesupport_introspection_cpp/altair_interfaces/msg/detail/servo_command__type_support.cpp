// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from altair_interfaces:msg/ServoCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "altair_interfaces/msg/detail/servo_command__struct.hpp"
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

void ServoCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) altair_interfaces::msg::ServoCommand(_init);
}

void ServoCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<altair_interfaces::msg::ServoCommand *>(message_memory);
  typed_message->~ServoCommand();
}

size_t size_function__ServoCommand__angles(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__ServoCommand__angles(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__ServoCommand__angles(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__ServoCommand__angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__ServoCommand__angles(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__ServoCommand__angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__ServoCommand__angles(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ServoCommand_message_member_array[1] = {
  {
    "angles",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces::msg::ServoCommand, angles),  // bytes offset in struct
    nullptr,  // default value
    size_function__ServoCommand__angles,  // size() function pointer
    get_const_function__ServoCommand__angles,  // get_const(index) function pointer
    get_function__ServoCommand__angles,  // get(index) function pointer
    fetch_function__ServoCommand__angles,  // fetch(index, &value) function pointer
    assign_function__ServoCommand__angles,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ServoCommand_message_members = {
  "altair_interfaces::msg",  // message namespace
  "ServoCommand",  // message name
  1,  // number of fields
  sizeof(altair_interfaces::msg::ServoCommand),
  ServoCommand_message_member_array,  // message members
  ServoCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  ServoCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ServoCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ServoCommand_message_members,
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
get_message_type_support_handle<altair_interfaces::msg::ServoCommand>()
{
  return &::altair_interfaces::msg::rosidl_typesupport_introspection_cpp::ServoCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, altair_interfaces, msg, ServoCommand)() {
  return &::altair_interfaces::msg::rosidl_typesupport_introspection_cpp::ServoCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
