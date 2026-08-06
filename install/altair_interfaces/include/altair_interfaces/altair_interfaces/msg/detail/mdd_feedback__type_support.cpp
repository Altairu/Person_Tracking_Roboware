// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from altair_interfaces:msg/MddFeedback.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "altair_interfaces/msg/detail/mdd_feedback__struct.hpp"
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

void MddFeedback_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) altair_interfaces::msg::MddFeedback(_init);
}

void MddFeedback_fini_function(void * message_memory)
{
  auto typed_message = static_cast<altair_interfaces::msg::MddFeedback *>(message_memory);
  typed_message->~MddFeedback();
}

size_t size_function__MddFeedback__limit_switches(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__MddFeedback__limit_switches(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<bool, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__MddFeedback__limit_switches(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<bool, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__MddFeedback__limit_switches(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const bool *>(
    get_const_function__MddFeedback__limit_switches(untyped_member, index));
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = item;
}

void assign_function__MddFeedback__limit_switches(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<bool *>(
    get_function__MddFeedback__limit_switches(untyped_member, index));
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  item = value;
}

size_t size_function__MddFeedback__angles(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__MddFeedback__angles(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__MddFeedback__angles(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__MddFeedback__angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__MddFeedback__angles(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__MddFeedback__angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__MddFeedback__angles(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__MddFeedback__speeds(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__MddFeedback__speeds(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__MddFeedback__speeds(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__MddFeedback__speeds(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__MddFeedback__speeds(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__MddFeedback__speeds(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__MddFeedback__speeds(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MddFeedback_message_member_array[5] = {
  {
    "limit_switches",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces::msg::MddFeedback, limit_switches),  // bytes offset in struct
    nullptr,  // default value
    size_function__MddFeedback__limit_switches,  // size() function pointer
    get_const_function__MddFeedback__limit_switches,  // get_const(index) function pointer
    get_function__MddFeedback__limit_switches,  // get(index) function pointer
    fetch_function__MddFeedback__limit_switches,  // fetch(index, &value) function pointer
    assign_function__MddFeedback__limit_switches,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "error_code",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces::msg::MddFeedback, error_code),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "system_status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces::msg::MddFeedback, system_status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "angles",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces::msg::MddFeedback, angles),  // bytes offset in struct
    nullptr,  // default value
    size_function__MddFeedback__angles,  // size() function pointer
    get_const_function__MddFeedback__angles,  // get_const(index) function pointer
    get_function__MddFeedback__angles,  // get(index) function pointer
    fetch_function__MddFeedback__angles,  // fetch(index, &value) function pointer
    assign_function__MddFeedback__angles,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "speeds",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces::msg::MddFeedback, speeds),  // bytes offset in struct
    nullptr,  // default value
    size_function__MddFeedback__speeds,  // size() function pointer
    get_const_function__MddFeedback__speeds,  // get_const(index) function pointer
    get_function__MddFeedback__speeds,  // get(index) function pointer
    fetch_function__MddFeedback__speeds,  // fetch(index, &value) function pointer
    assign_function__MddFeedback__speeds,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MddFeedback_message_members = {
  "altair_interfaces::msg",  // message namespace
  "MddFeedback",  // message name
  5,  // number of fields
  sizeof(altair_interfaces::msg::MddFeedback),
  MddFeedback_message_member_array,  // message members
  MddFeedback_init_function,  // function to initialize message memory (memory has to be allocated)
  MddFeedback_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MddFeedback_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MddFeedback_message_members,
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
get_message_type_support_handle<altair_interfaces::msg::MddFeedback>()
{
  return &::altair_interfaces::msg::rosidl_typesupport_introspection_cpp::MddFeedback_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, altair_interfaces, msg, MddFeedback)() {
  return &::altair_interfaces::msg::rosidl_typesupport_introspection_cpp::MddFeedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
