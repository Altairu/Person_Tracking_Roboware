// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from altair_interfaces:msg/SolenoidCommand.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__SOLENOID_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__SOLENOID_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "altair_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "altair_interfaces/msg/detail/solenoid_command__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace altair_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_altair_interfaces
cdr_serialize(
  const altair_interfaces::msg::SolenoidCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_altair_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  altair_interfaces::msg::SolenoidCommand & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_altair_interfaces
get_serialized_size(
  const altair_interfaces::msg::SolenoidCommand & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_altair_interfaces
max_serialized_size_SolenoidCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace altair_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_altair_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, altair_interfaces, msg, SolenoidCommand)();

#ifdef __cplusplus
}
#endif

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__SOLENOID_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
