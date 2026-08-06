// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from altair_interfaces:msg/SolenoidCommand.idl
// generated code does not contain a copyright notice
#include "altair_interfaces/msg/detail/solenoid_command__rosidl_typesupport_fastrtps_cpp.hpp"
#include "altair_interfaces/msg/detail/solenoid_command__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: valves
  {
    cdr << ros_message.valves;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_altair_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  altair_interfaces::msg::SolenoidCommand & ros_message)
{
  // Member: valves
  {
    cdr >> ros_message.valves;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_altair_interfaces
get_serialized_size(
  const altair_interfaces::msg::SolenoidCommand & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: valves
  {
    size_t array_size = 12;
    size_t item_size = sizeof(ros_message.valves[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_altair_interfaces
max_serialized_size_SolenoidCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: valves
  {
    size_t array_size = 12;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = altair_interfaces::msg::SolenoidCommand;
    is_plain =
      (
      offsetof(DataType, valves) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SolenoidCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const altair_interfaces::msg::SolenoidCommand *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SolenoidCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<altair_interfaces::msg::SolenoidCommand *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SolenoidCommand__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const altair_interfaces::msg::SolenoidCommand *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SolenoidCommand__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SolenoidCommand(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SolenoidCommand__callbacks = {
  "altair_interfaces::msg",
  "SolenoidCommand",
  _SolenoidCommand__cdr_serialize,
  _SolenoidCommand__cdr_deserialize,
  _SolenoidCommand__get_serialized_size,
  _SolenoidCommand__max_serialized_size
};

static rosidl_message_type_support_t _SolenoidCommand__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SolenoidCommand__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace altair_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_altair_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<altair_interfaces::msg::SolenoidCommand>()
{
  return &altair_interfaces::msg::typesupport_fastrtps_cpp::_SolenoidCommand__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, altair_interfaces, msg, SolenoidCommand)() {
  return &altair_interfaces::msg::typesupport_fastrtps_cpp::_SolenoidCommand__handle;
}

#ifdef __cplusplus
}
#endif
