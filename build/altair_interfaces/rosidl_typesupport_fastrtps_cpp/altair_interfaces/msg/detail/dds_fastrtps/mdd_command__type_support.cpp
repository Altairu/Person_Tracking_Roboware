// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from altair_interfaces:msg/MddCommand.idl
// generated code does not contain a copyright notice
#include "altair_interfaces/msg/detail/mdd_command__rosidl_typesupport_fastrtps_cpp.hpp"
#include "altair_interfaces/msg/detail/mdd_command__struct.hpp"

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
  const altair_interfaces::msg::MddCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: targets
  {
    cdr << ros_message.targets;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_altair_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  altair_interfaces::msg::MddCommand & ros_message)
{
  // Member: targets
  {
    cdr >> ros_message.targets;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_altair_interfaces
get_serialized_size(
  const altair_interfaces::msg::MddCommand & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: targets
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.targets[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_altair_interfaces
max_serialized_size_MddCommand(
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


  // Member: targets
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = altair_interfaces::msg::MddCommand;
    is_plain =
      (
      offsetof(DataType, targets) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _MddCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const altair_interfaces::msg::MddCommand *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MddCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<altair_interfaces::msg::MddCommand *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MddCommand__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const altair_interfaces::msg::MddCommand *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MddCommand__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_MddCommand(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _MddCommand__callbacks = {
  "altair_interfaces::msg",
  "MddCommand",
  _MddCommand__cdr_serialize,
  _MddCommand__cdr_deserialize,
  _MddCommand__get_serialized_size,
  _MddCommand__max_serialized_size
};

static rosidl_message_type_support_t _MddCommand__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MddCommand__callbacks,
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
get_message_type_support_handle<altair_interfaces::msg::MddCommand>()
{
  return &altair_interfaces::msg::typesupport_fastrtps_cpp::_MddCommand__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, altair_interfaces, msg, MddCommand)() {
  return &altair_interfaces::msg::typesupport_fastrtps_cpp::_MddCommand__handle;
}

#ifdef __cplusplus
}
#endif
