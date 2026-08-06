// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from altair_interfaces:msg/MddCommand.idl
// generated code does not contain a copyright notice
#include "altair_interfaces/msg/detail/mdd_command__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "altair_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "altair_interfaces/msg/detail/mdd_command__struct.h"
#include "altair_interfaces/msg/detail/mdd_command__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _MddCommand__ros_msg_type = altair_interfaces__msg__MddCommand;

static bool _MddCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MddCommand__ros_msg_type * ros_message = static_cast<const _MddCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: targets
  {
    size_t size = 4;
    auto array_ptr = ros_message->targets;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _MddCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MddCommand__ros_msg_type * ros_message = static_cast<_MddCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: targets
  {
    size_t size = 4;
    auto array_ptr = ros_message->targets;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_altair_interfaces
size_t get_serialized_size_altair_interfaces__msg__MddCommand(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MddCommand__ros_msg_type * ros_message = static_cast<const _MddCommand__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name targets
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->targets;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MddCommand__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_altair_interfaces__msg__MddCommand(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_altair_interfaces
size_t max_serialized_size_altair_interfaces__msg__MddCommand(
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

  // member: targets
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
    using DataType = altair_interfaces__msg__MddCommand;
    is_plain =
      (
      offsetof(DataType, targets) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MddCommand__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_altair_interfaces__msg__MddCommand(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MddCommand = {
  "altair_interfaces::msg",
  "MddCommand",
  _MddCommand__cdr_serialize,
  _MddCommand__cdr_deserialize,
  _MddCommand__get_serialized_size,
  _MddCommand__max_serialized_size
};

static rosidl_message_type_support_t _MddCommand__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MddCommand,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, altair_interfaces, msg, MddCommand)() {
  return &_MddCommand__type_support;
}

#if defined(__cplusplus)
}
#endif
