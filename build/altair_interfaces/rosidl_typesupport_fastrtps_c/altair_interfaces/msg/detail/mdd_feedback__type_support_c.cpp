// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from altair_interfaces:msg/MddFeedback.idl
// generated code does not contain a copyright notice
#include "altair_interfaces/msg/detail/mdd_feedback__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "altair_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "altair_interfaces/msg/detail/mdd_feedback__struct.h"
#include "altair_interfaces/msg/detail/mdd_feedback__functions.h"
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


using _MddFeedback__ros_msg_type = altair_interfaces__msg__MddFeedback;

static bool _MddFeedback__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MddFeedback__ros_msg_type * ros_message = static_cast<const _MddFeedback__ros_msg_type *>(untyped_ros_message);
  // Field name: limit_switches
  {
    size_t size = 4;
    auto array_ptr = ros_message->limit_switches;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: error_code
  {
    cdr << ros_message->error_code;
  }

  // Field name: system_status
  {
    cdr << ros_message->system_status;
  }

  // Field name: angles
  {
    size_t size = 4;
    auto array_ptr = ros_message->angles;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: speeds
  {
    size_t size = 4;
    auto array_ptr = ros_message->speeds;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _MddFeedback__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MddFeedback__ros_msg_type * ros_message = static_cast<_MddFeedback__ros_msg_type *>(untyped_ros_message);
  // Field name: limit_switches
  {
    size_t size = 4;
    auto array_ptr = ros_message->limit_switches;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: error_code
  {
    cdr >> ros_message->error_code;
  }

  // Field name: system_status
  {
    cdr >> ros_message->system_status;
  }

  // Field name: angles
  {
    size_t size = 4;
    auto array_ptr = ros_message->angles;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: speeds
  {
    size_t size = 4;
    auto array_ptr = ros_message->speeds;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_altair_interfaces
size_t get_serialized_size_altair_interfaces__msg__MddFeedback(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MddFeedback__ros_msg_type * ros_message = static_cast<const _MddFeedback__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name limit_switches
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->limit_switches;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_code
  {
    size_t item_size = sizeof(ros_message->error_code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name system_status
  {
    size_t item_size = sizeof(ros_message->system_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name angles
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->angles;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speeds
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->speeds;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MddFeedback__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_altair_interfaces__msg__MddFeedback(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_altair_interfaces
size_t max_serialized_size_altair_interfaces__msg__MddFeedback(
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

  // member: limit_switches
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: error_code
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: system_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: angles
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: speeds
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
    using DataType = altair_interfaces__msg__MddFeedback;
    is_plain =
      (
      offsetof(DataType, speeds) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MddFeedback__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_altair_interfaces__msg__MddFeedback(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MddFeedback = {
  "altair_interfaces::msg",
  "MddFeedback",
  _MddFeedback__cdr_serialize,
  _MddFeedback__cdr_deserialize,
  _MddFeedback__get_serialized_size,
  _MddFeedback__max_serialized_size
};

static rosidl_message_type_support_t _MddFeedback__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MddFeedback,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, altair_interfaces, msg, MddFeedback)() {
  return &_MddFeedback__type_support;
}

#if defined(__cplusplus)
}
#endif
