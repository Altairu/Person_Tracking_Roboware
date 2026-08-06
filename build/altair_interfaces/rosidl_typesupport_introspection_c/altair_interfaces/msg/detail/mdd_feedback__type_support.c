// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from altair_interfaces:msg/MddFeedback.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "altair_interfaces/msg/detail/mdd_feedback__rosidl_typesupport_introspection_c.h"
#include "altair_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "altair_interfaces/msg/detail/mdd_feedback__functions.h"
#include "altair_interfaces/msg/detail/mdd_feedback__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__MddFeedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  altair_interfaces__msg__MddFeedback__init(message_memory);
}

void altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__MddFeedback_fini_function(void * message_memory)
{
  altair_interfaces__msg__MddFeedback__fini(message_memory);
}

size_t altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__size_function__MddFeedback__limit_switches(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_const_function__MddFeedback__limit_switches(
  const void * untyped_member, size_t index)
{
  const bool * member =
    (const bool *)(untyped_member);
  return &member[index];
}

void * altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_function__MddFeedback__limit_switches(
  void * untyped_member, size_t index)
{
  bool * member =
    (bool *)(untyped_member);
  return &member[index];
}

void altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__fetch_function__MddFeedback__limit_switches(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_const_function__MddFeedback__limit_switches(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__assign_function__MddFeedback__limit_switches(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_function__MddFeedback__limit_switches(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

size_t altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__size_function__MddFeedback__angles(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_const_function__MddFeedback__angles(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_function__MddFeedback__angles(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__fetch_function__MddFeedback__angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_const_function__MddFeedback__angles(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__assign_function__MddFeedback__angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_function__MddFeedback__angles(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__size_function__MddFeedback__speeds(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_const_function__MddFeedback__speeds(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_function__MddFeedback__speeds(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__fetch_function__MddFeedback__speeds(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_const_function__MddFeedback__speeds(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__assign_function__MddFeedback__speeds(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_function__MddFeedback__speeds(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__MddFeedback_message_member_array[5] = {
  {
    "limit_switches",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces__msg__MddFeedback, limit_switches),  // bytes offset in struct
    NULL,  // default value
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__size_function__MddFeedback__limit_switches,  // size() function pointer
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_const_function__MddFeedback__limit_switches,  // get_const(index) function pointer
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_function__MddFeedback__limit_switches,  // get(index) function pointer
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__fetch_function__MddFeedback__limit_switches,  // fetch(index, &value) function pointer
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__assign_function__MddFeedback__limit_switches,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces__msg__MddFeedback, error_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "system_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces__msg__MddFeedback, system_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angles",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces__msg__MddFeedback, angles),  // bytes offset in struct
    NULL,  // default value
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__size_function__MddFeedback__angles,  // size() function pointer
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_const_function__MddFeedback__angles,  // get_const(index) function pointer
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_function__MddFeedback__angles,  // get(index) function pointer
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__fetch_function__MddFeedback__angles,  // fetch(index, &value) function pointer
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__assign_function__MddFeedback__angles,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speeds",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces__msg__MddFeedback, speeds),  // bytes offset in struct
    NULL,  // default value
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__size_function__MddFeedback__speeds,  // size() function pointer
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_const_function__MddFeedback__speeds,  // get_const(index) function pointer
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__get_function__MddFeedback__speeds,  // get(index) function pointer
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__fetch_function__MddFeedback__speeds,  // fetch(index, &value) function pointer
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__assign_function__MddFeedback__speeds,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__MddFeedback_message_members = {
  "altair_interfaces__msg",  // message namespace
  "MddFeedback",  // message name
  5,  // number of fields
  sizeof(altair_interfaces__msg__MddFeedback),
  altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__MddFeedback_message_member_array,  // message members
  altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__MddFeedback_init_function,  // function to initialize message memory (memory has to be allocated)
  altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__MddFeedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__MddFeedback_message_type_support_handle = {
  0,
  &altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__MddFeedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_altair_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, altair_interfaces, msg, MddFeedback)() {
  if (!altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__MddFeedback_message_type_support_handle.typesupport_identifier) {
    altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__MddFeedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &altair_interfaces__msg__MddFeedback__rosidl_typesupport_introspection_c__MddFeedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
