// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from altair_interfaces:msg/MddCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "altair_interfaces/msg/detail/mdd_command__rosidl_typesupport_introspection_c.h"
#include "altair_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "altair_interfaces/msg/detail/mdd_command__functions.h"
#include "altair_interfaces/msg/detail/mdd_command__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__MddCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  altair_interfaces__msg__MddCommand__init(message_memory);
}

void altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__MddCommand_fini_function(void * message_memory)
{
  altair_interfaces__msg__MddCommand__fini(message_memory);
}

size_t altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__size_function__MddCommand__targets(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__get_const_function__MddCommand__targets(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__get_function__MddCommand__targets(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__fetch_function__MddCommand__targets(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__get_const_function__MddCommand__targets(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__assign_function__MddCommand__targets(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__get_function__MddCommand__targets(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__MddCommand_message_member_array[1] = {
  {
    "targets",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces__msg__MddCommand, targets),  // bytes offset in struct
    NULL,  // default value
    altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__size_function__MddCommand__targets,  // size() function pointer
    altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__get_const_function__MddCommand__targets,  // get_const(index) function pointer
    altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__get_function__MddCommand__targets,  // get(index) function pointer
    altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__fetch_function__MddCommand__targets,  // fetch(index, &value) function pointer
    altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__assign_function__MddCommand__targets,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__MddCommand_message_members = {
  "altair_interfaces__msg",  // message namespace
  "MddCommand",  // message name
  1,  // number of fields
  sizeof(altair_interfaces__msg__MddCommand),
  altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__MddCommand_message_member_array,  // message members
  altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__MddCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__MddCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__MddCommand_message_type_support_handle = {
  0,
  &altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__MddCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_altair_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, altair_interfaces, msg, MddCommand)() {
  if (!altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__MddCommand_message_type_support_handle.typesupport_identifier) {
    altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__MddCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &altair_interfaces__msg__MddCommand__rosidl_typesupport_introspection_c__MddCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
