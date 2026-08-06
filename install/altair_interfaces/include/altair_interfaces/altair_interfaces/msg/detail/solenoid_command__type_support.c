// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from altair_interfaces:msg/SolenoidCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "altair_interfaces/msg/detail/solenoid_command__rosidl_typesupport_introspection_c.h"
#include "altair_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "altair_interfaces/msg/detail/solenoid_command__functions.h"
#include "altair_interfaces/msg/detail/solenoid_command__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__SolenoidCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  altair_interfaces__msg__SolenoidCommand__init(message_memory);
}

void altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__SolenoidCommand_fini_function(void * message_memory)
{
  altair_interfaces__msg__SolenoidCommand__fini(message_memory);
}

size_t altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__size_function__SolenoidCommand__valves(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__get_const_function__SolenoidCommand__valves(
  const void * untyped_member, size_t index)
{
  const bool * member =
    (const bool *)(untyped_member);
  return &member[index];
}

void * altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__get_function__SolenoidCommand__valves(
  void * untyped_member, size_t index)
{
  bool * member =
    (bool *)(untyped_member);
  return &member[index];
}

void altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__fetch_function__SolenoidCommand__valves(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__get_const_function__SolenoidCommand__valves(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__assign_function__SolenoidCommand__valves(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__get_function__SolenoidCommand__valves(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__SolenoidCommand_message_member_array[1] = {
  {
    "valves",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces__msg__SolenoidCommand, valves),  // bytes offset in struct
    NULL,  // default value
    altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__size_function__SolenoidCommand__valves,  // size() function pointer
    altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__get_const_function__SolenoidCommand__valves,  // get_const(index) function pointer
    altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__get_function__SolenoidCommand__valves,  // get(index) function pointer
    altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__fetch_function__SolenoidCommand__valves,  // fetch(index, &value) function pointer
    altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__assign_function__SolenoidCommand__valves,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__SolenoidCommand_message_members = {
  "altair_interfaces__msg",  // message namespace
  "SolenoidCommand",  // message name
  1,  // number of fields
  sizeof(altair_interfaces__msg__SolenoidCommand),
  altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__SolenoidCommand_message_member_array,  // message members
  altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__SolenoidCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__SolenoidCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__SolenoidCommand_message_type_support_handle = {
  0,
  &altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__SolenoidCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_altair_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, altair_interfaces, msg, SolenoidCommand)() {
  if (!altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__SolenoidCommand_message_type_support_handle.typesupport_identifier) {
    altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__SolenoidCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &altair_interfaces__msg__SolenoidCommand__rosidl_typesupport_introspection_c__SolenoidCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
