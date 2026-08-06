// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from altair_interfaces:msg/CanStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "altair_interfaces/msg/detail/can_status__rosidl_typesupport_introspection_c.h"
#include "altair_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "altair_interfaces/msg/detail/can_status__functions.h"
#include "altair_interfaces/msg/detail/can_status__struct.h"


// Include directives for member types
// Member `active_port`
// Member `error_message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void altair_interfaces__msg__CanStatus__rosidl_typesupport_introspection_c__CanStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  altair_interfaces__msg__CanStatus__init(message_memory);
}

void altair_interfaces__msg__CanStatus__rosidl_typesupport_introspection_c__CanStatus_fini_function(void * message_memory)
{
  altair_interfaces__msg__CanStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember altair_interfaces__msg__CanStatus__rosidl_typesupport_introspection_c__CanStatus_message_member_array[4] = {
  {
    "is_connected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces__msg__CanStatus, is_connected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "active_port",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces__msg__CanStatus, active_port),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bitrate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces__msg__CanStatus, bitrate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(altair_interfaces__msg__CanStatus, error_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers altair_interfaces__msg__CanStatus__rosidl_typesupport_introspection_c__CanStatus_message_members = {
  "altair_interfaces__msg",  // message namespace
  "CanStatus",  // message name
  4,  // number of fields
  sizeof(altair_interfaces__msg__CanStatus),
  altair_interfaces__msg__CanStatus__rosidl_typesupport_introspection_c__CanStatus_message_member_array,  // message members
  altair_interfaces__msg__CanStatus__rosidl_typesupport_introspection_c__CanStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  altair_interfaces__msg__CanStatus__rosidl_typesupport_introspection_c__CanStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t altair_interfaces__msg__CanStatus__rosidl_typesupport_introspection_c__CanStatus_message_type_support_handle = {
  0,
  &altair_interfaces__msg__CanStatus__rosidl_typesupport_introspection_c__CanStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_altair_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, altair_interfaces, msg, CanStatus)() {
  if (!altair_interfaces__msg__CanStatus__rosidl_typesupport_introspection_c__CanStatus_message_type_support_handle.typesupport_identifier) {
    altair_interfaces__msg__CanStatus__rosidl_typesupport_introspection_c__CanStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &altair_interfaces__msg__CanStatus__rosidl_typesupport_introspection_c__CanStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
