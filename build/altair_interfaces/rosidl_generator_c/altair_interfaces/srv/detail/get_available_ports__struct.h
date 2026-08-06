// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from altair_interfaces:srv/GetAvailablePorts.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__GET_AVAILABLE_PORTS__STRUCT_H_
#define ALTAIR_INTERFACES__SRV__DETAIL__GET_AVAILABLE_PORTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetAvailablePorts in the package altair_interfaces.
typedef struct altair_interfaces__srv__GetAvailablePorts_Request
{
  uint8_t structure_needs_at_least_one_member;
} altair_interfaces__srv__GetAvailablePorts_Request;

// Struct for a sequence of altair_interfaces__srv__GetAvailablePorts_Request.
typedef struct altair_interfaces__srv__GetAvailablePorts_Request__Sequence
{
  altair_interfaces__srv__GetAvailablePorts_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__srv__GetAvailablePorts_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'ports'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetAvailablePorts in the package altair_interfaces.
typedef struct altair_interfaces__srv__GetAvailablePorts_Response
{
  /// 検出されたポート名（パス）の配列
  rosidl_runtime_c__String__Sequence ports;
} altair_interfaces__srv__GetAvailablePorts_Response;

// Struct for a sequence of altair_interfaces__srv__GetAvailablePorts_Response.
typedef struct altair_interfaces__srv__GetAvailablePorts_Response__Sequence
{
  altair_interfaces__srv__GetAvailablePorts_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__srv__GetAvailablePorts_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__GET_AVAILABLE_PORTS__STRUCT_H_
