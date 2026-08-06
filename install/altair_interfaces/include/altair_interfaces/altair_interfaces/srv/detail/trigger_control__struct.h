// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from altair_interfaces:srv/TriggerControl.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__TRIGGER_CONTROL__STRUCT_H_
#define ALTAIR_INTERFACES__SRV__DETAIL__TRIGGER_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/TriggerControl in the package altair_interfaces.
typedef struct altair_interfaces__srv__TriggerControl_Request
{
  uint8_t structure_needs_at_least_one_member;
} altair_interfaces__srv__TriggerControl_Request;

// Struct for a sequence of altair_interfaces__srv__TriggerControl_Request.
typedef struct altair_interfaces__srv__TriggerControl_Request__Sequence
{
  altair_interfaces__srv__TriggerControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__srv__TriggerControl_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/TriggerControl in the package altair_interfaces.
typedef struct altair_interfaces__srv__TriggerControl_Response
{
  /// 制御開始指示が成功したか
  bool success;
  /// 結果のステータスメッセージやエラー情報
  rosidl_runtime_c__String message;
} altair_interfaces__srv__TriggerControl_Response;

// Struct for a sequence of altair_interfaces__srv__TriggerControl_Response.
typedef struct altair_interfaces__srv__TriggerControl_Response__Sequence
{
  altair_interfaces__srv__TriggerControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__srv__TriggerControl_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__TRIGGER_CONTROL__STRUCT_H_
