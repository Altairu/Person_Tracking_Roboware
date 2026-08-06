// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from altair_interfaces:srv/StartBehavior.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__START_BEHAVIOR__STRUCT_H_
#define ALTAIR_INTERFACES__SRV__DETAIL__START_BEHAVIOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'behavior_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/StartBehavior in the package altair_interfaces.
typedef struct altair_interfaces__srv__StartBehavior_Request
{
  /// 起動するスクリプトファイル名 (例: "example_behavior.py")
  rosidl_runtime_c__String behavior_name;
} altair_interfaces__srv__StartBehavior_Request;

// Struct for a sequence of altair_interfaces__srv__StartBehavior_Request.
typedef struct altair_interfaces__srv__StartBehavior_Request__Sequence
{
  altair_interfaces__srv__StartBehavior_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__srv__StartBehavior_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/StartBehavior in the package altair_interfaces.
typedef struct altair_interfaces__srv__StartBehavior_Response
{
  /// プロセスの起動が成功したか
  bool success;
  /// 結果のステータスメッセージやエラー内容
  rosidl_runtime_c__String message;
} altair_interfaces__srv__StartBehavior_Response;

// Struct for a sequence of altair_interfaces__srv__StartBehavior_Response.
typedef struct altair_interfaces__srv__StartBehavior_Response__Sequence
{
  altair_interfaces__srv__StartBehavior_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__srv__StartBehavior_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__START_BEHAVIOR__STRUCT_H_
