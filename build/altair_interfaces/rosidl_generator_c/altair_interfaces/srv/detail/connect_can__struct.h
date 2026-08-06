// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from altair_interfaces:srv/ConnectCan.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__STRUCT_H_
#define ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'port'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ConnectCan in the package altair_interfaces.
typedef struct altair_interfaces__srv__ConnectCan_Request
{
  /// Request
  /// 接続するシリアルポート名 (例: "/dev/ttyACM0")
  rosidl_runtime_c__String port;
  /// 自動接続（探索）モードを有効にするか (Trueで有効。その場合port引数は無視される)
  bool auto_connect;
} altair_interfaces__srv__ConnectCan_Request;

// Struct for a sequence of altair_interfaces__srv__ConnectCan_Request.
typedef struct altair_interfaces__srv__ConnectCan_Request__Sequence
{
  altair_interfaces__srv__ConnectCan_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__srv__ConnectCan_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ConnectCan in the package altair_interfaces.
typedef struct altair_interfaces__srv__ConnectCan_Response
{
  /// 接続（slcan起動 ＆ インターフェースUP）が成功したか
  bool success;
  /// 結果のステータスメッセージやエラー内容
  rosidl_runtime_c__String message;
} altair_interfaces__srv__ConnectCan_Response;

// Struct for a sequence of altair_interfaces__srv__ConnectCan_Response.
typedef struct altair_interfaces__srv__ConnectCan_Response__Sequence
{
  altair_interfaces__srv__ConnectCan_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__srv__ConnectCan_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__STRUCT_H_
