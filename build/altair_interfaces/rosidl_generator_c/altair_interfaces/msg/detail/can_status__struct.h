// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from altair_interfaces:msg/CanStatus.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__CAN_STATUS__STRUCT_H_
#define ALTAIR_INTERFACES__MSG__DETAIL__CAN_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'active_port'
// Member 'error_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CanStatus in the package altair_interfaces.
/**
  * USB-to-CAN（slcan）の接続状態を保持するメッセージ定義
 */
typedef struct altair_interfaces__msg__CanStatus
{
  /// 物理CANインターフェース(can0)が正常にUPしているか
  bool is_connected;
  /// 現在接続されている物理ポート名 (例: "/dev/ttyACM0")
  rosidl_runtime_c__String active_port;
  /// 現在のビットレート (例: 1000000)
  uint32_t bitrate;
  /// 接続失敗時や切断時のエラーメッセージ
  rosidl_runtime_c__String error_message;
} altair_interfaces__msg__CanStatus;

// Struct for a sequence of altair_interfaces__msg__CanStatus.
typedef struct altair_interfaces__msg__CanStatus__Sequence
{
  altair_interfaces__msg__CanStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__msg__CanStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__CAN_STATUS__STRUCT_H_
