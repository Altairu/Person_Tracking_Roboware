// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from altair_interfaces:msg/MddCommand.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__MDD_COMMAND__STRUCT_H_
#define ALTAIR_INTERFACES__MSG__DETAIL__MDD_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MddCommand in the package altair_interfaces.
/**
  * MDD (モータドライバ) の目標値指令メッセージ定義
 */
typedef struct altair_interfaces__msg__MddCommand
{
  /// モータ1〜4の目標値
  /// 【単位スケール】
  /// 速度モード: rps
  /// 角度モード: degree (度)
  /// 位置モード: mm (ミリメートル)
  /// (※ NUCブリッジ側で、マイコン用のスケール(各10倍のint16)に自動変換して送信されます)
  float targets[4];
} altair_interfaces__msg__MddCommand;

// Struct for a sequence of altair_interfaces__msg__MddCommand.
typedef struct altair_interfaces__msg__MddCommand__Sequence
{
  altair_interfaces__msg__MddCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__msg__MddCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__MDD_COMMAND__STRUCT_H_
