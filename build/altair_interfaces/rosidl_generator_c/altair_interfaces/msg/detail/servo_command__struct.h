// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from altair_interfaces:msg/ServoCommand.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__SERVO_COMMAND__STRUCT_H_
#define ALTAIR_INTERFACES__MSG__DETAIL__SERVO_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ServoCommand in the package altair_interfaces.
/**
  * Servo (6ch サーボモジュール) の角度指令メッセージ定義
 */
typedef struct altair_interfaces__msg__ServoCommand
{
  /// サーボ1〜6の目標角度
  /// (※ 180を超える場合はマイコン側で180に自動的にクリップされます)
  uint8_t angles[6];
} altair_interfaces__msg__ServoCommand;

// Struct for a sequence of altair_interfaces__msg__ServoCommand.
typedef struct altair_interfaces__msg__ServoCommand__Sequence
{
  altair_interfaces__msg__ServoCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__msg__ServoCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__SERVO_COMMAND__STRUCT_H_
