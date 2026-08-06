// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from altair_interfaces:msg/SolenoidCommand.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__SOLENOID_COMMAND__STRUCT_H_
#define ALTAIR_INTERFACES__MSG__DETAIL__SOLENOID_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SolenoidCommand in the package altair_interfaces.
/**
  * Solenoid Valve (12ch 電磁弁モジュール) のON/OFF指令メッセージ定義
 */
typedef struct altair_interfaces__msg__SolenoidCommand
{
  /// バルブ1〜12のON/OFF状態 (True = ON/オープン, False = OFF/クローズ)
  /// (※ NUCブリッジ側で、2バイト(12ビット)のビットマップデータにパックしてCAN送信します)
  bool valves[12];
} altair_interfaces__msg__SolenoidCommand;

// Struct for a sequence of altair_interfaces__msg__SolenoidCommand.
typedef struct altair_interfaces__msg__SolenoidCommand__Sequence
{
  altair_interfaces__msg__SolenoidCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__msg__SolenoidCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__SOLENOID_COMMAND__STRUCT_H_
