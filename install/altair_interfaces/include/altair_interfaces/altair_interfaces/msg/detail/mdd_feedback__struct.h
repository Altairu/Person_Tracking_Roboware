// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from altair_interfaces:msg/MddFeedback.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__STRUCT_H_
#define ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MddFeedback in the package altair_interfaces.
/**
  * MDD (モータドライバ) のフィードバック情報メッセージ定義
 */
typedef struct altair_interfaces__msg__MddFeedback
{
  /// Limit SW1 - SW4 の状態 (True = ON / 押されている, False = OFF)
  bool limit_switches[4];
  /// システムエラーコード (0:正常, 1:初期化タイムアウト, 2:CAN受信タイムアウト, 4:送信失敗)
  uint8_t error_code;
  /// システム動作状態 (0: パラメータ設定モード, 1: 制御実行モード)
  uint8_t system_status;
  /// モータ1〜4の現在角度 (生データの 0.1 deg 単位を変換)
  float angles[4];
  /// モータ1〜4の現在回転速度 (生データの 0.01 rps 単位を変換)
  float speeds[4];
} altair_interfaces__msg__MddFeedback;

// Struct for a sequence of altair_interfaces__msg__MddFeedback.
typedef struct altair_interfaces__msg__MddFeedback__Sequence
{
  altair_interfaces__msg__MddFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} altair_interfaces__msg__MddFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__STRUCT_H_
