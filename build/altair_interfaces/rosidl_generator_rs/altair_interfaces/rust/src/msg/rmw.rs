#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__msg__CanStatus() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__msg__CanStatus__init(msg: *mut CanStatus) -> bool;
    fn altair_interfaces__msg__CanStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CanStatus>, size: usize) -> bool;
    fn altair_interfaces__msg__CanStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CanStatus>);
    fn altair_interfaces__msg__CanStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CanStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<CanStatus>) -> bool;
}

// Corresponds to altair_interfaces__msg__CanStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// USB-to-CAN（slcan）の接続状態を保持するメッセージ定義

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CanStatus {
    /// 物理CANインターフェース(can0)が正常にUPしているか
    pub is_connected: bool,

    /// 現在接続されている物理ポート名 (例: "/dev/ttyACM0")
    pub active_port: rosidl_runtime_rs::String,

    /// 現在のビットレート (例: 1000000)
    pub bitrate: u32,

    /// 接続失敗時や切断時のエラーメッセージ
    pub error_message: rosidl_runtime_rs::String,

}



impl Default for CanStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__msg__CanStatus__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__msg__CanStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CanStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__CanStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__CanStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__CanStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CanStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CanStatus where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/msg/CanStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__msg__CanStatus() }
  }
}


#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__msg__MddFeedback() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__msg__MddFeedback__init(msg: *mut MddFeedback) -> bool;
    fn altair_interfaces__msg__MddFeedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MddFeedback>, size: usize) -> bool;
    fn altair_interfaces__msg__MddFeedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MddFeedback>);
    fn altair_interfaces__msg__MddFeedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MddFeedback>, out_seq: *mut rosidl_runtime_rs::Sequence<MddFeedback>) -> bool;
}

// Corresponds to altair_interfaces__msg__MddFeedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// MDD (モータドライバ) のフィードバック情報メッセージ定義

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MddFeedback {
    /// Limit SW1 - SW4 の状態 (True = ON / 押されている, False = OFF)
    pub limit_switches: [bool; 4],

    /// システムエラーコード (0:正常, 1:初期化タイムアウト, 2:CAN受信タイムアウト, 4:送信失敗)
    pub error_code: u8,

    /// システム動作状態 (0: パラメータ設定モード, 1: 制御実行モード)
    pub system_status: u8,

    /// モータ1〜4の現在角度 (生データの 0.1 deg 単位を変換)
    pub angles: [f32; 4],

    /// モータ1〜4の現在回転速度 (生データの 0.01 rps 単位を変換)
    pub speeds: [f32; 4],

}



impl Default for MddFeedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__msg__MddFeedback__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__msg__MddFeedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MddFeedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__MddFeedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__MddFeedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__MddFeedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MddFeedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MddFeedback where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/msg/MddFeedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__msg__MddFeedback() }
  }
}


#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__msg__MddCommand() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__msg__MddCommand__init(msg: *mut MddCommand) -> bool;
    fn altair_interfaces__msg__MddCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MddCommand>, size: usize) -> bool;
    fn altair_interfaces__msg__MddCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MddCommand>);
    fn altair_interfaces__msg__MddCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MddCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<MddCommand>) -> bool;
}

// Corresponds to altair_interfaces__msg__MddCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// MDD (モータドライバ) の目標値指令メッセージ定義

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MddCommand {
    /// モータ1〜4の目標値
    /// 【単位スケール】
    /// 速度モード: rps
    /// 角度モード: degree (度)
    /// 位置モード: mm (ミリメートル)
    /// (※ NUCブリッジ側で、マイコン用のスケール(各10倍のint16)に自動変換して送信されます)
    pub targets: [f32; 4],

}



impl Default for MddCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__msg__MddCommand__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__msg__MddCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MddCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__MddCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__MddCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__MddCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MddCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MddCommand where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/msg/MddCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__msg__MddCommand() }
  }
}


#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__msg__ServoCommand() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__msg__ServoCommand__init(msg: *mut ServoCommand) -> bool;
    fn altair_interfaces__msg__ServoCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ServoCommand>, size: usize) -> bool;
    fn altair_interfaces__msg__ServoCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ServoCommand>);
    fn altair_interfaces__msg__ServoCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ServoCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<ServoCommand>) -> bool;
}

// Corresponds to altair_interfaces__msg__ServoCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Servo (6ch サーボモジュール) の角度指令メッセージ定義

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ServoCommand {
    /// サーボ1〜6の目標角度
    /// (※ 180を超える場合はマイコン側で180に自動的にクリップされます)
    pub angles: [u8; 6],

}



impl Default for ServoCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__msg__ServoCommand__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__msg__ServoCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ServoCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__ServoCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__ServoCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__ServoCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ServoCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ServoCommand where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/msg/ServoCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__msg__ServoCommand() }
  }
}


#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__msg__SolenoidCommand() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__msg__SolenoidCommand__init(msg: *mut SolenoidCommand) -> bool;
    fn altair_interfaces__msg__SolenoidCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SolenoidCommand>, size: usize) -> bool;
    fn altair_interfaces__msg__SolenoidCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SolenoidCommand>);
    fn altair_interfaces__msg__SolenoidCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SolenoidCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<SolenoidCommand>) -> bool;
}

// Corresponds to altair_interfaces__msg__SolenoidCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Solenoid Valve (12ch 電磁弁モジュール) のON/OFF指令メッセージ定義

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SolenoidCommand {
    /// バルブ1〜12のON/OFF状態 (True = ON/オープン, False = OFF/クローズ)
    /// (※ NUCブリッジ側で、2バイト(12ビット)のビットマップデータにパックしてCAN送信します)
    pub valves: [bool; 12],

}



impl Default for SolenoidCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__msg__SolenoidCommand__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__msg__SolenoidCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SolenoidCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__SolenoidCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__SolenoidCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__msg__SolenoidCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SolenoidCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SolenoidCommand where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/msg/SolenoidCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__msg__SolenoidCommand() }
  }
}


