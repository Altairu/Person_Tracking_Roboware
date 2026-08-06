#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__TriggerControl_Request() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__srv__TriggerControl_Request__init(msg: *mut TriggerControl_Request) -> bool;
    fn altair_interfaces__srv__TriggerControl_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TriggerControl_Request>, size: usize) -> bool;
    fn altair_interfaces__srv__TriggerControl_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TriggerControl_Request>);
    fn altair_interfaces__srv__TriggerControl_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TriggerControl_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<TriggerControl_Request>) -> bool;
}

// Corresponds to altair_interfaces__srv__TriggerControl_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TriggerControl_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for TriggerControl_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__srv__TriggerControl_Request__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__srv__TriggerControl_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TriggerControl_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__TriggerControl_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__TriggerControl_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__TriggerControl_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TriggerControl_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TriggerControl_Request where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/srv/TriggerControl_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__TriggerControl_Request() }
  }
}


#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__TriggerControl_Response() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__srv__TriggerControl_Response__init(msg: *mut TriggerControl_Response) -> bool;
    fn altair_interfaces__srv__TriggerControl_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TriggerControl_Response>, size: usize) -> bool;
    fn altair_interfaces__srv__TriggerControl_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TriggerControl_Response>);
    fn altair_interfaces__srv__TriggerControl_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TriggerControl_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<TriggerControl_Response>) -> bool;
}

// Corresponds to altair_interfaces__srv__TriggerControl_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TriggerControl_Response {
    /// 制御開始指示が成功したか
    pub success: bool,

    /// 結果のステータスメッセージやエラー情報
    pub message: rosidl_runtime_rs::String,

}



impl Default for TriggerControl_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__srv__TriggerControl_Response__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__srv__TriggerControl_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TriggerControl_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__TriggerControl_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__TriggerControl_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__TriggerControl_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TriggerControl_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TriggerControl_Response where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/srv/TriggerControl_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__TriggerControl_Response() }
  }
}


#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__GetAvailablePorts_Request() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__srv__GetAvailablePorts_Request__init(msg: *mut GetAvailablePorts_Request) -> bool;
    fn altair_interfaces__srv__GetAvailablePorts_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetAvailablePorts_Request>, size: usize) -> bool;
    fn altair_interfaces__srv__GetAvailablePorts_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetAvailablePorts_Request>);
    fn altair_interfaces__srv__GetAvailablePorts_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetAvailablePorts_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetAvailablePorts_Request>) -> bool;
}

// Corresponds to altair_interfaces__srv__GetAvailablePorts_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetAvailablePorts_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetAvailablePorts_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__srv__GetAvailablePorts_Request__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__srv__GetAvailablePorts_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetAvailablePorts_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__GetAvailablePorts_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__GetAvailablePorts_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__GetAvailablePorts_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetAvailablePorts_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetAvailablePorts_Request where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/srv/GetAvailablePorts_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__GetAvailablePorts_Request() }
  }
}


#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__GetAvailablePorts_Response() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__srv__GetAvailablePorts_Response__init(msg: *mut GetAvailablePorts_Response) -> bool;
    fn altair_interfaces__srv__GetAvailablePorts_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetAvailablePorts_Response>, size: usize) -> bool;
    fn altair_interfaces__srv__GetAvailablePorts_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetAvailablePorts_Response>);
    fn altair_interfaces__srv__GetAvailablePorts_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetAvailablePorts_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetAvailablePorts_Response>) -> bool;
}

// Corresponds to altair_interfaces__srv__GetAvailablePorts_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetAvailablePorts_Response {
    /// 検出されたポート名（パス）の配列
    pub ports: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

}



impl Default for GetAvailablePorts_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__srv__GetAvailablePorts_Response__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__srv__GetAvailablePorts_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetAvailablePorts_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__GetAvailablePorts_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__GetAvailablePorts_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__GetAvailablePorts_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetAvailablePorts_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetAvailablePorts_Response where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/srv/GetAvailablePorts_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__GetAvailablePorts_Response() }
  }
}


#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__ConnectCan_Request() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__srv__ConnectCan_Request__init(msg: *mut ConnectCan_Request) -> bool;
    fn altair_interfaces__srv__ConnectCan_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ConnectCan_Request>, size: usize) -> bool;
    fn altair_interfaces__srv__ConnectCan_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ConnectCan_Request>);
    fn altair_interfaces__srv__ConnectCan_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ConnectCan_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ConnectCan_Request>) -> bool;
}

// Corresponds to altair_interfaces__srv__ConnectCan_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ConnectCan_Request {
    /// Request
    /// 接続するシリアルポート名 (例: "/dev/ttyACM0")
    pub port: rosidl_runtime_rs::String,

    /// 自動接続（探索）モードを有効にするか (Trueで有効。その場合port引数は無視される)
    pub auto_connect: bool,

}



impl Default for ConnectCan_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__srv__ConnectCan_Request__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__srv__ConnectCan_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ConnectCan_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__ConnectCan_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__ConnectCan_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__ConnectCan_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ConnectCan_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ConnectCan_Request where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/srv/ConnectCan_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__ConnectCan_Request() }
  }
}


#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__ConnectCan_Response() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__srv__ConnectCan_Response__init(msg: *mut ConnectCan_Response) -> bool;
    fn altair_interfaces__srv__ConnectCan_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ConnectCan_Response>, size: usize) -> bool;
    fn altair_interfaces__srv__ConnectCan_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ConnectCan_Response>);
    fn altair_interfaces__srv__ConnectCan_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ConnectCan_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ConnectCan_Response>) -> bool;
}

// Corresponds to altair_interfaces__srv__ConnectCan_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ConnectCan_Response {
    /// 接続（slcan起動 ＆ インターフェースUP）が成功したか
    pub success: bool,

    /// 結果のステータスメッセージやエラー内容
    pub message: rosidl_runtime_rs::String,

}



impl Default for ConnectCan_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__srv__ConnectCan_Response__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__srv__ConnectCan_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ConnectCan_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__ConnectCan_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__ConnectCan_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__ConnectCan_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ConnectCan_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ConnectCan_Response where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/srv/ConnectCan_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__ConnectCan_Response() }
  }
}


#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__StartBehavior_Request() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__srv__StartBehavior_Request__init(msg: *mut StartBehavior_Request) -> bool;
    fn altair_interfaces__srv__StartBehavior_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<StartBehavior_Request>, size: usize) -> bool;
    fn altair_interfaces__srv__StartBehavior_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<StartBehavior_Request>);
    fn altair_interfaces__srv__StartBehavior_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<StartBehavior_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<StartBehavior_Request>) -> bool;
}

// Corresponds to altair_interfaces__srv__StartBehavior_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StartBehavior_Request {
    /// 起動するスクリプトファイル名 (例: "example_behavior.py")
    pub behavior_name: rosidl_runtime_rs::String,

}



impl Default for StartBehavior_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__srv__StartBehavior_Request__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__srv__StartBehavior_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for StartBehavior_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__StartBehavior_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__StartBehavior_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__StartBehavior_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for StartBehavior_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for StartBehavior_Request where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/srv/StartBehavior_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__StartBehavior_Request() }
  }
}


#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__StartBehavior_Response() -> *const std::ffi::c_void;
}

#[link(name = "altair_interfaces__rosidl_generator_c")]
extern "C" {
    fn altair_interfaces__srv__StartBehavior_Response__init(msg: *mut StartBehavior_Response) -> bool;
    fn altair_interfaces__srv__StartBehavior_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<StartBehavior_Response>, size: usize) -> bool;
    fn altair_interfaces__srv__StartBehavior_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<StartBehavior_Response>);
    fn altair_interfaces__srv__StartBehavior_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<StartBehavior_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<StartBehavior_Response>) -> bool;
}

// Corresponds to altair_interfaces__srv__StartBehavior_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StartBehavior_Response {
    /// プロセスの起動が成功したか
    pub success: bool,

    /// 結果のステータスメッセージやエラー内容
    pub message: rosidl_runtime_rs::String,

}



impl Default for StartBehavior_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !altair_interfaces__srv__StartBehavior_Response__init(&mut msg as *mut _) {
        panic!("Call to altair_interfaces__srv__StartBehavior_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for StartBehavior_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__StartBehavior_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__StartBehavior_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { altair_interfaces__srv__StartBehavior_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for StartBehavior_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for StartBehavior_Response where Self: Sized {
  const TYPE_NAME: &'static str = "altair_interfaces/srv/StartBehavior_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__altair_interfaces__srv__StartBehavior_Response() }
  }
}






#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__altair_interfaces__srv__TriggerControl() -> *const std::ffi::c_void;
}

// Corresponds to altair_interfaces__srv__TriggerControl
#[allow(missing_docs, non_camel_case_types)]
pub struct TriggerControl;

impl rosidl_runtime_rs::Service for TriggerControl {
    type Request = TriggerControl_Request;
    type Response = TriggerControl_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__altair_interfaces__srv__TriggerControl() }
    }
}




#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__altair_interfaces__srv__GetAvailablePorts() -> *const std::ffi::c_void;
}

// Corresponds to altair_interfaces__srv__GetAvailablePorts
#[allow(missing_docs, non_camel_case_types)]
pub struct GetAvailablePorts;

impl rosidl_runtime_rs::Service for GetAvailablePorts {
    type Request = GetAvailablePorts_Request;
    type Response = GetAvailablePorts_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__altair_interfaces__srv__GetAvailablePorts() }
    }
}




#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__altair_interfaces__srv__ConnectCan() -> *const std::ffi::c_void;
}

// Corresponds to altair_interfaces__srv__ConnectCan
#[allow(missing_docs, non_camel_case_types)]
pub struct ConnectCan;

impl rosidl_runtime_rs::Service for ConnectCan {
    type Request = ConnectCan_Request;
    type Response = ConnectCan_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__altair_interfaces__srv__ConnectCan() }
    }
}




#[link(name = "altair_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__altair_interfaces__srv__StartBehavior() -> *const std::ffi::c_void;
}

// Corresponds to altair_interfaces__srv__StartBehavior
#[allow(missing_docs, non_camel_case_types)]
pub struct StartBehavior;

impl rosidl_runtime_rs::Service for StartBehavior {
    type Request = StartBehavior_Request;
    type Response = StartBehavior_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__altair_interfaces__srv__StartBehavior() }
    }
}


