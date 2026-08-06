#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "can_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__can_msgs__msg__Frame() -> *const std::ffi::c_void;
}

#[link(name = "can_msgs__rosidl_generator_c")]
extern "C" {
    fn can_msgs__msg__Frame__init(msg: *mut Frame) -> bool;
    fn can_msgs__msg__Frame__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Frame>, size: usize) -> bool;
    fn can_msgs__msg__Frame__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Frame>);
    fn can_msgs__msg__Frame__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Frame>, out_seq: *mut rosidl_runtime_rs::Sequence<Frame>) -> bool;
}

// Corresponds to can_msgs__msg__Frame
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Frame {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_rtr: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_extended: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_error: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dlc: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub data: [u8; 8],

}



impl Default for Frame {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !can_msgs__msg__Frame__init(&mut msg as *mut _) {
        panic!("Call to can_msgs__msg__Frame__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Frame {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { can_msgs__msg__Frame__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { can_msgs__msg__Frame__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { can_msgs__msg__Frame__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Frame {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Frame where Self: Sized {
  const TYPE_NAME: &'static str = "can_msgs/msg/Frame";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__can_msgs__msg__Frame() }
  }
}


