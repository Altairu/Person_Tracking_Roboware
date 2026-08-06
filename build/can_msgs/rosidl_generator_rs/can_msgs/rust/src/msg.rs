#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to can_msgs__msg__Frame

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Frame {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Frame::default())
  }
}

impl rosidl_runtime_rs::Message for Frame {
  type RmwMsg = super::msg::rmw::Frame;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        id: msg.id,
        is_rtr: msg.is_rtr,
        is_extended: msg.is_extended,
        is_error: msg.is_error,
        dlc: msg.dlc,
        data: msg.data,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      id: msg.id,
      is_rtr: msg.is_rtr,
      is_extended: msg.is_extended,
      is_error: msg.is_error,
      dlc: msg.dlc,
        data: msg.data,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      id: msg.id,
      is_rtr: msg.is_rtr,
      is_extended: msg.is_extended,
      is_error: msg.is_error,
      dlc: msg.dlc,
      data: msg.data,
    }
  }
}


