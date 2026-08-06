#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to altair_interfaces__srv__TriggerControl_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TriggerControl_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for TriggerControl_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::TriggerControl_Request::default())
  }
}

impl rosidl_runtime_rs::Message for TriggerControl_Request {
  type RmwMsg = super::srv::rmw::TriggerControl_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to altair_interfaces__srv__TriggerControl_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TriggerControl_Response {
    /// 制御開始指示が成功したか
    pub success: bool,

    /// 結果のステータスメッセージやエラー情報
    pub message: std::string::String,

}



impl Default for TriggerControl_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::TriggerControl_Response::default())
  }
}

impl rosidl_runtime_rs::Message for TriggerControl_Response {
  type RmwMsg = super::srv::rmw::TriggerControl_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to altair_interfaces__srv__GetAvailablePorts_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetAvailablePorts_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetAvailablePorts_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetAvailablePorts_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetAvailablePorts_Request {
  type RmwMsg = super::srv::rmw::GetAvailablePorts_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to altair_interfaces__srv__GetAvailablePorts_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetAvailablePorts_Response {
    /// 検出されたポート名（パス）の配列
    pub ports: Vec<std::string::String>,

}



impl Default for GetAvailablePorts_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetAvailablePorts_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetAvailablePorts_Response {
  type RmwMsg = super::srv::rmw::GetAvailablePorts_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ports: msg.ports
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ports: msg.ports
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ports: msg.ports
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
    }
  }
}


// Corresponds to altair_interfaces__srv__ConnectCan_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ConnectCan_Request {
    /// Request
    /// 接続するシリアルポート名 (例: "/dev/ttyACM0")
    pub port: std::string::String,

    /// 自動接続（探索）モードを有効にするか (Trueで有効。その場合port引数は無視される)
    pub auto_connect: bool,

}



impl Default for ConnectCan_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ConnectCan_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ConnectCan_Request {
  type RmwMsg = super::srv::rmw::ConnectCan_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        port: msg.port.as_str().into(),
        auto_connect: msg.auto_connect,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        port: msg.port.as_str().into(),
      auto_connect: msg.auto_connect,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      port: msg.port.to_string(),
      auto_connect: msg.auto_connect,
    }
  }
}


// Corresponds to altair_interfaces__srv__ConnectCan_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ConnectCan_Response {
    /// 接続（slcan起動 ＆ インターフェースUP）が成功したか
    pub success: bool,

    /// 結果のステータスメッセージやエラー内容
    pub message: std::string::String,

}



impl Default for ConnectCan_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ConnectCan_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ConnectCan_Response {
  type RmwMsg = super::srv::rmw::ConnectCan_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to altair_interfaces__srv__StartBehavior_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StartBehavior_Request {
    /// 起動するスクリプトファイル名 (例: "example_behavior.py")
    pub behavior_name: std::string::String,

}



impl Default for StartBehavior_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::StartBehavior_Request::default())
  }
}

impl rosidl_runtime_rs::Message for StartBehavior_Request {
  type RmwMsg = super::srv::rmw::StartBehavior_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        behavior_name: msg.behavior_name.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        behavior_name: msg.behavior_name.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      behavior_name: msg.behavior_name.to_string(),
    }
  }
}


// Corresponds to altair_interfaces__srv__StartBehavior_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StartBehavior_Response {
    /// プロセスの起動が成功したか
    pub success: bool,

    /// 結果のステータスメッセージやエラー内容
    pub message: std::string::String,

}



impl Default for StartBehavior_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::StartBehavior_Response::default())
  }
}

impl rosidl_runtime_rs::Message for StartBehavior_Response {
  type RmwMsg = super::srv::rmw::StartBehavior_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
    }
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


