#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to altair_interfaces__msg__CanStatus
/// USB-to-CAN（slcan）の接続状態を保持するメッセージ定義

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CanStatus {
    /// 物理CANインターフェース(can0)が正常にUPしているか
    pub is_connected: bool,

    /// 現在接続されている物理ポート名 (例: "/dev/ttyACM0")
    pub active_port: std::string::String,

    /// 現在のビットレート (例: 1000000)
    pub bitrate: u32,

    /// 接続失敗時や切断時のエラーメッセージ
    pub error_message: std::string::String,

}



impl Default for CanStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CanStatus::default())
  }
}

impl rosidl_runtime_rs::Message for CanStatus {
  type RmwMsg = super::msg::rmw::CanStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        is_connected: msg.is_connected,
        active_port: msg.active_port.as_str().into(),
        bitrate: msg.bitrate,
        error_message: msg.error_message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      is_connected: msg.is_connected,
        active_port: msg.active_port.as_str().into(),
      bitrate: msg.bitrate,
        error_message: msg.error_message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      is_connected: msg.is_connected,
      active_port: msg.active_port.to_string(),
      bitrate: msg.bitrate,
      error_message: msg.error_message.to_string(),
    }
  }
}


// Corresponds to altair_interfaces__msg__MddFeedback
/// MDD (モータドライバ) のフィードバック情報メッセージ定義

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MddFeedback::default())
  }
}

impl rosidl_runtime_rs::Message for MddFeedback {
  type RmwMsg = super::msg::rmw::MddFeedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        limit_switches: msg.limit_switches,
        error_code: msg.error_code,
        system_status: msg.system_status,
        angles: msg.angles,
        speeds: msg.speeds,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        limit_switches: msg.limit_switches,
      error_code: msg.error_code,
      system_status: msg.system_status,
        angles: msg.angles,
        speeds: msg.speeds,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      limit_switches: msg.limit_switches,
      error_code: msg.error_code,
      system_status: msg.system_status,
      angles: msg.angles,
      speeds: msg.speeds,
    }
  }
}


// Corresponds to altair_interfaces__msg__MddCommand
/// MDD (モータドライバ) の目標値指令メッセージ定義

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MddCommand::default())
  }
}

impl rosidl_runtime_rs::Message for MddCommand {
  type RmwMsg = super::msg::rmw::MddCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        targets: msg.targets,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        targets: msg.targets,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      targets: msg.targets,
    }
  }
}


// Corresponds to altair_interfaces__msg__ServoCommand
/// Servo (6ch サーボモジュール) の角度指令メッセージ定義

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ServoCommand {
    /// サーボ1〜6の目標角度
    /// (※ 180を超える場合はマイコン側で180に自動的にクリップされます)
    pub angles: [u8; 6],

}



impl Default for ServoCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ServoCommand::default())
  }
}

impl rosidl_runtime_rs::Message for ServoCommand {
  type RmwMsg = super::msg::rmw::ServoCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        angles: msg.angles,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        angles: msg.angles,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      angles: msg.angles,
    }
  }
}


// Corresponds to altair_interfaces__msg__SolenoidCommand
/// Solenoid Valve (12ch 電磁弁モジュール) のON/OFF指令メッセージ定義

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SolenoidCommand {
    /// バルブ1〜12のON/OFF状態 (True = ON/オープン, False = OFF/クローズ)
    /// (※ NUCブリッジ側で、2バイト(12ビット)のビットマップデータにパックしてCAN送信します)
    pub valves: [bool; 12],

}



impl Default for SolenoidCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SolenoidCommand::default())
  }
}

impl rosidl_runtime_rs::Message for SolenoidCommand {
  type RmwMsg = super::msg::rmw::SolenoidCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        valves: msg.valves,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        valves: msg.valves,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      valves: msg.valves,
    }
  }
}


