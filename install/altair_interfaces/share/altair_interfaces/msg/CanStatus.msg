# USB-to-CAN（slcan）の接続状態を保持するメッセージ定義
bool is_connected       # 物理CANインターフェース(can0)が正常にUPしているか
string active_port      # 現在接続されている物理ポート名 (例: "/dev/ttyACM0")
uint32 bitrate          # 現在のビットレート (例: 1000000)
string error_message    # 接続失敗時や切断時のエラーメッセージ
