# MDD (モータドライバ) のフィードバック情報メッセージ定義

bool[4] limit_switches  # Limit SW1 - SW4 の状態 (True = ON / 押されている, False = OFF)
uint8 error_code        # システムエラーコード (0:正常, 1:初期化タイムアウト, 2:CAN受信タイムアウト, 4:送信失敗)
uint8 system_status     # システム動作状態 (0: パラメータ設定モード, 1: 制御実行モード)

float32[4] angles       # モータ1〜4の現在角度 [degree] (生データの 0.1 deg 単位を変換)
float32[4] speeds       # モータ1〜4の現在回転速度 [rps] (生データの 0.01 rps 単位を変換)
