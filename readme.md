# **Person_Tracking_Roboware　Robowarepkg README**

## **概要**

**Robowarepkg**は、人の追跡や制御を行うロボットアプリケーションパッケージです。
ROS 2を基盤とし、RealSenseカメラによるデータ取得、PID制御、シリアル通信、WebSocket通信などの機能を組み合わせて動作している。


## **システム構成**

### ノード一覧
- **Roboware_node**
  - システムの中心ノードで、各モードの切り替えやホイール速度の制御を行います。
- **Roboware_node_mpn / Roboware_node_pn / Roboware_node_newmpn**
  - 制御式が異なるバージョンを記録するテスト用ノード。
  - `Roboware_node`に統合して使用可能です。
- **PID_node**
  - PID制御を用いてホイール速度を制御します。
- **RealSense_node**
  - RealSenseカメラで得たデータをもとに人の位置を計測します。
- **serial_send_node / serial_read_node**
  - マイコンとのシリアル通信を担当します。
- **web_socket_node**
  - WebSocket通信で外部アプリケーションとリアルタイム通信を行います。

---

## **通信仕様**

### **マイコンとの通信**
#### **送信データ形式**（PC → マイコン）
- **ヘッダー**: `0xA5, 0xA5`（2バイト）
- **右車輪速度 (Vr)**: `float`形式（4バイト）
- **左車輪速度 (Vl)**: `float`形式（4バイト）
- **合計**: 10バイト

#### **受信データ形式**（マイコン → PC）
- **ヘッダー**: `0xA5, 0xA5`（2バイト）
- **右車輪回転数 (ωr)**: `float`形式（4バイト）
- **左車輪回転数 (ωl)**: `float`形式（4バイト）
- **合計**: 10バイト

### **WebSocket通信**
- **IPアドレス**: `192.168.62.212`
- **ポート番号**: `8080`
- **コマンドフォーマット**: `[mode, rx, ry, lx, ly, stop]`
  - `mode`: 0（手動操作）または1（追跡モード）
  - `stop`: 1（緊急停止）または0（動作中）

---

## **ノード詳細とトピック**

### Roboware_node
- **購読トピック**: 
  - `web_socket_pub` (String): WebSocket通信からのモード切り替え指示。
  - `camera_data` (Float32MultiArray): RealSenseノードからのデータ。
- **発行トピック**:
  - `wheel_targets` (Float32MultiArray): 各ホイールの速度指示。

### PID_node
- **購読トピック**:
  - `wheel_targets` (Float32MultiArray): ホイール目標速度。
  - `wheel_feedback` (Float32MultiArray): 実際のホイール速度。
- **発行トピック**:
  - `wheel_controls` (Float32MultiArray): シリアル送信用の制御信号。

### RealSense_node
- **発行トピック**:
  - `camera_data` (Float32MultiArray): 人物の距離とオフセット。

### serial_send_node
- **購読トピック**:
  - `wheel_controls` (Float32MultiArray): PIDノードからの制御信号。

### serial_read_node
- **発行トピック**:
  - `wheel_feedback` (Float32MultiArray): 実際のホイール速度。

### web_socket_node
- **購読トピック**:
  - `estimated_position` (Float32MultiArray): 推定位置データ。
- **発行トピック**:
  - `web_socket_pub` (String): WebSocket通信で受信したコマンド。

---

## **使用方法**

### 1. **セットアップ**
```bash
colcon build
source install/setup.bash
```

### 2. **ノード起動例**
```bash
ros2 run Robowarepkg Roboware_node
ros2 run Robowarepkg PID_node
ros2 run Robowarepkg RealSense_node
ros2 run Robowarepkg serial_send_node
ros2 run Robowarepkg serial_read_node
ros2 run Robowarepkg web_socket_node
```

---

## **WebSocketノードの使用方法**

1. **HTMLファイルの利用**:
   `/home/altair/Roboware/UI.txt` をブラウザで開くことでロボット操作が可能。
   - **URL**: `http://192.168.62.212:8080`
2. **モード切り替え**:
   - ボタンを押して`Control`モードと`Follow`モードを切り替えます。
3. **接続再試行**:
   - 接続が切れると5秒間隔で自動再接続を試みます。

---

## **rqtによるモニタリング**

### 1. **rqt_graph**
ノード間の接続状況を視覚化：
```bash
rqt_graph
```

### 2. **rqt_plot**
トピックデータをリアルタイムでプロット：
```bash
rqt_plot /wheel_feedback[0] /wheel_feedback[1]
```

---

## **注意事項**

- IPアドレスとポート番号は環境に合わせて変更してください。
- ノードを起動する前にすべての依存パッケージをインストールしてください。
