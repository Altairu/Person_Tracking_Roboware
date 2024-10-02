
# Robowarepkg - README

## 概要

`Robowarepkg`は、ROS 2を使用して二輪移動ロボットを制御するパッケージです。このパッケージは、WebSocketを介してコントローラーから速度指令を受け取り、二輪の運動学に基づいて左右の車輪速度（回転数、RPS: Revolutions Per Second）を計算し、シリアル通信を通じてマイコンに送信します。また、マイコンからの車輪速度情報（RPS）を受け取り、ロボットの自己位置を推定します。自己位置の推定には、二輪の運動学モデルを利用しています。

---

## システム構成

### ノード構成

`Robowarepkg`は、以下のノードで構成されています：

1. **`web_socket_node`**:
    - WebSocket通信を介して外部コントローラーから速度指令（直進速度V[mm/s]と角速度ω[rad/s]）を受け取り、それらをROSトピックにパブリッシュします。
    - FastAPIを使用したWebSocketサーバーが起動し、ジョイスティックのUIを通じて指令が送信されます。

2. **`Roboware_node`**:
    - `web_socket_node`から送られた速度指令（V[mm/s], ω[rad/s]）に基づいて、左右の車輪の回転速度（RPS）を計算します。
    - 計算されたRPSは、シリアル通信を介してマイコンに送信するためのトピック`wheel_rps`にパブリッシュされます。

3. **`serial_send_node`**:
    - `Roboware_node`で計算されたRPSをマイコンにシリアル通信で送信します。データフォーマットは以下の通りです。
    
4. **`serial_read_node`**:
    - マイコンから受け取った左右の車輪の回転数（RPS）をROSトピック`robot_position`にパブリッシュします。

5. **`position_node`**:
    - マイコンからのRPSを基に、自己位置を推定します。推定された位置はROSトピック`estimated_position`にパブリッシュされ、外部システムやモニタリングに使用されます。

---

## マイコンとの通信仕様

### 送信データ形式 (PCからマイコンへ)
PCからマイコンに送信されるデータは、以下のフォーマットで構成されています。

- **ヘッダー**:
    - 0xA5, 0xA5（2バイト）
- **右車輪速度** (Vr [mm/s]):
    - float形式（4バイト）
- **左車輪速度** (Vl [mm/s]):
    - float形式（4バイト）

合計 **10バイト** のデータがマイコンに送信されます。

### 受信データ形式 (マイコンからPCへ)
マイコンからPCに送信されるデータは以下のフォーマットです。

- **ヘッダー**:
    - 0xA5, 0xA5（2バイト）
- **右車輪の回転数** (ωr [RPS]):
    - float形式（4バイト）
- **左車輪の回転数** (ωl [RPS]):
    - float形式（4バイト）

合計 **10バイト** のデータがPCに送信されます。

---

## ノード詳細とトピック

### 1. `web_socket_node`
- **トピック**:
    - `/web_socket_pub`（`String`）：外部からの速度指令をパブリッシュ。
    - `/estimated_position`（`Float32MultiArray`）：推定された自己位置データを受信してWebSocketを通じてクライアントに返す。

### 2. `Roboware_node`
- **トピック**:
    - `/web_socket_pub`（`String`）：WebSocketからの速度指令をサブスクライブ。
    - `/wheel_rps`（`Float32MultiArray`）：計算された左右の車輪速度（RPS）をパブリッシュ。

### 3. `serial_send_node`
- **トピック**:
    - `/wheel_rps`（`Float32MultiArray`）：ロボットに送信するRPSデータをサブスクライブ。

### 4. `serial_read_node`
- **トピック**:
    - `/robot_position`（`Float32MultiArray`）：マイコンからの左右の車輪RPSデータをパブリッシュ。

### 5. `position_node`
- **トピック**:
    - `/robot_position`（`Float32MultiArray`）：受信した車輪の回転数（RPS）をサブスクライブ。
    - `/estimated_position`（`Float32MultiArray`）：自己位置推定結果をパブリッシュ。

---

## 使用方法

### 1. 環境のセットアップ
1. **ROS 2** (FoxyやHumbleなど)をインストールして、作業環境をセットアップします。
2. 必要な依存パッケージをインストールします。特に、`FastAPI`、`Uvicorn`、`serial`などが必要です。

```bash
pip install fastapi uvicorn pyserial
```

### 2. パッケージのビルド
ROS 2ワークスペースでパッケージをビルドします。

```bash
colcon build --packages-select robowarepkg
```

### 3. 実行
ROS 2ノードとFastAPIサーバーを実行するために以下のコマンドを実行します。

```bash
ros2 run robowarepkg web_socket_node
```

このコマンドにより、WebSocketサーバーとROS 2ノードが同時に実行されます。

ブラウザから`http://<ip_address>:8080/`にアクセスすると、ジョイスティックUIを操作する画面が表示されます。

---

## 通信内容

### WebSocket通信
WebSocket通信を使用して、クライアント（ブラウザなど）からロボットの速度指令を送信します。具体的には、ジョイスティック操作によって生成された速度指令`V`（mm/s）と角速度`ω`（rad/s）が、次のフォーマットでサーバーに送信されます。

- `V, ω`
    - 例: `500, 0.1`（前進速度500 mm/s、角速度0.1 rad/s）

### シリアル通信
上記で述べたフォーマットに基づいて、PCからマイコンへと速度指令を送信し、マイコンから車輪の回転数（RPS）を受け取ります。

---

## rqtによるモニタリング

`rqt`を使用して、ROSトピックのモニタリングやデバッグを行うことができます。特に、自己位置推定データや車輪の回転数を視覚的に確認するのに役立ちます。

### 1. rqt_graph
ROSのトピックとノードの関係を視覚化するために`rqt_graph`を使います。

```bash
rqt_graph
```

### 2. rqt_plot
`rqt_plot`を使用して、車輪の回転数や自己位置のデータをリアルタイムにプロットすることができます。

```bash
rqt_plot /estimated_position[0] /estimated_position[1]
```

これにより、x座標（前進方向）とy座標（横方向）の推定位置がリアルタイムでプロットされます。
