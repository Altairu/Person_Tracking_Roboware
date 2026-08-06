#!/usr/bin/env python3
"""
ethernet_can_bridge.py

Ethernet-CAN モジュール(STM32/W5500)と Linux の vcan インタフェース(can0)を
双方向でブリッジするスクリプト。

STM32 は接続中は常時 CAN データを送り続けるため、
送信スレッドと受信スレッドは独立した非同期構成で動かす。

通常起動:
  python3 ethernet_can_bridge.py [IP] [PORT] [--mode multinode|analyzer]

WSL/テスト用 (vcan を使わず受信フレームを標準出力に表示するだけ):
  python3 ethernet_can_bridge.py [IP] [PORT] [--mode multinode|analyzer] --dry-run
"""

import argparse
import socket
import struct
import subprocess
import sys
import threading
import time

# SocketCAN フレームのフォーマット: can_id(4B) + dlc(1B) + pad(3B) + data(8B) = 16B
SOCKETCAN_FRAME_FMT = "=IB3x8s"
SOCKETCAN_FRAME_SIZE = struct.calcsize(SOCKETCAN_FRAME_FMT)

# Ethernet-CAN パケットサイズ
PACKET_SIZE_MULTINODE = 100
PACKET_SIZE_ANALYZER = 20

# スロットあたりのバイト数 (MultiNode プロトコル)
SLOT_SIZE = 10

# vcan インタフェース名 (can_bridge_node の USB モードと同一にする)
IFACE_NAME = "can0"

# 再接続間隔(秒)
RECONNECT_INTERVAL = 2.0


# --- vcan セットアップ ---

def setup_vcan(iface: str) -> bool:
    """vcan モジュールをロードして仮想 CAN インタフェースを作成する。"""
    ret = subprocess.call(["sudo", "modprobe", "vcan"])
    if ret != 0:
        print(f"[ERROR] vcan モジュールのロードに失敗しました (exit={ret})", flush=True)
        return False

    subprocess.call(
        ["sudo", "ip", "link", "delete", iface],
        stderr=subprocess.DEVNULL,
    )

    ret = subprocess.call(["sudo", "ip", "link", "add", "dev", iface, "type", "vcan"])
    if ret != 0:
        print(f"[ERROR] {iface} の作成に失敗しました (exit={ret})", flush=True)
        return False

    ret = subprocess.call(["sudo", "ip", "link", "set", "up", iface])
    if ret != 0:
        print(f"[ERROR] {iface} の起動に失敗しました (exit={ret})", flush=True)
        return False

    print(f"[INFO] {iface} (vcan) を起動しました。", flush=True)
    return True


def teardown_vcan(iface: str):
    """vcan インタフェースを削除する。"""
    subprocess.call(
        ["sudo", "ip", "link", "delete", iface],
        stderr=subprocess.DEVNULL,
    )
    print(f"[INFO] {iface} を削除しました。", flush=True)


def open_can_socket(iface: str) -> socket.socket:
    """SocketCAN の RAW ソケットを開いて指定インタフェースにバインドする。"""
    PF_CAN = 29
    CAN_RAW = 1
    sock = socket.socket(PF_CAN, socket.SOCK_RAW, CAN_RAW)
    sock.bind((iface,))
    sock.settimeout(1.0)
    return sock


# --- プロトコル変換 ---

def pack_can_to_multinode_offset(can_id: int) -> int:
    """CAN ID から MultiNode バッファ内の CAN1 スロット先頭オフセットを返す。"""
    if 0x100 <= can_id <= 0x1FF:
        return 0
    elif 0x200 <= can_id <= 0x3FF:
        return 10
    elif 0x400 <= can_id <= 0x4FF:
        return 20
    elif 0x500 <= can_id <= 0x5FF:
        return 30
    elif 0x600 <= can_id <= 0x6FF:
        return 40
    return 0


def multinode_to_can_frames(packet: bytes):
    """MultiNode 100B パケットを CAN フレームのリストに変換する。"""
    frames = []
    for slot in range(10):
        offset = slot * SLOT_SIZE
        can_id = (packet[offset] << 8) | packet[offset + 1]
        if can_id == 0:
            continue
        data = bytes(packet[offset + 2: offset + 10])
        is_can2 = slot >= 5
        frames.append((can_id, data, is_can2))
    return frames


def analyzer_to_can_frames(packet: bytes):
    """Analyzer 20B パケットを CAN フレームのリストに変換する。"""
    frames = []
    can1_id = (packet[0] << 8) | packet[1]
    if can1_id != 0:
        frames.append((can1_id, bytes(packet[2:10]), False))
    can2_id = (packet[10] << 8) | packet[11]
    if can2_id != 0:
        frames.append((can2_id, bytes(packet[12:20]), True))
    return frames


# --- ブリッジ本体 ---

class EthernetCanBridge:
    def __init__(self, ip: str, port: int, mode: str, dry_run: bool = False):
        self.ip = ip
        self.port = port
        self.mode = mode
        self.dry_run = dry_run
        self.packet_size = PACKET_SIZE_MULTINODE if mode == "multinode" else PACKET_SIZE_ANALYZER

        self.tcp_sock = None
        self.can_sock = None
        self.running = False

        # MultiNode 送信バッファ (100B 固定)
        self.tx_buffer = bytearray(PACKET_SIZE_MULTINODE)
        self.tx_lock = threading.Lock()

    def connect_tcp(self) -> bool:
        """Ethernet-CAN モジュールへ TCP 接続する。"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((self.ip, self.port))
            sock.settimeout(1.0)
            self.tcp_sock = sock
            print(f"[INFO] TCP 接続完了: {self.ip}:{self.port}", flush=True)
            return True
        except Exception as e:
            print(f"[WARN] TCP 接続失敗: {e}", flush=True)
            return False

    def disconnect_tcp(self):
        if self.tcp_sock:
            try:
                self.tcp_sock.close()
            except Exception:
                pass
            self.tcp_sock = None

    # --- TCP 受信スレッド: STM32 -> SocketCAN/stdout ---

    def tcp_recv_thread(self):
        """TCP から 100B パケットを受信して CAN フレームに変換するスレッド。"""
        buf = bytearray()
        frame_count = 0

        while self.running:
            if self.tcp_sock is None:
                time.sleep(0.1)
                continue
            try:
                chunk = self.tcp_sock.recv(512)
                if not chunk:
                    print("[WARN] TCP 切断。再接続します...", flush=True)
                    self.disconnect_tcp()
                    time.sleep(RECONNECT_INTERVAL)
                    self.connect_tcp()
                    buf.clear()
                    continue
                buf.extend(chunk)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[WARN] TCP 受信エラー: {e}", flush=True)
                self.disconnect_tcp()
                time.sleep(RECONNECT_INTERVAL)
                self.connect_tcp()
                buf.clear()
                continue

            # packet_size バイト単位で処理
            while len(buf) >= self.packet_size:
                packet = bytes(buf[: self.packet_size])
                del buf[: self.packet_size]

                if self.mode == "multinode":
                    frames = multinode_to_can_frames(packet)
                else:
                    frames = analyzer_to_can_frames(packet)

                for can_id, data, is_can2 in frames:
                    # CAN2 は使用しないためスキップ
                    if is_can2:
                        continue
                    # データが全ゼロのフレームはノイズ/スタレデータとして無視
                    if all(b == 0 for b in data):
                        continue
                    frame_count += 1
                    if self.dry_run:
                        hex_data = " ".join(f"{b:02X}" for b in data)
                        print(
                            f"[DRY-RUN] #{frame_count:05d} CAN1 ID=0x{can_id:03X}  [{hex_data}]",
                            flush=True,
                        )
                    else:
                        self._write_can_frame(can_id, data)

    # --- TCP 送信スレッド: SocketCAN/polling -> STM32 ---

    def tcp_send_thread(self):
        """STM32 へ定期的にパケットを送信するスレッド。
        干渉を避けるため一定間隔でゼロパケット(dry-run)または
        tx_buffer の内容(通常モード)を送り続ける。"""
        empty = bytes(self.packet_size)
        while self.running:
            if self.tcp_sock is None:
                time.sleep(0.1)
                continue
            try:
                if self.dry_run:
                    self.tcp_sock.sendall(empty)
                else:
                    with self.tx_lock:
                        self.tcp_sock.sendall(bytes(self.tx_buffer))
            except Exception as e:
                if self.running:
                    print(f"[WARN] TCP 送信エラー: {e}", flush=True)
            # STM32 が溢れないよう小さな待機を入れる
            time.sleep(0.005)

    # --- SocketCAN 受信スレッド (通常モードのみ): vcan -> tx_buffer ---

    def can_rx_thread(self):
        """vcan0 から受信した CAN フレームを tx_buffer へ反映するスレッド。"""
        while self.running:
            if self.can_sock is None:
                time.sleep(0.1)
                continue
            try:
                raw = self.can_sock.recv(SOCKETCAN_FRAME_SIZE)
                if len(raw) < SOCKETCAN_FRAME_SIZE:
                    continue
                can_id, dlc, data_raw = struct.unpack(SOCKETCAN_FRAME_FMT, raw)
                can_id = can_id & 0x1FFFFFFF  # EFF/ERR フラグを除去
                data = data_raw[:min(dlc, 8)]
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[WARN] SocketCAN 読み取りエラー: {e}", flush=True)
                continue

            with self.tx_lock:
                offset = pack_can_to_multinode_offset(can_id)
                self.tx_buffer[offset] = (can_id >> 8) & 0xFF
                self.tx_buffer[offset + 1] = can_id & 0xFF
                padded = bytes(data).ljust(8, b"\x00")
                self.tx_buffer[offset + 2: offset + 10] = padded

    def _write_can_frame(self, can_id: int, data: bytes):
        """vcan0 へ CAN フレームを書き込む。"""
        if self.can_sock is None:
            return
        try:
            dlc = min(len(data), 8)
            padded = data[:dlc].ljust(8, b"\x00")
            frame = struct.pack(SOCKETCAN_FRAME_FMT, can_id, dlc, padded)
            self.can_sock.send(frame)
        except Exception as e:
            if self.running:
                print(f"[WARN] SocketCAN 書き込みエラー: {e}", flush=True)

    def run(self):
        """ブリッジのメインループ。"""
        self.running = True

        if not self.dry_run:
            try:
                self.can_sock = open_can_socket(IFACE_NAME)
            except Exception as e:
                print(f"[ERROR] SocketCAN {IFACE_NAME} のオープンに失敗しました: {e}", flush=True)
                sys.exit(1)
        else:
            print("[DRY-RUN] vcan / SocketCAN は使用しません。受信フレームを標準出力に表示します。", flush=True)

        self.connect_tcp()

        # 受信スレッド (TCP -> CAN/stdout)
        threading.Thread(target=self.tcp_recv_thread, daemon=True).start()
        # 送信スレッド (CAN/polling -> TCP)
        threading.Thread(target=self.tcp_send_thread, daemon=True).start()
        # 通常モードのみ: vcan -> tx_buffer 更新スレッド
        if not self.dry_run:
            threading.Thread(target=self.can_rx_thread, daemon=True).start()

        label = "DRY-RUN" if self.dry_run else IFACE_NAME
        print(
            f"[INFO] ブリッジ起動完了: {self.ip}:{self.port} <-> {label} (モード: {self.mode})",
            flush=True,
        )

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("[INFO] 終了します...", flush=True)
        finally:
            self.running = False
            self.disconnect_tcp()
            if self.can_sock:
                self.can_sock.close()


def main():
    parser = argparse.ArgumentParser(
        description="Ethernet-CAN モジュールと SocketCAN(vcan) のブリッジ"
    )
    parser.add_argument(
        "ip", nargs="?", default="192.168.2.123",
        help="Ethernet-CAN モジュールの IP アドレス (default: 192.168.2.123)"
    )
    parser.add_argument(
        "port", nargs="?", type=int, default=5000,
        help="TCP ポート番号 (default: 5000)"
    )
    parser.add_argument(
        "--mode", choices=["multinode", "analyzer"], default="multinode",
        help="通信モード: multinode=100B / analyzer=20B (default: multinode)"
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="vcan を使わず受信フレームを標準出力に表示するだけ (WSL/テスト用)"
    )
    args = parser.parse_args()

    if args.dry_run:
        print(f"[DRY-RUN] モード: {args.mode}  接続先: {args.ip}:{args.port}", flush=True)
        bridge = EthernetCanBridge(args.ip, args.port, args.mode, dry_run=True)
        bridge.run()
    else:
        if not setup_vcan(IFACE_NAME):
            sys.exit(1)
        bridge = EthernetCanBridge(args.ip, args.port, args.mode, dry_run=False)
        try:
            bridge.run()
        finally:
            teardown_vcan(IFACE_NAME)


if __name__ == "__main__":
    main()
