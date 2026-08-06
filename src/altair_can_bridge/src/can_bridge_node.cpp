// C++ 標準ヘッダー
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <filesystem>
#include <iostream>
#include <cstdlib>
#include <cerrno>
#include <termios.h>
// Linux SocketCAN ヘッダー
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// ROS2 ヘッダー
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "altair_interfaces/msg/can_status.hpp"
#include "altair_interfaces/srv/get_available_ports.hpp"
#include "altair_interfaces/srv/connect_can.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class CanBridgeNode : public rclcpp::Node
{
public:
  CanBridgeNode()
  : Node("can_bridge_node"), is_connected_(false), can_socket_(-1), multinode_tx_buffer_(100, 0)
  {
    // パラメータの宣言と取得
    this->declare_parameter<std::string>("connection_mode", "usb");
    this->declare_parameter<std::string>("ethernet_ip", "192.168.2.123");
    this->declare_parameter<int>("ethernet_port", 5000);
    this->declare_parameter<int>("ethernet_tx_size", 10);

    this->get_parameter("connection_mode", connection_mode_);
    this->get_parameter("ethernet_ip", ethernet_ip_);
    this->get_parameter("ethernet_port", ethernet_port_);
    this->get_parameter("ethernet_tx_size", ethernet_tx_size_);
    // ROS2パブリッシャ・サブスクライバの初期化
    can_rx_pub_ = this->create_publisher<can_msgs::msg::Frame>("/altair/can/rx", 100);
    can_status_pub_ = this->create_publisher<altair_interfaces::msg::CanStatus>("/altair/can/status", 10);
    
    can_tx_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      "/altair/can/tx", 100,
      std::bind(&CanBridgeNode::handle_can_tx_callback, this, std::placeholders::_1));

    // ROS2サービスの初期化
    get_ports_srv_ = this->create_service<altair_interfaces::srv::GetAvailablePorts>(
      "/altair/can/get_ports",
      std::bind(&CanBridgeNode::handle_get_ports_service, this, std::placeholders::_1, std::placeholders::_2));

    connect_can_srv_ = this->create_service<altair_interfaces::srv::ConnectCan>(
      "/altair/can/connect",
      std::bind(&CanBridgeNode::handle_connect_can_service, this, std::placeholders::_1, std::placeholders::_2));

    // 接続ステータス配信用のタイマー (1Hz)
    status_timer_ = this->create_wall_timer(
      1000ms, std::bind(&CanBridgeNode::publish_status, this));

    // CAN受信スレッドの起動
    rx_thread_running_ = true;
    rx_thread_ = std::thread(&CanBridgeNode::rx_thread_loop, this);

    RCLCPP_INFO(this->get_logger(), "Altair CAN Bridge Node が起動しました。接続待ち状態です。");

    // 自動接続・再接続スレッドの起動
    auto_connect_running_ = true;
    auto_connect_thread_ = std::thread(&CanBridgeNode::auto_connect_loop, this);
  }

  ~CanBridgeNode()
  {
    auto_connect_running_ = false;
    if (auto_connect_thread_.joinable()) {
      auto_connect_thread_.join();
    }
    rx_thread_running_ = false;
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    close_can_socket();
  }

private:
  // --- 1. ROS2トピック ➔ 物理CAN (TX) ---
  void handle_can_tx_callback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (!is_connected_ || can_socket_ < 0) {
      // 接続されていない場合は黙って無視
      return;
    }

    if (connection_mode_ == "ethernet") {
      this->get_parameter("ethernet_tx_size", ethernet_tx_size_);
      
      if (ethernet_tx_size_ == 100) {
        // MultiNode モード (100バイト)
        bool is_can2 = (msg->header.frame_id == "can1" || msg->header.frame_id == "can2");
        int base_offset = 0;
        
        // ID に応じてスロット（基本オフセット）を判定
        uint32_t raw_id = msg->id;
        if (raw_id >= 0x100 && raw_id <= 0x1FF) {
          base_offset = 0;
        } else if (raw_id >= 0x200 && raw_id <= 0x3FF) {
          base_offset = 10;
        } else if (raw_id >= 0x400 && raw_id <= 0x4FF) {
          base_offset = 20;
        } else if (raw_id >= 0x500 && raw_id <= 0x5FF) {
          base_offset = 30;
        } else if (raw_id >= 0x600 && raw_id <= 0x6FF) {
          base_offset = 40;
        } else {
          base_offset = 0;
        }
        
        int offset = base_offset + (is_can2 ? 50 : 0);
        
        // 送信キャッシュバッファを更新
        multinode_tx_buffer_[offset + 0] = (raw_id >> 8) & 0xFF;
        multinode_tx_buffer_[offset + 1] = raw_id & 0xFF;
        size_t copy_len = std::min(static_cast<size_t>(msg->dlc), static_cast<size_t>(8));
        std::copy(msg->data.begin(), msg->data.begin() + copy_len, multinode_tx_buffer_.begin() + offset + 2);
        // 残りデータ領域を0埋め (DLCが8未満の時のため)
        std::fill(multinode_tx_buffer_.begin() + offset + 2 + copy_len, multinode_tx_buffer_.begin() + offset + 10, 0);

        int bytes_sent = write(can_socket_, multinode_tx_buffer_.data(), 100);
        if (bytes_sent < 0) {
          if (errno == ENOBUFS || errno == EAGAIN || errno == EWOULDBLOCK) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
              "TCP送信バッファ満杯 (ENOBUFS/EAGAIN) を検知しました。パケットをスキップします。");
            return;
          }
          RCLCPP_ERROR(this->get_logger(), "TCPでのCANフレーム送信(100B)に失敗しました: %s", strerror(errno));
          handle_disconnect("TCPでのCANフレーム送信失敗(100B)");
        }
      } 
      else if (ethernet_tx_size_ == 20) {
        // Analyzer モード (20バイト)
        std::vector<uint8_t> buf(20, 0);
        bool is_can2 = (msg->header.frame_id == "can1" || msg->header.frame_id == "can2");
        int offset = is_can2 ? 10 : 0;
        
        buf[offset + 0] = (msg->id >> 8) & 0xFF;
        buf[offset + 1] = msg->id & 0xFF;
        size_t copy_len = std::min(static_cast<size_t>(msg->dlc), static_cast<size_t>(8));
        std::copy(msg->data.begin(), msg->data.begin() + copy_len, buf.begin() + offset + 2);
        
        int bytes_sent = write(can_socket_, buf.data(), 20);
        if (bytes_sent < 0) {
          if (errno == ENOBUFS || errno == EAGAIN || errno == EWOULDBLOCK) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
              "TCP送信バッファ満杯 (ENOBUFS/EAGAIN) を検知しました。パケットをスキップします。");
            return;
          }
          RCLCPP_ERROR(this->get_logger(), "TCPでのCANフレーム送信(20B)に失敗しました: %s", strerror(errno));
          handle_disconnect("TCPでのCANフレーム送信失敗(20B)");
        }
      }
      else {
        // 10バイトモード
        std::vector<uint8_t> buf(10, 0);
        buf[0] = (msg->id >> 8) & 0xFF;
        buf[1] = msg->id & 0xFF;
        size_t copy_len = std::min(static_cast<size_t>(msg->dlc), static_cast<size_t>(8));
        std::copy(msg->data.begin(), msg->data.begin() + copy_len, buf.begin() + 2);
        
        int bytes_sent = write(can_socket_, buf.data(), buf.size());
        if (bytes_sent < 0) {
          if (errno == ENOBUFS || errno == EAGAIN || errno == EWOULDBLOCK) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
              "TCP送信バッファ満杯 (ENOBUFS/EAGAIN) を検知しました。パケットをスキップします。");
            return;
          }
          RCLCPP_ERROR(this->get_logger(), "TCPでのCANフレーム送信に失敗しました: %s", strerror(errno));
          handle_disconnect("TCPでのCANフレーム送信失敗");
        }
      }
    } else {
      struct can_frame frame;
      memset(&frame, 0, sizeof(frame));

      frame.can_id = msg->id;
      if (msg->is_rtr) {
        frame.can_id |= CAN_RTR_FLAG;
      }
      if (msg->is_extended) {
        frame.can_id |= CAN_EFF_FLAG;
      }
      frame.can_dlc = msg->dlc;
      memcpy(frame.data, msg->data.data(), msg->dlc);

      // SocketCANに書き込み
      int bytes_sent = write(can_socket_, &frame, sizeof(struct can_frame));
      if (bytes_sent < 0) {
        if (errno == ENOBUFS || errno == EAGAIN || errno == EWOULDBLOCK) {
          // 送信バッファ一時満杯などの軽微なエラー時は切断せずパケットをスキップ
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "CAN送信バッファ満杯 (ENOBUFS/EAGAIN) を検知しました。パケットをスキップします。");
          return;
        }
        RCLCPP_ERROR(this->get_logger(), "CANフレームの送信に失敗しました: %s", strerror(errno));
        // 致命的な通信エラーのみ切断状態に移行
        handle_disconnect("CANフレーム送信失敗による通信切断");
      }
    }
  }

  // --- 2. 物理CAN ➔ ROS2トピック (RXスレッド) ---
  void rx_thread_loop()
  {
    struct can_frame frame;
    std::vector<uint8_t> tcp_rx_buf;

    while (rx_thread_running_ && rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (!is_connected_ || can_socket_ < 0) {
          // 未接続時はスリープして待機
          socket_mutex_.unlock();
          std::this_thread::sleep_for(100ms);
          socket_mutex_.lock();
          continue;
        }
      }

      // ソケットのポーリング設定（ブロッキングを避けるためタイムアウトを設定）
      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 100000; // 100ms
      
      fd_set rdfs;
      FD_ZERO(&rdfs);
      FD_SET(can_socket_, &rdfs);

      int ret = select(can_socket_ + 1, &rdfs, NULL, NULL, &tv);
      if (ret < 0) {
        if (errno == EINTR) continue;
        RCLCPP_ERROR(this->get_logger(), "ソケットの選択中にエラーが発生しました: %s", strerror(errno));
        handle_disconnect("ソケット選択エラー");
        continue;
      }
      else if (ret == 0) {
        // タイムアウト、ループ継続
        continue;
      }

      // 接続モードに応じて受信処理を分岐
      if (connection_mode_ == "ethernet") {
        uint8_t temp_buf[256];
        int nbytes = read(can_socket_, temp_buf, sizeof(temp_buf));
        if (nbytes < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
          RCLCPP_ERROR(this->get_logger(), "TCPデータの読み取りに失敗しました: %s", strerror(errno));
          handle_disconnect("TCP読み取りエラー");
          continue;
        }
        else if (nbytes == 0) {
          RCLCPP_WARN(this->get_logger(), "TCP接続がリモートホストによって切断されました。");
          handle_disconnect("TCPリモート切断");
          continue;
        }

        // 受信バッファに追加
        tcp_rx_buf.insert(tcp_rx_buf.end(), temp_buf, temp_buf + nbytes);

        this->get_parameter("ethernet_tx_size", ethernet_tx_size_);
        size_t packet_size = ethernet_tx_size_;
        if (packet_size < 10) packet_size = 10;

        while (tcp_rx_buf.size() >= packet_size) {
          if (packet_size == 100) {
            // MultiNode モード (100バイト)
            for (int slot = 0; slot < 10; ++slot) {
              int offset = slot * 10;
              uint32_t can_id = (tcp_rx_buf[offset + 0] << 8) | tcp_rx_buf[offset + 1];
              if (can_id != 0) {
                auto rx_msg = std::make_shared<can_msgs::msg::Frame>();
                rx_msg->header.stamp = this->now();
                rx_msg->header.frame_id = (slot < 5) ? "can0" : "can1";
                
                rx_msg->is_extended = false;
                rx_msg->id = can_id;
                rx_msg->is_rtr = false;
                rx_msg->is_error = false;
                rx_msg->dlc = 8;
                
                std::copy(tcp_rx_buf.begin() + offset + 2, tcp_rx_buf.begin() + offset + 10, rx_msg->data.begin());
                can_rx_pub_->publish(*rx_msg);
              }
            }
          }
          else if (packet_size == 20) {
            // Analyzer モード (20バイト)
            // CAN1 (0 - 9B)
            uint32_t can1_id = (tcp_rx_buf[0] << 8) | tcp_rx_buf[1];
            if (can1_id != 0) {
              auto rx_msg = std::make_shared<can_msgs::msg::Frame>();
              rx_msg->header.stamp = this->now();
              rx_msg->header.frame_id = "can0";
              rx_msg->is_extended = false;
              rx_msg->id = can1_id;
              rx_msg->is_rtr = false;
              rx_msg->is_error = false;
              rx_msg->dlc = 8;
              std::copy(tcp_rx_buf.begin() + 2, tcp_rx_buf.begin() + 10, rx_msg->data.begin());
              can_rx_pub_->publish(*rx_msg);
            }
            // CAN2 (10 - 19B)
            uint32_t can2_id = (tcp_rx_buf[10] << 8) | tcp_rx_buf[11];
            if (can2_id != 0) {
              auto rx_msg = std::make_shared<can_msgs::msg::Frame>();
              rx_msg->header.stamp = this->now();
              rx_msg->header.frame_id = "can1";
              rx_msg->is_extended = false;
              rx_msg->id = can2_id;
              rx_msg->is_rtr = false;
              rx_msg->is_error = false;
              rx_msg->dlc = 8;
              std::copy(tcp_rx_buf.begin() + 12, tcp_rx_buf.begin() + 20, rx_msg->data.begin());
              can_rx_pub_->publish(*rx_msg);
            }
          }
          else {
            // 10バイトモード
            uint32_t can_id = (tcp_rx_buf[0] << 8) | tcp_rx_buf[1];
            if (can_id != 0) {
              auto rx_msg = std::make_shared<can_msgs::msg::Frame>();
              rx_msg->header.stamp = this->now();
              rx_msg->header.frame_id = "can0";
              rx_msg->is_extended = false;
              rx_msg->id = can_id;
              rx_msg->is_rtr = false;
              rx_msg->is_error = false;
              rx_msg->dlc = 8;
              std::copy(tcp_rx_buf.begin() + 2, tcp_rx_buf.begin() + 10, rx_msg->data.begin());
              can_rx_pub_->publish(*rx_msg);
            }
          }

          // 処理分をバッファから削除
          tcp_rx_buf.erase(tcp_rx_buf.begin(), tcp_rx_buf.begin() + packet_size);
        }
      } else {
        // CANデータ読み取り
        int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
          RCLCPP_ERROR(this->get_logger(), "CANフレームの読み取りに失敗しました: %s", strerror(errno));
          handle_disconnect("CANフレーム読み取りエラー");
          continue;
        }

        if (nbytes < (int)sizeof(struct can_frame)) {
          RCLCPP_ERROR(this->get_logger(), "不正なサイズのCANフレームを受信しました。");
          continue;
        }

        // ROS2メッセージへ詰め替え
        auto rx_msg = std::make_shared<can_msgs::msg::Frame>();
        rx_msg->header.stamp = this->now();
        rx_msg->header.frame_id = "can0";
        
        rx_msg->is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
        rx_msg->id = rx_msg->is_extended ? (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);
        rx_msg->is_rtr = (frame.can_id & CAN_RTR_FLAG) != 0;
        rx_msg->is_error = (frame.can_id & CAN_ERR_FLAG) != 0;
        rx_msg->dlc = frame.can_dlc;
        
        std::copy(frame.data, frame.data + frame.can_dlc, rx_msg->data.begin());

        // パブリッシュ
        can_rx_pub_->publish(*rx_msg);
      }
    }
  }

  // --- TCP 接続処理 ---
  bool connect_ethernet(const std::string& ip, int port)
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    close_can_socket_nolock();
    is_connected_ = false;

    can_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "TCPソケットの作成に失敗しました: %s", strerror(errno));
      last_error_ = "TCPソケット作成失敗: " + std::string(strerror(errno));
      return false;
    }

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0) {
      RCLCPP_ERROR(this->get_logger(), "無効なIPアドレスです: %s", ip.c_str());
      last_error_ = "無効なIPアドレス: " + ip;
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    if (connect(can_socket_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "TCPサーバーへの接続に失敗しました: %s", strerror(errno));
      last_error_ = "TCP接続失敗: " + std::string(strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    // ノンブロッキングモードに設定
    int opts = fcntl(can_socket_, F_GETFL);
    if (opts >= 0) {
      fcntl(can_socket_, F_SETFL, opts | O_NONBLOCK);
    }

    is_connected_ = true;
    active_port_ = ip + ":" + std::to_string(port);
    last_error_ = "";
    RCLCPP_INFO(this->get_logger(), "ethernet-can (%s:%d) へのTCP接続が完了しました！", ip.c_str(), port);
    return true;
  }

  // --- 3. 接続処理コア ---
  bool trigger_connect(const std::string& req_port, bool auto_connect)
  {
    this->get_parameter("connection_mode", connection_mode_);
    this->get_parameter("ethernet_ip", ethernet_ip_);
    this->get_parameter("ethernet_port", ethernet_port_);

    if (connection_mode_ == "ethernet") {
      std::string ip = ethernet_ip_;
      int port = ethernet_port_;

      if (!req_port.empty() && req_port != "auto" && req_port != "None" && req_port.find("tty") == std::string::npos) {
        size_t colon_pos = req_port.find(":");
        if (colon_pos != std::string::npos) {
          ip = req_port.substr(0, colon_pos);
          try {
            port = std::stoi(req_port.substr(colon_pos + 1));
          } catch (...) {}
        } else {
          ip = req_port;
        }
      }
      return connect_ethernet(ip, port);
    }

    std::string port = req_port;

    if (auto_connect) {
      // 事前にslcandをキルしてシリアルポートを解放
      std::system("echo 'altair' | sudo -S killall slcand 2>/dev/null");
      std::this_thread::sleep_for(200ms);

      port = detect_can_port();
      if (port.empty()) {
        last_error_ = "自動検出に失敗しました";
        return false;
      }
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);
    
    // 一度既存のソケットを閉じる
    close_can_socket_nolock();
    is_connected_ = false;

    // パッケージシェアディレクトリ内のsetup_slcan.shのパスをROS2標準APIで取得
    std::string pkg_share_dir = "";
    try {
      pkg_share_dir = ament_index_cpp::get_package_share_directory("altair_can_bridge");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "パッケージシェアディレクトリの検出に失敗しました: %s", e.what());
      last_error_ = "パッケージ検出失敗";
      return false;
    }

    std::string script_path = pkg_share_dir + "/scripts/setup_slcan.sh";
    // もしインストールディレクトリに無ければ、カレントワークスペースのローカルパスも探す (デバッグ用)
    if (!fs::exists(script_path)) {
      script_path = "./src/altair_can_bridge/scripts/setup_slcan.sh";
    }
    if (!fs::exists(script_path)) {
      script_path = "./src/altair_framework/altair_can_bridge/scripts/setup_slcan.sh";
    }
    
    std::string cmd = "echo 'altair' | sudo -S bash " + script_path + " " + port + " 1000000";

    RCLCPP_INFO(this->get_logger(), "接続スクリプトを実行中: %s", cmd.c_str());
    
    // スクリプトの実行
    int exit_code = std::system(cmd.c_str());
    if (exit_code != 0) {
      RCLCPP_WARN(this->get_logger(), "接続スクリプトが非ゼロで終了しました (コード: %d)。すでに can0 が起動しているか、sudo パスワードが必要です。ソケットバインドを試行します...", exit_code);
    }

    // SocketCAN ソケットのオープン
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "SocketCAN ソケットの作成に失敗しました: %s", strerror(errno));
      last_error_ = "Socket作成失敗: " + std::string(strerror(errno));
      return false;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "インターフェース can0 の検出に失敗しました: %s", strerror(errno));
      last_error_ = "can0 検出失敗: " + std::string(strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // ソケットをバインド
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "SocketCAN can0 へのバインドに失敗しました: %s", strerror(errno));
      last_error_ = "can0 バインド失敗: " + std::string(strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    // ノンブロッキングモードに設定
    int opts = fcntl(can_socket_, F_GETFL);
    if (opts >= 0) {
      fcntl(can_socket_, F_SETFL, opts | O_NONBLOCK);
    }

    // 接続成功
    is_connected_ = true;
    active_port_ = port;
    last_error_ = "";
    RCLCPP_INFO(this->get_logger(), "SocketCAN can0 への接続が完了しました！");
    return true;
  }

  void handle_disconnect(const std::string& error_msg)
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    close_can_socket_nolock();
    is_connected_ = false;
    last_error_ = error_msg;
    RCLCPP_WARN(this->get_logger(), "CAN接続が切断されました: %s", error_msg.c_str());
  }

  void close_can_socket()
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    close_can_socket_nolock();
  }

  void close_can_socket_nolock()
  {
    if (can_socket_ >= 0) {
      close(can_socket_);
      can_socket_ = -1;
    }
  }

  // --- 4. サービスハンドラー ---
  void handle_get_ports_service(
    const std::shared_ptr<altair_interfaces::srv::GetAvailablePorts::Request> request,
    std::shared_ptr<altair_interfaces::srv::GetAvailablePorts::Response> response)
  {
    (void)request;
    std::vector<std::string> detected_ports;

    // /dev ディレクトリから ttyACM* と ttyUSB* を探索 (C++17 filesystem)
    try {
      if (fs::exists("/dev")) {
        for (const auto& entry : fs::directory_iterator("/dev")) {
          std::string name = entry.path().filename().string();
          if (name.find("ttyACM") == 0 || name.find("ttyUSB") == 0) {
            detected_ports.push_back(entry.path().string());
          }
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "ポート一覧スキャン中に例外が発生しました: %s", e.what());
    }

    // WSL上でデバッグしやすいようにモックポートも追加 (WSLで物理COMポートが見えない場合のテスト用)
    if (detected_ports.empty()) {
      detected_ports.push_back("/dev/ttyACM0 (Mock CANable2)");
      detected_ports.push_back("/dev/ttyACM1 (Mock USB-CAN)");
    }

    response->ports = detected_ports;
  }

  void handle_connect_can_service(
    const std::shared_ptr<altair_interfaces::srv::ConnectCan::Request> request,
    std::shared_ptr<altair_interfaces::srv::ConnectCan::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "CAN接続サービス要求を受信しました。");
    
    // 要求されたポートから接続モードを自動判定してパラメータを更新
    std::string req_port = request->port;
    if (req_port == "ethernet" || req_port.find(".") != std::string::npos || req_port.find(":") != std::string::npos) {
      this->set_parameter(rclcpp::Parameter("connection_mode", "ethernet"));
      if (req_port.find(".") != std::string::npos) {
        size_t colon_pos = req_port.find(":");
        if (colon_pos != std::string::npos) {
          std::string ip = req_port.substr(0, colon_pos);
          try {
            int port = std::stoi(req_port.substr(colon_pos + 1));
            this->set_parameter(rclcpp::Parameter("ethernet_port", port));
          } catch (...) {}
          this->set_parameter(rclcpp::Parameter("ethernet_ip", ip));
        } else {
          this->set_parameter(rclcpp::Parameter("ethernet_ip", req_port));
        }
      }
      this->get_parameter("connection_mode", connection_mode_);
      this->get_parameter("ethernet_ip", ethernet_ip_);
      this->get_parameter("ethernet_port", ethernet_port_);
    } else if (!request->auto_connect && !req_port.empty()) {
      this->set_parameter(rclcpp::Parameter("connection_mode", "usb"));
      this->get_parameter("connection_mode", connection_mode_);
    }

    if (is_connected_) {
      // 既に接続されている場合で、ポート指定が同じまたはauto_connectの場合は無視する
      std::string target_port = req_port;
      if (connection_mode_ == "ethernet" && target_port.find(".") != std::string::npos) {
        if (target_port.find(":") == std::string::npos) {
          target_port = target_port + ":" + std::to_string(ethernet_port_);
        }
      }
      if (request->auto_connect || target_port == active_port_) {
        RCLCPP_INFO(this->get_logger(), "既に接続されています。再接続をスキップします。");
        response->success = true;
        response->message = "既に接続されています。";
        return;
      }
    }

    // 非同期で接続プロセスを実行 (ブロッキングを避けるため)
    bool success = trigger_connect(request->port, request->auto_connect);
    
    response->success = success;
    if (success) {
      if (connection_mode_ == "ethernet") {
        response->message = "接続成功: ethernet-can (" + active_port_ + ") に接続しました。";
      } else {
        response->message = "接続成功: can0 が正常に起動しました。";
      }
    } else {
      response->message = "接続失敗: " + last_error_;
    }
  }

  // --- 6. 自動接続・デバイス検出 ---
  std::string detect_can_port()
  {
    std::vector<std::string> candidates;
    try {
      if (fs::exists("/dev")) {
        for (const auto& entry : fs::directory_iterator("/dev")) {
          std::string name = entry.path().filename().string();
          if (name.find("ttyACM") == 0 || name.find("ttyUSB") == 0) {
            candidates.push_back(entry.path().string());
          }
        }
      }
    } catch (...) {}

    for (const auto& port : candidates) {
      int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
      if (fd == -1) continue;

      struct termios options;
      tcgetattr(fd, &options);
      cfsetispeed(&options, B115200);
      cfsetospeed(&options, B115200);
      options.c_cflag |= (CLOCAL | CREAD);
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
      options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
      options.c_iflag &= ~(IXON | IXOFF | IXANY);
      options.c_oflag &= ~OPOST;
      
      // non-blocking off for read timeout
      fcntl(fd, F_SETFL, 0); 
      
      // Set read timeout to 0.1s
      options.c_cc[VMIN]  = 0;
      options.c_cc[VTIME] = 1;
      tcsetattr(fd, TCSANOW, &options);

      // リセット
      write(fd, "C\r", 2);
      std::this_thread::sleep_for(100ms);
      tcflush(fd, TCIFLUSH);
      
      // バージョン取得 (V\r) でデバイスを判定
      // CANable2 の一部ファームウェアは O\r に対する応答を返さないため、
      // 確実に応答がある V\r コマンドを使用する
      write(fd, "V\r", 2);
      std::this_thread::sleep_for(100ms);

      // 応答確認
      char buf[128];
      int n = read(fd, buf, sizeof(buf));
      
      // slcandで使えるように念のためクローズコマンド
      write(fd, "C\r", 2);
      std::this_thread::sleep_for(50ms);
      
      close(fd);

      if (n > 0) {
        std::string resp(buf, n);
        RCLCPP_INFO(this->get_logger(), "slcanデバイス検出: %s (応答=%s)", port.c_str(), resp.c_str());
        return port;
      }
    }
    return "";
  }

  void auto_connect_loop()
  {
    RCLCPP_INFO(this->get_logger(), "自動接続ループを開始します...");
    while (auto_connect_running_ && rclcpp::ok()) {
      // 最新のパラメータを取得
      this->get_parameter("connection_mode", connection_mode_);
      this->get_parameter("ethernet_ip", ethernet_ip_);
      this->get_parameter("ethernet_port", ethernet_port_);

      if (is_connected_) {
        std::this_thread::sleep_for(1000ms);
        continue;
      }

      if (connection_mode_ == "ethernet") {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
          "自動接続: ethernet-can (%s:%d) へのTCP接続を試みます...", ethernet_ip_.c_str(), ethernet_port_);
        trigger_connect("", false);
      } else {
        // 未接続時はまずslcandをキルしてポートを解放
        std::system("echo 'altair' | sudo -S killall slcand 2>/dev/null");
        std::this_thread::sleep_for(200ms);

        std::string port = detect_can_port();
        if (!port.empty()) {
          RCLCPP_INFO(this->get_logger(), "自動接続: %s に接続を試みます...", port.c_str());
          trigger_connect(port, false);
        } else {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "CANデバイスが見つかりません。再スキャンします...");
        }
      }
      std::this_thread::sleep_for(2000ms);
    }
  }

  // --- 5. 接続ステータス配信 ---
  void publish_status()
  {
    auto status_msg = altair_interfaces::msg::CanStatus();
    status_msg.is_connected = is_connected_;
    status_msg.active_port = is_connected_ ? active_port_ : "None";
    status_msg.bitrate = is_connected_ ? 1000000 : 0;
    status_msg.error_message = last_error_;

    can_status_pub_->publish(status_msg);
  }

  // メンバー変数
  std::atomic<bool> is_connected_;
  std::string active_port_;
  std::string last_error_;
  int can_socket_;
  std::mutex socket_mutex_;
  
  // Ethernet/TCP 接続設定
  std::string connection_mode_;
  std::string ethernet_ip_;
  int ethernet_port_;
  int ethernet_tx_size_;
  std::vector<uint8_t> multinode_tx_buffer_;

  std::thread rx_thread_;
  std::atomic<bool> rx_thread_running_;

  std::thread auto_connect_thread_;
  std::atomic<bool> auto_connect_running_;

  // ROS2 インターフェース
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_rx_pub_;
  rclcpp::Publisher<altair_interfaces::msg::CanStatus>::SharedPtr can_status_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_tx_sub_;
  
  rclcpp::Service<altair_interfaces::srv::GetAvailablePorts>::SharedPtr get_ports_srv_;
  rclcpp::Service<altair_interfaces::srv::ConnectCan>::SharedPtr connect_can_srv_;
  
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
