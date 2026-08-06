from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # CAN ブリッジ (物理 CAN <-> ROS2)
        Node(
            package='altair_can_bridge',
            executable='can_bridge_node',
            name='can_bridge_node',
            output='screen',
        ),

        # CAN 制御ノード (パラメータ送信・速度指令・フィードバック)
        Node(
            package='Robowarepkg',
            executable='can_node',
            name='can_node',
            output='screen',
        ),

        # RealSense カメラ (人物検出・距離計測)
        Node(
            package='Robowarepkg',
            executable='RealSense_node',
            name='RealSense_node',
            output='screen',
        ),

        # 追従制御演算 (MPN)
        Node(
            package='Robowarepkg',
            executable='Roboware_node',
            name='Roboware_node',
            output='screen',
        ),

        # WebSocket / GUI サーバー
        Node(
            package='Robowarepkg',
            executable='web_socket_node',
            name='web_socket_node',
            output='screen',
        ),

        # 顔アニメーション
        Node(
            package='Robowarepkg',
            executable='FaceAnimation_node',
            name='FaceAnimation_node',
            output='screen',
        ),
    ])
