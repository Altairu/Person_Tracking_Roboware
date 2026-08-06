cmake_minimum_required(VERSION 3.8)
project(altair_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# カスタムメッセージファイルの登録
set(msg_files
  "msg/CanStatus.msg"
  "msg/MddFeedback.msg"
  "msg/MddCommand.msg"
  "msg/ServoCommand.msg"
  "msg/SolenoidCommand.msg"
)

# カスタムサービスファイルの登録
set(srv_files
  "srv/TriggerControl.srv"
  "srv/GetAvailablePorts.srv"
  "srv/ConnectCan.srv"
  "srv/StartBehavior.srv"
)

# インターフェースの生成
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_package()
