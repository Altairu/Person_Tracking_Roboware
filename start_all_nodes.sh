#!/bin/bash

# ワークスペースルートで実行する
# (src/ install/ build/ が並んでいるディレクトリ)

set -euo pipefail

run_env="source /opt/ros/humble/setup.bash; source install/setup.bash"

# ビルドして環境をソース
colcon build
set +u
eval "$run_env"
set -u

# launch で全ノードを一括起動
ros2 launch Robowarepkg roboware.launch.py
