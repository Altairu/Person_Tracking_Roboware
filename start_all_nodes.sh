#!/bin/bash

# Stop on errors so builds or spawns don't silently fail
set -euo pipefail

# Ensure WezTerm CLI is available
if ! command -v wezterm >/dev/null 2>&1; then
	echo "Error: wezterm command not found. Please install WezTerm or add it to PATH." >&2
	exit 1
fi

run_env="source /opt/ros/humble/setup.bash; source install/setup.bash"

spawn_wezterm_tab() {
	local title="$1"
	shift
	local command="$*"

	local output
	if ! output=$(wezterm cli spawn --cwd "$PWD" -- bash -lc "$command" 2>/dev/null); then
		echo "Failed to spawn WezTerm tab for ${title}" >&2
		return 1
	fi

	local tab_id
	tab_id=$(printf '%s\n' "$output" | awk -F': ' '/^tab:/ {print $2; exit}')
	if [[ -n "${tab_id:-}" ]]; then
		wezterm cli set-tab-title --tab-id "$tab_id" --title "$title" >/dev/null 2>&1 || true
	fi
}

# Build workspace and source environment for current shell
colcon build
# some ROS setup scripts expect unset variables; temporarily disable nounset
set +u
eval "$run_env"
set -u

spawn_wezterm_tab "RealSense_node" "$run_env; ros2 run Robowarepkg RealSense_node; exec bash"
spawn_wezterm_tab "web_socket_node" "$run_env; ros2 run Robowarepkg web_socket_node; exec bash"
spawn_wezterm_tab "Roboware_node" "$run_env; ros2 run Robowarepkg Roboware_node; exec bash"
spawn_wezterm_tab "PID_node" "$run_env; ros2 run Robowarepkg PID_node; exec bash"
spawn_wezterm_tab "serial_read_node" "$run_env; ros2 run Robowarepkg serial_read_node; exec bash"
spawn_wezterm_tab "serial_send_node" "$run_env; ros2 run Robowarepkg serial_send_node; exec bash"
spawn_wezterm_tab "FaceAnimation_node" "$run_env; ros2 run Robowarepkg FaceAnimation_node; exec bash"
