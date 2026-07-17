#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-}"
POS_TIME="${2:-}"
PAUSE_TIME="${3:-}"
NEG_TIME="${4:-}"

if [[ -z "$MODE" || -z "$POS_TIME" || -z "$PAUSE_TIME" || -z "$NEG_TIME" ]]; then
  echo "Usage: $0 <speed|mit> <forward_sec> <pause_sec> <reverse_sec>"
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$HOME/tianbot_ws/install/setup.bash"

PUB_RATE="${SPIN_PUB_RATE:-20}"
SPEED_WZ="${SPIN_SPEED_WZ:-0.6}"
MIT_TFF="${SPIN_MIT_TFF:-0.10}"

publish_zero_speed() {
  timeout 0.3s ros2 topic pub -r "$PUB_RATE" /tianbot/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
}

publish_speed_spin() {
  local wz="$1"
  timeout "${2}s" ros2 topic pub -r "$PUB_RATE" /tianbot/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${wz}}}" >/dev/null 2>&1 || true
}

publish_zero_mit() {
  timeout 0.3s ros2 topic pub -r "$PUB_RATE" /tianbot/wheel_mit_cmd tianbot_core/msg/DmMitCommand \
    "{p_des: [0.0, 0.0, 0.0, 0.0], v_des: [0.0, 0.0, 0.0, 0.0], kp: [0.0, 0.0, 0.0, 0.0], kd: [0.0, 0.0, 0.0, 0.0], t_ff: [0.0, 0.0, 0.0, 0.0]}" >/dev/null 2>&1 || true
}

publish_mit_spin() {
  local tff="$1"
  local duration="$2"
  timeout "${duration}s" ros2 topic pub -r "$PUB_RATE" /tianbot/wheel_mit_cmd tianbot_core/msg/DmMitCommand \
    "{p_des: [0.0, 0.0, 0.0, 0.0], v_des: [0.0, 0.0, 0.0, 0.0], kp: [0.0, 0.0, 0.0, 0.0], kd: [0.0, 0.0, 0.0, 0.0], t_ff: [${tff}, -${tff}, ${tff}, -${tff}]}" >/dev/null 2>&1 || true
}

if [[ "$MODE" == "speed" ]]; then
  echo "Switching to speed mode..."
  ros2 service call /tianbot/set_control_mode tianbot_core/srv/SetControlMode "{mode: 'speed', save_to_flash: false}" >/dev/null
  publish_zero_speed
  echo "Spin + for ${POS_TIME}s at wz=${SPEED_WZ}"
  publish_speed_spin "$SPEED_WZ" "$POS_TIME"
  publish_zero_speed
  echo "Pause ${PAUSE_TIME}s"
  sleep "$PAUSE_TIME"
  echo "Spin - for ${NEG_TIME}s at wz=-${SPEED_WZ}"
  publish_speed_spin "-$SPEED_WZ" "$NEG_TIME"
  publish_zero_speed
elif [[ "$MODE" == "mit" ]]; then
  echo "Switching to mit mode..."
  ros2 service call /tianbot/set_control_mode tianbot_core/srv/SetControlMode "{mode: 'mit', save_to_flash: false}" >/dev/null
  publish_zero_mit
  echo "Spin + for ${POS_TIME}s at t_ff=${MIT_TFF}"
  publish_mit_spin "$MIT_TFF" "$POS_TIME"
  publish_zero_mit
  echo "Pause ${PAUSE_TIME}s"
  sleep "$PAUSE_TIME"
  echo "Spin - for ${NEG_TIME}s at t_ff=-${MIT_TFF}"
  publish_mit_spin "-$MIT_TFF" "$NEG_TIME"
  publish_zero_mit
else
  echo "Unsupported mode: $MODE"
  exit 1
fi

echo "Done."
