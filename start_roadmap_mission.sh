#!/usr/bin/env bash
#
# 一键启动：底盘/雷达 -> SLAM/Nav2 -> 目标导向 Roadmap + 视觉泊车。
# 目标坐标以小车启动时的 base_link 为基准，单位为米：
#   goal_forward: 前方为正
#   goal_left:    左侧为正，右侧为负

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="${ROBOT_WS:-${SCRIPT_DIR}}"
LIDAR_SETUP="${LIDAR_SETUP:-${HOME}/sdk_ldrobotsensorteam_stl/ros2_app/install/setup.bash}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
GOAL_RADIUS="${GOAL_RADIUS:-0.15}"
STARTUP_ESCAPE_DISTANCE="${STARTUP_ESCAPE_DISTANCE:-0.20}"
STARTUP_ESCAPE_SPEED="${STARTUP_ESCAPE_SPEED:-0.08}"

ATTACH=true
REPLACE=false
POSITIONAL=()

usage()
{
  cat <<'EOF'
用法：
  ./start_roadmap_mission.sh
  ./start_roadmap_mission.sh <前方距离m> <左侧距离m>

示例：
  ./start_roadmap_mission.sh 2.0 8.2
  ./start_roadmap_mission.sh 3.6 -0.9

坐标约定：
  前方为 goal_forward 正方向；
  左侧为 goal_left 正方向，右侧请输入负数。

选项：
  --replace     停止已有 smartcar_test tmux 会话后再启动
  --no-attach   启动后不自动进入 tmux 日志界面
  -h, --help    显示帮助

可选环境变量：
  ROBOT_WS、LIDAR_SETUP、VIDEO_DEVICE、GOAL_RADIUS、
  STARTUP_ESCAPE_DISTANCE、STARTUP_ESCAPE_SPEED
EOF
}

is_number()
{
  [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]
}

while (($# > 0)); do
  case "$1" in
    --replace)
      REPLACE=true
      ;;
    --no-attach)
      ATTACH=false
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      POSITIONAL+=("$@")
      break
      ;;
    -*)
      if is_number "$1"; then
        POSITIONAL+=("$1")
      else
        echo "未知选项：$1" >&2
        usage >&2
        exit 2
      fi
      ;;
    *)
      POSITIONAL+=("$1")
      ;;
  esac
  shift
done

if ((${#POSITIONAL[@]} > 2)); then
  echo "最多只能输入两个位置参数。" >&2
  usage >&2
  exit 2
fi

GOAL_FORWARD="${POSITIONAL[0]:-}"
GOAL_LEFT="${POSITIONAL[1]:-}"

if [[ -z "${GOAL_FORWARD}" ]]; then
  if [[ ! -t 0 ]]; then
    echo "非交互模式必须提供前方距离和左侧距离。" >&2
    exit 2
  fi
  read -r -p "请输入目标前方距离（米）： " GOAL_FORWARD
fi
if [[ -z "${GOAL_LEFT}" ]]; then
  if [[ ! -t 0 ]]; then
    echo "非交互模式必须提供左侧距离。" >&2
    exit 2
  fi
  read -r -p "请输入目标左侧距离（米，右侧为负）： " GOAL_LEFT
fi

for value_name in GOAL_FORWARD GOAL_LEFT GOAL_RADIUS \
  STARTUP_ESCAPE_DISTANCE STARTUP_ESCAPE_SPEED; do
  value="${!value_name}"
  if ! is_number "${value}"; then
    echo "${value_name} 不是有效数字：${value}" >&2
    exit 2
  fi
done

for required_file in \
  /opt/ros/humble/setup.bash \
  "${LIDAR_SETUP}" \
  "${ROBOT_WS}/install/setup.bash"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "缺少环境文件：${required_file}" >&2
    exit 1
  fi
done

if ! command -v tmux >/dev/null 2>&1; then
  echo "未安装 tmux，请先执行：sudo apt install -y tmux" >&2
  exit 1
fi

ROS_PROCESS_PATTERN='/controller_server|/car_base_node|/ldlidar|/roadmap_exploration_server|/roadmap_explore_mission|/slam_toolbox|ros2 launch car_'

source_ros_environment()
{
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "${LIDAR_SETUP}"
  # shellcheck disable=SC1090
  source "${ROBOT_WS}/install/setup.bash"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export RCUTILS_COLORIZED_OUTPUT=0
}

stop_known_sessions()
{
  source_ros_environment

  timeout 3 ros2 topic pub --once \
    /cmd_vel_escape geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  timeout 3 ros2 topic pub --once \
    /cmd_vel_dock geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  timeout 3 ros2 topic pub --once \
    /cmd_vel_raw geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true

  mapfile -t sessions < <(
    tmux list-sessions -F '#{session_name}' 2>/dev/null |
      awk '/^smartcar_test[0-9]+$/')
  for session in "${sessions[@]}"; do
    tmux send-keys -t "${session}:goal" C-c 2>/dev/null || true
    tmux send-keys -t "${session}:navigation" C-c 2>/dev/null || true
  done
  sleep 3
  for session in "${sessions[@]}"; do
    tmux send-keys -t "${session}:bringup" C-c 2>/dev/null || true
  done
  sleep 4
  for session in "${sessions[@]}"; do
    tmux kill-session -t "${session}" 2>/dev/null || true
  done
}

if pgrep -af "${ROS_PROCESS_PATTERN}" >/dev/null 2>&1; then
  if [[ "${REPLACE}" == true ]]; then
    echo "检测到旧测试，正在安全停止已有 smartcar_test 会话……"
    stop_known_sessions
  else
    echo "检测到已有智能车 ROS 进程，拒绝启动第二套控制节点：" >&2
    pgrep -af "${ROS_PROCESS_PATTERN}" >&2 || true
    echo "确认小车安全并需要替换旧测试时，请使用 --replace。" >&2
    exit 1
  fi
fi

if pgrep -af "${ROS_PROCESS_PATTERN}" >/dev/null 2>&1; then
  echo "仍有不属于 smartcar_test 会话的 ROS 进程，请先手动停止：" >&2
  pgrep -af "${ROS_PROCESS_PATTERN}" >&2 || true
  exit 1
fi

mkdir -p "${ROBOT_WS}/test_runs"
next_test=1
shopt -s nullglob
for test_dir in "${ROBOT_WS}"/test_runs/test[0-9]*; do
  test_name="${test_dir##*/}"
  test_number="${test_name#test}"
  if [[ "${test_number}" =~ ^[0-9]+$ ]] &&
      ((10#${test_number} >= next_test)); then
    next_test=$((10#${test_number} + 1))
  fi
done
shopt -u nullglob

RUN_NAME="test${next_test}"
SESSION="smartcar_${RUN_NAME}"
RUN_DIR="${ROBOT_WS}/test_runs/${RUN_NAME}"
LOG_DIR="${RUN_DIR}/logs"
mkdir -p "${LOG_DIR}"

cat >"${RUN_DIR}/mission.env" <<EOF
GOAL_FORWARD=${GOAL_FORWARD}
GOAL_LEFT=${GOAL_LEFT}
GOAL_RADIUS=${GOAL_RADIUS}
STARTUP_ESCAPE_DISTANCE=${STARTUP_ESCAPE_DISTANCE}
STARTUP_ESCAPE_SPEED=${STARTUP_ESCAPE_SPEED}
VIDEO_DEVICE=${VIDEO_DEVICE}
GIT_COMMIT=$(git -C "${ROBOT_WS}" rev-parse --short HEAD 2>/dev/null || echo unknown)
EOF

printf -v q_ws '%q' "${ROBOT_WS}"
printf -v q_lidar_setup '%q' "${LIDAR_SETUP}"
printf -v q_log_dir '%q' "${LOG_DIR}"
printf -v q_video_device '%q' "${VIDEO_DEVICE}"

COMMON_ENV="cd ${q_ws}; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export RCUTILS_COLORIZED_OUTPUT=0; source /opt/ros/humble/setup.bash; source ${q_lidar_setup}; source install/setup.bash"
BRINGUP_CMD="${COMMON_ENV}; stdbuf -oL -eL ros2 launch car_bringup bringup.launch.py use_velocity_mux:=true use_collision_monitor:=false 2>&1 | tee ${q_log_dir}/terminal1_bringup.log"
NAVIGATION_CMD="${COMMON_ENV}; stdbuf -oL -eL ros2 launch car_navigation slam_navigation.launch.py 2>&1 | tee ${q_log_dir}/terminal2_navigation.log"
GOAL_CMD="${COMMON_ENV}; stdbuf -oL -eL ros2 launch car_explore roadmap_exploration.launch.py goal_forward:=${GOAL_FORWARD} goal_left:=${GOAL_LEFT} goal_radius:=${GOAL_RADIUS} startup_escape_distance:=${STARTUP_ESCAPE_DISTANCE} startup_escape_speed:=${STARTUP_ESCAPE_SPEED} video_device:=${q_video_device} 2>&1 | tee ${q_log_dir}/terminal3_goal.log"

wait_for_log()
{
  local pattern="$1"
  local file="$2"
  local timeout_seconds="$3"
  local deadline=$((SECONDS + timeout_seconds))
  while ((SECONDS < deadline)); do
    if [[ -f "${file}" ]] && grep -qE "${pattern}" "${file}"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

start_tmux_window()
{
  local target="$1"
  local command="$2"
  local quoted_command
  printf -v quoted_command '%q' "${command}"
  tmux send-keys -t "${target}" "bash -lc ${quoted_command}" Enter
}

cleanup_failed_start()
{
  echo "启动失败，正在停止 ${SESSION}……" >&2
  tmux send-keys -t "${SESSION}:goal" C-c 2>/dev/null || true
  tmux send-keys -t "${SESSION}:navigation" C-c 2>/dev/null || true
  sleep 2
  tmux send-keys -t "${SESSION}:bringup" C-c 2>/dev/null || true
  sleep 2
  tmux kill-session -t "${SESSION}" 2>/dev/null || true
}

trap cleanup_failed_start ERR

echo "测试编号：${RUN_NAME}"
echo "目标：前方 ${GOAL_FORWARD} m，左侧 ${GOAL_LEFT} m"
echo "碰撞监控：关闭；速度复用：开启"
echo "日志目录：${LOG_DIR}"

printf -v quoted_bringup '%q' "${BRINGUP_CMD}"
tmux new-session -d -s "${SESSION}" -n bringup \
  "bash -lc ${quoted_bringup}"
tmux new-window -d -t "${SESSION}" -n navigation
tmux new-window -d -t "${SESSION}" -n goal

echo "[1/3] 正在启动底盘、雷达和速度复用……"
if ! wait_for_log \
    'car_base 节点已启动' \
    "${LOG_DIR}/terminal1_bringup.log" 20; then
  tail -80 "${LOG_DIR}/terminal1_bringup.log" >&2 || true
  false
fi
if ! wait_for_log \
    'ldlidar communication is normal' \
    "${LOG_DIR}/terminal1_bringup.log" 15; then
  tail -80 "${LOG_DIR}/terminal1_bringup.log" >&2 || true
  false
fi

echo "[2/3] 正在启动 SLAM 和 Nav2……"
start_tmux_window "${SESSION}:navigation" "${NAVIGATION_CMD}"
if ! wait_for_log \
    'Managed nodes are active' \
    "${LOG_DIR}/terminal2_navigation.log" 45; then
  tail -120 "${LOG_DIR}/terminal2_navigation.log" >&2 || true
  false
fi

echo "[3/3] 正在发送 Roadmap 目标……"
start_tmux_window "${SESSION}:goal" "${GOAL_CMD}"
sleep 2

trap - ERR
echo
echo "启动完成：${SESSION}"
echo "查看三个终端：tmux attach -t ${SESSION}"
echo "后台退出 tmux：按 Ctrl+b，再按 d"
echo "完整停止：进入 tmux 后依次在 goal、navigation、bringup 窗口按 Ctrl+C"

if [[ "${ATTACH}" == true && -t 0 && -t 1 ]]; then
  tmux select-window -t "${SESSION}:goal"
  exec tmux attach-session -t "${SESSION}"
fi
