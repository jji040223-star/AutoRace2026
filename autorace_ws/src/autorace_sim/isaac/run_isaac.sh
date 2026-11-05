#!/usr/bin/env bash
# Isaac Sim python 실행 래퍼.
# 1) ROS2 Humble 을 source 한 셸에서는 PYTHONPATH 의 python3.10 rclpy 가 Isaac(python3.11) 내장 rclpy 를 가려서
#    import 가 깨진다 → ROS python 경로만 빼고 Isaac 내장 humble 라이브러리를 쓰게 한다.
# 2) Isaac 의 python.sh 는 python 을 자식으로 띄우고 시그널을 넘기지 않는다 → ros2 launch 의 Ctrl+C/SIGTERM 을
#    직접 넘겨 줘야 Isaac 이 남지 않는다.
# 사용: run_isaac.sh <script.py> [args...]   (ISAACSIM_DIR 기본 ~/isaacsim)
ISAACSIM_DIR="${ISAACSIM_DIR:-$HOME/isaacsim}"
PYTHONPATH="$(printf '%s' "${PYTHONPATH:-}" | tr ':' '\n' | grep -v -E '/opt/ros/|python3\.10' | paste -sd: - || true)"
export PYTHONPATH
export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ISAACSIM_DIR/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib"

"$ISAACSIM_DIR/python.sh" "$@" &
pid=$!
trap 'pkill -INT -P "$pid"' INT
trap 'pkill -TERM -P "$pid"' TERM
rc=0
while kill -0 "$pid" 2>/dev/null; do
    wait "$pid"; rc=$?         # 시그널을 받으면 wait 가 먼저 돌아오므로 자식이 끝날 때까지 반복
done
exit "$rc"
