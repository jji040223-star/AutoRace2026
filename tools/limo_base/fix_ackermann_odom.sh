#!/usr/bin/env bash
# wego limo_base (limo_ros2) 아커만 odom 수정 — 젯슨 (LIMO Pro) 에서 한 번.
#   bash fix_ackermann_odom.sh [~/wego_ws]
# 문제: limo_driver.cpp 의 오도메트리 switch (motion_mode_) 에 MODE_ACKERMANN 이 없어서, 본체가 아커만이면
#       default 로 빠져 "Invalid motion mode you should change limo to ackermann mode!" 만 찍고 /odom·/imu 를 안 냄
#       → mission_manager 가 위치 없음으로 M5 가 안 끝나고 M3 에서 TypeError 로 죽음 (2026-11-09 실차 ack1).
# 수정: 그 switch 의 case MODE_FOUR_DIFF 앞에 case MODE_ACKERMANN: 을 넣어 같은 계산 (휠 선속도) 을 쓴다.
#       우리 노드는 /odom 의 선속도 + IMU 방향 (odom_imu) 만 쓰므로 이것으로 충분.
# 원본은 limo_driver.cpp.orig_diffonly 로 남김. 조향 배율 (left/right_angle_scale_ 2.47) 은 건드리지 않음
#   (1.2 로 바꿔도 바퀴가 ~15° 에서 안 늘어남 — 본체 한계, 실차_아커만_정리_1109.md 참고).
set -euo pipefail
WS="${1:-$HOME/wego_ws}"
F="$WS/src/limo_ros2/limo_base/src/limo_driver.cpp"
[ -f "$F" ] || { echo "없음: $F"; exit 1; }
if awk 'NR>500 && /case MODE_ACKERMANN:/ {f=1} END {exit !f}' "$F"; then echo "이미 수정됨: $F"; else
  cp -n "$F" "$F.orig_diffonly"
  N=$(awk 'NR>500 && /case MODE_FOUR_DIFF: \{/ {print NR; exit}' "$F")
  [ -n "$N" ] || { echo "odom switch 의 case MODE_FOUR_DIFF 를 못 찾음 — 손으로 고칠 것"; exit 1; }
  sed -i "${N}i\\        case MODE_ACKERMANN:   // added 2026-11-09: wego driver had no ackermann odom case" "$F"
  echo "수정: $F (줄 $N)"; sed -n "$((N-2)),$((N+2))p" "$F"
fi
set +u; [ -z "${ROS_DISTRO:-}" ] && source /opt/ros/humble/setup.bash; set -u
cd "$WS" && colcon build --packages-select limo_base
echo "끝. teleop_launch (limo_base) 를 다시 켜고: ros2 topic hz /odom  (나와야 함)"
