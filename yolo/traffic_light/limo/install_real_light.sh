#!/usr/bin/env bash
# 실차 신호등 YOLO 를 autorace_ws 에 넣는다 (LIMO Pro·다른 PC 공통).
#   bash install_real_light.sh            # ~/autorace_ws 에 설치
#   bash install_real_light.sh /경로/ws   # 다른 워크스페이스
# 하는 일: files/src/ 아래 파일을 워크스페이스 src/ 에 복사 (덮어쓰는 파일은 *.bak_날짜 로 남김) → 두 패키지 (+ 의존 패키지) 만 다시 빌드.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${1:-$HOME/autorace_ws}"
WS="$(cd "$WS" && pwd)"
[ -d "$WS/src/autorace_perception" ] && [ -d "$WS/src/autorace_mission" ] \
  || { echo "autorace_ws 가 아님: $WS (src/autorace_perception, src/autorace_mission 이 있어야 함)"; exit 1; }

STAMP="$(date +%Y%m%d_%H%M%S)"
cd "$HERE/files"
find src -type f | while read -r f; do
  dst="$WS/$f"
  mkdir -p "$(dirname "$dst")"
  if [ -f "$dst" ] && ! cmp -s "$f" "$dst"; then
    cp -p "$dst" "$dst.bak_$STAMP"
    echo "  백업: $dst.bak_$STAMP"
  fi
  cp "$f" "$dst"
  echo "  복사: $f"
done

# ROS 환경 (source 안 된 셸에서 돌려도 되게)
set +u
[ -z "${ROS_DISTRO:-}" ] && source /opt/ros/humble/setup.bash
set -u
cd "$WS"
colcon build --symlink-install --packages-up-to autorace_perception autorace_mission

cat <<EOF

설치 끝. 새 터미널에서 (또는 source $WS/install/setup.bash 뒤):
  ros2 launch autorace_mission drive.launch.py use_sim_time:=false      # 실차: 신호등 모델 자동으로 real
  ros2 launch autorace_mission drive.launch.py use_sim_time:=false enable:=false   # 바퀴 안 돌리고 인식만 확인
로그에 '[drive] 신호등 모델 (real): …traffic_light_real_yolov8n.pt' 가 나오면 맞게 올라간 것.
신호등 박스 확인: ros2 run rqt_image_view rqt_image_view /perception/traffic_light_debug  (고른 박스 초록, 버린 박스 빨강 + 이유)
                  ros2 topic echo /perception/traffic_light
EOF
