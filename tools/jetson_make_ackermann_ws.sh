#!/usr/bin/env bash
# 젯슨 (LIMO Pro) 에서: 기존 디퍼렌셜 워크스페이스는 그대로 두고, 아커만 워크스페이스를 따로 만든다.
#   bash jetson_make_ackermann_ws.sh                         # 디퍼렌셜 = ~/autorace_ws
#   bash jetson_make_ackermann_ws.sh /경로/디퍼렌셜_ws
# 결과: ~/AutoRace2026_ackermann (브랜치 ackermann_jetson) → 워크스페이스 ~/AutoRace2026_ackermann/autorace_ws
#   · mark2455 아커만 코드 + 실차 신호등 YOLO, mission_manager·parking_controller drive_mode 기본 ackermann
#   · 디퍼렌셜 워크스페이스에만 있는 패키지 (드라이버 등) 는 복사해 옴
#   · 같은 이름 패키지인데 내용이 다른 파일 (젯슨에서 고친 값) 은 목록만 보여 줌 — 덮어쓰지 않음
set -euo pipefail
SRC="${1:-$HOME/autorace_ws}"
REPO="$HOME/AutoRace2026_ackermann"
DST="$REPO/autorace_ws"
[ -d "$SRC/src" ] || { echo "디퍼렌셜 워크스페이스가 아님: $SRC (src/ 가 있어야 함) — 경로를 인자로 주세요"; exit 1; }
[ -e "$REPO" ] && { echo "$REPO 가 이미 있음 — 지우거나 이름을 바꾼 뒤 다시"; exit 1; }

git clone -b ackermann_jetson --single-branch "${ACK_REPO_URL:-https://github.com/jji040223-star/AutoRace2026.git}" "$REPO"

echo; echo "== 디퍼렌셜에만 있는 패키지 복사"
for d in "$SRC"/src/*/; do
  n="$(basename "$d")"
  if [ ! -e "$DST/src/$n" ]; then cp -a "$d" "$DST/src/"; echo "  복사: $n"; fi
done

echo; echo "== 같은 패키지인데 다른 파일 (젯슨에서 고친 것일 수 있음 — 필요하면 손으로 옮기기)"
for d in "$DST"/src/*/; do
  n="$(basename "$d")"
  [ -d "$SRC/src/$n" ] && diff -rq --exclude=__pycache__ --exclude='*.bak_*' "$SRC/src/$n" "$DST/src/$n" 2>/dev/null \
    | grep -v "^Only in $DST" || true
done

set +u; [ -z "${ROS_DISTRO:-}" ] && source /opt/ros/humble/setup.bash; set -u
cd "$DST"
colcon build --symlink-install

cat <<EOF

끝. 디퍼렌셜 ($SRC) 은 그대로입니다. 한 터미널에는 둘 중 하나만 source 하세요 (패키지 이름이 같음).
  아커만:     source $DST/install/setup.bash
  디퍼렌셜:   source $SRC/install/setup.bash
실차 (LIMO 를 아커만 모드로 바꾼 뒤):
  ros2 launch autorace_mission drive.launch.py use_sim_time:=false reset_sim:=false \\
      params:=\$(ros2 pkg prefix autorace_bringup)/share/autorace_bringup/config/real_limo.yaml
확인: ros2 param get /parking_controller drive_mode   → ackermann
EOF
