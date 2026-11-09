#!/usr/bin/env bash
# 젯슨 (LIMO Pro): 디퍼렌셜 워크스페이스를 아커만 (mark2455 ackermann + 실차 신호등, drive_mode ackermann) 으로 바꾼다.
#   bash jetson_replace_with_ackermann.sh              # ~/autorace_ws (없으면 ~/AutoRace2026/autorace_ws)
#   bash jetson_replace_with_ackermann.sh /경로/ws
# 기존 src 는 ~/autorace_ws_diff_backup_<날짜>.tar.gz 로 남김 (필요 없으면 지워도 됨).
# 젯슨에만 있는 패키지 (드라이버 등) 는 그대로 옮겨 오고, 젯슨에서 고친 파일 목록은 ~/autorace_ws_diff_changes_<날짜>.txt 로.
set -euo pipefail
WS="${1:-}"
if [ -z "$WS" ]; then
  for c in "$HOME/autorace_ws" "$HOME/AutoRace2026/autorace_ws"; do [ -d "$c/src" ] && { WS="$c"; break; }; done
fi
[ -n "$WS" ] && [ -d "$WS/src/autorace_mission" ] || { echo "워크스페이스를 못 찾음 — 경로를 인자로 주세요"; exit 1; }
WS="$(cd "$WS" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
BK="$HOME/autorace_ws_diff_backup_$STAMP.tar.gz"
CH="$HOME/autorace_ws_diff_changes_$STAMP.txt"
echo "== 워크스페이스: $WS"
tar czf "$BK" -C "$WS" src
echo "== 백업: $BK"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
git clone -q --depth 1 -b ackermann_jetson "${ACK_REPO_URL:-https://github.com/jji040223-star/AutoRace2026.git}" "$TMP/repo"
NEW="$TMP/repo/autorace_ws/src"
for d in "$WS"/src/*/; do
  n="$(basename "$d")"
  if [ ! -e "$NEW/$n" ]; then cp -a "$d" "$NEW/"; echo "  젯슨에만 있던 패키지 유지: $n"; fi
done
diff -rq --exclude=__pycache__ --exclude='*.bak_*' "$WS/src" "$NEW" > "$CH" 2>/dev/null || true
echo "== 바뀌는 파일 목록: $CH ($(grep -c . "$CH" || true) 줄)"

rm -rf "$WS/src" "$WS/build" "$WS/install" "$WS/log"
mv "$NEW" "$WS/src"
set +u; [ -z "${ROS_DISTRO:-}" ] && source /opt/ros/humble/setup.bash; set -u
cd "$WS"
colcon build --symlink-install
echo
echo "끝: $WS 는 이제 아커만. 새 터미널에서 source $WS/install/setup.bash"
echo "확인: ros2 param get /parking_controller drive_mode → ackermann (drive.launch 띄운 뒤)"
