#!/usr/bin/env bash
# LIMO 실차 주행 기록 (~/AutoRace2026_sim2d/runs/<날짜>_<이름>/: drive.log, bag, params.yaml) 을 이 컴퓨터로 가져온다.
#   pull_runs.sh          한 번 가져오기 (이미 받은 건 건너뜀)
#   pull_runs.sh --watch  30 s 마다 계속 (LIMO 가 꺼져 있으면 기다림) — 주행 끝날 때마다 자동으로 받아짐
LIMO=${LIMO:-wego@100.88.175.76}
SRC=${SRC:-AutoRace2026_sim2d/runs/}
DST=${DST:-$HOME/AutoRace2026/real_runs/}
mkdir -p "$DST"
pull() {
  # --partial: 끊겨도 이어받기. 녹화 중인 bag 은 다음 번에 마저 받음
  rsync -a --partial --timeout=20 -e "ssh -o BatchMode=yes -o ConnectTimeout=5" "$LIMO:$SRC" "$DST" 2>/dev/null
}
if [ "$1" = "--watch" ]; then
  echo "[pull_runs] $LIMO:$SRC → $DST (30 s 마다, Ctrl+C 로 끝)"
  while true; do
    before=$(ls "$DST" | wc -l)
    if pull; then
      after=$(ls "$DST" | wc -l)
      [ "$after" != "$before" ] && echo "[pull_runs] $(date +%H:%M:%S) 새 주행 $((after - before)) 개: $(ls -t "$DST" | head -$((after - before)) | tr '\n' ' ')"
    fi
    sleep 30
  done
else
  pull && echo "[pull_runs] 받음 → $DST ($(ls "$DST" | wc -l) 개 주행)" || echo "[pull_runs] LIMO ($LIMO) 에 접속 못 함 — 전원·Tailscale 확인"
fi
