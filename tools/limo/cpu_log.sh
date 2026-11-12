#!/bin/bash
# 주행 중 CPU 기록: 2초마다 전체·프로세스별 CPU, tegrastats(GPU). 사용: cpu_log.sh <폴더>  (Ctrl+C/kill 로 끝)
D=${1:-$HOME/AutoRace2026_sim2d/runs/cpu_$(date +%m%d_%H%M%S)}; mkdir -p $D
tegrastats --interval 2000 --logfile $D/tegrastats.log &
TP=$!; trap "kill $TP 2>/dev/null; exit" INT TERM
while true; do echo "=== $(date +%T)"; top -b -n1 -w 200 -c | head -30; sleep 2; done > $D/top.log
