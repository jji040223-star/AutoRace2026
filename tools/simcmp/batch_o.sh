#!/bin/bash
# 10 개 동시: cfg/s1~s7 고정 + 무작위 3 개. 실차 모드, 출발선부터 한 바퀴 전체 미션
cd ~/AutoRace2026/tools/simcmp
C=$HOME/AutoRace2026/autorace_ws/src/autorace_bringup/config/sim_real_mode.yaml
# ACK=1 이면 아커만 (시뮬 차 움직임 + 미션·주차 노드), G8=<cfg> 면 8 번 배치를 그 파일로 (보기 창이 붙는 자리)
A=${ACK:+,$PWD/ackermann.yaml}
# 출발 위치: 기본 = 출발선부터 한 바퀴 전체 (11-09 사용자). START=start_m7.yaml 이면 횡단보도 앞
ST=$PWD/${START:-start_full.yaml}
P=${1:-o}
# 강제 종료된 시뮬이 남긴 FastDDS 공유 메모리 (/dev/shm/fastrtps_*) 가 쌓이면 16 GB 가 차서 노드가 시작하자마자 죽음 (11-09 f 묶음)
# → 어떤 프로세스도 매핑하지 않은 내 것만 지움
cat /proc/[0-9]*/maps 2>/dev/null | grep -o "/dev/shm/[^ ]*" | sort -u > /tmp/.shm_used_$$
for f in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; do
  [ -O "$f" ] || continue; grep -qx "$f" /tmp/.shm_used_$$ && continue; rm -f -- "$f"
done; rm -f /tmp/.shm_used_$$
i=0
# CDEF=1 이면 s1~s4 를 라바콘 공식 배치 (default, USD S자) 로
L=${CDEF:+s1_default_cdef s2_default_cdef s3_mirror_cdef s4_mirror_cdef}
for s in ${L:-s1_default s2_default s3_mirror s4_mirror} s5_rand s6_rand s7_rand; do
  i=$((i+1)); n=${P}${i}_${s#s?_}
  MP=$PWD/practice_m7.yaml ./run.sh $n $((70+i)) $C,$PWD/cfg/$s.yaml,$ST$A $C$A & PIDS="$PIDS $!"
  sleep 2
done
for k in 8 9 10; do
  n=${P}${k}_gen
  if [ $k = 8 ] && [ -n "$G8" ]; then cp $G8 cfg/$n.yaml; echo "$G8 그대로" > out_live_$n.txt; else python3 gen_layout.py cfg/$n.yaml > out_live_$n.txt; fi
  MP=$PWD/practice_m7.yaml ./run.sh $n $((70+k)) $C,$PWD/cfg/$n.yaml,$ST$A $C$A & PIDS="$PIDS $!"
  sleep 2
done
# 하나 (k=8) 는 보기 창으로 보여줌 (NOVIEW=1 이면 안 띄움)
[ -z "$NOVIEW" ] && ( sleep 5; source /opt/ros/jazzy/setup.bash; source ~/AutoRace2026/autorace_ws/install/setup.bash
  ROS_DOMAIN_ID=78 FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/fastdds_shm.xml DISPLAY=${DISPLAY:-:1} python3 view.py ) &
wait $PIDS
echo ALL_DONE
