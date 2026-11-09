#!/bin/bash
# 10 개 동시: cfg/s1~s7 고정 + 무작위 3 개. 실차 모드, 횡단보도 앞 출발, 전체 미션
cd ~/AutoRace2026/tools/simcmp
C=$HOME/AutoRace2026/autorace_ws/src/autorace_bringup/config/sim_real_mode.yaml
P=${1:-o}
i=0
# CDEF=1 이면 s1~s4 를 라바콘 공식 배치 (default, USD S자) 로
L=${CDEF:+s1_default_cdef s2_default_cdef s3_mirror_cdef s4_mirror_cdef}
for s in ${L:-s1_default s2_default s3_mirror s4_mirror} s5_rand s6_rand s7_rand; do
  i=$((i+1)); n=${P}${i}_${s#s?_}
  MP=$PWD/practice_m7.yaml ./run.sh $n $((70+i)) $C,$PWD/cfg/$s.yaml,$PWD/start_m7.yaml $C & PIDS="$PIDS $!"
  sleep 2
done
for k in 8 9 10; do
  n=${P}${k}_gen; python3 gen_layout.py cfg/$n.yaml > out_live_$n.txt
  MP=$PWD/practice_m7.yaml ./run.sh $n $((70+k)) $C,$PWD/cfg/$n.yaml,$PWD/start_m7.yaml $C & PIDS="$PIDS $!"
  sleep 2
done
# 하나 (k=8) 는 보기 창으로 보여줌
( sleep 5; source /opt/ros/jazzy/setup.bash; source ~/AutoRace2026/autorace_ws/install/setup.bash
  ROS_DOMAIN_ID=78 FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/fastdds_shm.xml DISPLAY=${DISPLAY:-:1} python3 view.py ) &
wait $PIDS
echo ALL_DONE
