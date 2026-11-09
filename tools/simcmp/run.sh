#!/bin/bash
# 사용: run.sh <이름> <도메인> <sim_params (쉼표)> <drive params (쉼표, 없으면 '')>
N=$1; DOM=$2; SP=$3; DP=$4
OUT=~/AutoRace2026/tools/simcmp/out/$N; rm -rf $OUT; mkdir -p $OUT
source /opt/ros/jazzy/setup.bash; source ~/AutoRace2026/autorace_ws/install/setup.bash
export ROS_DOMAIN_ID=$DOM
# 영상을 공유 메모리로 (기본 UDP 는 수신 버퍼가 작아 카메라 영상이 대부분 버려져 차선 인식이 초당 1–5 장이 됨)
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/AutoRace2026/tools/simcmp/fastdds_shm.xml
setsid ros2 launch autorace_bringup sim2d.launch.py sim_params:=$SP > $OUT/sim.log 2>&1 &
sleep 6
setsid ros2 launch autorace_mission drive.launch.py perception:=sim2d light_source:=fake ${DP:+params:=$DP} \
  mission_params:=${MP:-$HOME/AutoRace2026/tools/simcmp/practice.yaml} > $OUT/drive.log 2>&1 &
timeout 1500 ros2 run autorace_sim sim2d_eval --until parking_done --timeout 600 --frames $OUT/frames --every 10 > $OUT/eval.log 2>&1
# 정리: 이 도메인 launch 들만
for p in $(jobs -p); do kill -INT -- -$p 2>/dev/null; done; sleep 5
for p in $(jobs -p); do kill -9 -- -$p 2>/dev/null; done
echo done > $OUT/finished
