#!/bin/bash
# 주행 (센서는 wego teleop_launch 가 켜져 있어야 함). 사용: SPEED=0.3 run_drive.sh <이름> [drive.launch 추가 인자...]
source ~/AutoRace2026_sim2d/env.sh
N=${1:-run}; shift
D=~/AutoRace2026_sim2d/runs/$(date +%m%d_%H%M%S)_$N; mkdir -p $D
P0=$(ros2 pkg prefix autorace_bringup)/share/autorace_bringup/config/real_limo.yaml
P=$D/params.yaml
python3 ~/AutoRace2026_sim2d/merge_params.py $P0 $P ${SPEED:-0.3} || exit 1
date "+%F %T 시작" > $D/start_time.txt
# CPU 클럭 최대 고정 (ack51 처음 버벅임 원인 후보). sudoers 에 jetson_clocks 비밀번호 없이 허용돼 있어야 함
if sudo -n /usr/bin/jetson_clocks 2>/dev/null; then echo "jetson_clocks 켬 (CPU 최소 $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq) kHz)" > $D/clocks.log; else echo "jetson_clocks 못 켬 (sudo 비밀번호 필요)" > $D/clocks.log; fi
[ -n "$NOBAG" ] && echo "녹화 안 함 (NOBAG)" > $D/bag.log || setsid nohup ros2 bag record -o $D/bag /camera/color/image_raw/compressed /camera/depth/image_raw/compressedDepth /scan /odom /imu /cmd_vel /lane/state /mission/state /mission/lane_cmd /perception/detections /perception/obstacles /perception/cones /parking/state /tf /tf_static > $D/bag.log 2>&1 < /dev/null &
setsid nohup ros2 launch autorace_mission drive.launch.py use_sim_time:=false reset_sim:=false params:=$P mission_params:=$HOME/AutoRace2026_sim2d/${MP:-practice.yaml} light_source:=${LIGHT:-none} "$@" > $D/drive.log 2>&1 < /dev/null &
echo $D
