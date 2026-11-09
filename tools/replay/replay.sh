#!/bin/bash
# 실차 bag 을 다시 틀고 판단 노드 (mission_manager) 만 돌려 지금 코드가 어떻게 판단했을지 본다. 차는 안 움직임.
# 사용: replay.sh <real_runs 폴더 이름> <도메인>
R=$1; DOM=$2; RUN=~/AutoRace2026/real_runs/$R; OUT=~/AutoRace2026/tools/replay/out/$R; rm -rf $OUT; mkdir -p $OUT
source /opt/ros/jazzy/setup.bash; source ~/AutoRace2026/autorace_ws/install/setup.bash
export ROS_DOMAIN_ID=$DOM
S=$(ros2 pkg prefix autorace_mission)/share/autorace_mission/config/mission_manager.yaml
setsid python3 ~/AutoRace2026/autorace_ws/install/autorace_mission/lib/autorace_mission/odom_imu --ros-args -p use_sim_time:=true > $OUT/odom.log 2>&1 &
setsid python3 ~/AutoRace2026/autorace_ws/install/autorace_mission/lib/autorace_mission/mission_manager --ros-args \
  --params-file $S --params-file $RUN/params.yaml --params-file ~/AutoRace2026/tools/replay/practice_real.yaml \
  -p use_sim_time:=true -p odom_topic:=/odom_imu -p imu_topic:=/imu_unbiased > $OUT/mission.log 2>&1 &
sleep 4
ros2 bag play $RUN/bag --clock --rate 2.0 --topics /odom /imu /lane/state /perception/obstacles /perception/detections /scan /perception/cones > $OUT/play.log 2>&1
sleep 2
for p in $(jobs -p); do kill -INT -- -$p 2>/dev/null; done; sleep 2; for p in $(jobs -p); do kill -9 -- -$p 2>/dev/null; done
echo done > $OUT/finished
