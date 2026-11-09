#!/bin/bash
# 화면용 시뮬 하나: 무작위 배치 + 표지판, 3차선 직전 출발, 실차 모드. 보기 창도 띄움. 사용: live.sh [이름]
cd ~/AutoRace2026/tools/simcmp
N=${1:-live_$(date +%H%M%S)}
python3 gen_layout.py cfg/$N.yaml | tee out_live_$N.txt
C=$HOME/AutoRace2026/autorace_ws/src/autorace_bringup/config/sim_real_mode.yaml
MP=$PWD/practice_m7.yaml ./run.sh $N 85 $C,$PWD/cfg/$N.yaml,$PWD/start_m7.yaml $C &
sleep 5
source /opt/ros/jazzy/setup.bash; source ~/AutoRace2026/autorace_ws/install/setup.bash
ROS_DOMAIN_ID=85 FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/fastdds_shm.xml DISPLAY=${DISPLAY:-:1} python3 view.py &
wait
