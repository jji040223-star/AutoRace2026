#!/bin/bash
# 주행 즉시 멈춤: 주행 노드를 바로 죽이고 (mux 가 먼저 죽어 /cmd_vel 이 끊김) 0 을 계속 보냄, 녹화 끝
source ~/AutoRace2026_sim2d/env.sh
pkill -9 -f "lib/autorace_mission/cmd_vel_mux"
pkill -9 -f "lib/autorace_(mission|lane|perception)/"
pkill -9 -f "ros2 launch autorace_mission"
timeout 2 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
pkill -INT -f "ros2 bag record"
sleep 1; pgrep -fa "lib/autorace_|bag record" | grep -v -e pgrep -e stop_drive || echo "주행 노드 모두 꺼짐"
