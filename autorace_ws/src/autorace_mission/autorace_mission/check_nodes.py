"""같은 ROS_DOMAIN_ID 에 이미 떠 있는 노드가 있으면 exit 1 — drive.launch 가 맨 먼저 돌린다.

  ros2 run autorace_mission check_nodes lane_follower yolo_detector mission_manager

다른 창에 남아 있는 차선 추종이 /cmd_vel 을 같이 내서 차가 이상하게 움직이는 일을 막는다 (2026-10-31 겪음).
"""
import sys
import time

import rclpy


def main():
    want = [a for a in sys.argv[1:] if not a.startswith('-')]
    if '--ros-args' in sys.argv:            # launch 가 붙이는 인자는 무시
        want = [a for a in sys.argv[1:sys.argv.index('--ros-args')] if not a.startswith('-')]
    rclpy.init()
    node = rclpy.create_node('check_nodes')
    found = set()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.5:      # DDS 발견에 1~2 s (노드마다 보이는 시점이 달라 모아 둔다)
        rclpy.spin_once(node, timeout_sec=0.1)
        found |= {n for n in node.get_node_names() if n in want}
    node.destroy_node()
    rclpy.shutdown()
    if found:
        print(f"[check_nodes] 이미 떠 있는 노드: {', '.join(sorted(found))} — 다른 터미널(창)에서 Ctrl+C 로 끈 뒤 다시 실행하세요.\n"
              f"  어디 떠 있는지: ps aux | grep -E '{'|'.join(sorted(found))}'", file=sys.stderr)
        sys.exit(1)
    print(f"[check_nodes] 겹치는 노드 없음 ({', '.join(want)})")
