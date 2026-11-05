"""시뮬을 끄지 않고 처음 상태로: LIMO 출발 자세, 라바콘 원위치, 신호등·갈림길·광고판 주기 0 초부터.

  ros2 run autorace_sim reset_sim [--service /sim/reset] [--timeout 10]
  ros2 run autorace_sim reset_sim --pose 5.30 4.84 0      # 이번 리셋만 이 출발 자세 (world x y yaw_deg) — /sim/start_pose

run_sim.py 의 /sim/reset (std_srvs/Trigger) 을 부른다. lane_follow.launch.py 는 시작할 때 이것부터 돌린다 (reset_sim:=true).
시뮬을 막 띄운 직후에는 시뮬이 리셋을 하고도 응답을 못 보낼 때가 있다 ("failed to send response (timeout)" — 응답 쪽 연결이
아직 안 됨, 2026-11-01 시험에서 2 번). 리셋은 여러 번 해도 같으므로 응답이 없으면 --retries 번까지 다시 부른다.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose2D
from std_srvs.srv import Trigger


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--service", default="/sim/reset")
    ap.add_argument("--timeout", type=float, default=10.0, help="서비스를 기다리는 시간 [s]")
    ap.add_argument("--retries", type=int, default=3, help="응답이 없을 때 다시 부르는 횟수")
    ap.add_argument("--reply_timeout", type=float, default=5.0, help="한 번 부르고 응답을 기다리는 시간 [s]")
    ap.add_argument("--pose", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"),
                    help="이번 리셋만 이 출발 자세 (/sim/start_pose 를 보낸 뒤 리셋, 응답의 자세로 확인)")
    a, _ = ap.parse_known_args()   # launch 가 붙이는 --ros-args 는 무시
    rclpy.init()
    node = rclpy.create_node("reset_sim")
    cli = node.create_client(Trigger, a.service)
    ok = False
    if not cli.wait_for_service(timeout_sec=a.timeout):
        # 시뮬은 떠 있는데 서비스만 없으면 = /sim/reset 을 넣기 (2026-10-26 22:23) 전에 띄운 옛 시뮬
        if node.count_publishers("/clock") > 0:
            why = ("시뮬은 떠 있지만 리셋 서비스가 없는 옛 버전입니다.\n"
                   "  → bringup 터미널을 Ctrl+C 로 끄고 다시 실행: ros2 launch autorace_bringup bringup.launch.py mode:=sim")
        else:
            why = ("시뮬(/clock)이 안 보입니다. bringup mode:=sim 이 떠 있는지, 두 터미널의 ROS_DOMAIN_ID 가 같은지 확인\n"
                   "  (echo $ROS_DOMAIN_ID)")
        print(f"[reset_sim] {a.service} 를 {a.timeout:.0f} s 동안 못 찾음 — {why}", file=sys.stderr)
    else:
        res = None
        pose_pub = node.create_publisher(Pose2D, "/sim/start_pose", 10) if a.pose else None
        for i in range(1 + max(0, a.retries)):
            if pose_pub is not None:
                # 출발 자세는 다음 리셋 한 번만 쓰인다 — 리셋 전에 꼭 도착하게: 구독자를 기다렸다 보내고 잠깐 돌린다
                t0 = time.time()
                while pose_pub.get_subscription_count() < 1 and time.time() - t0 < a.timeout:
                    rclpy.spin_once(node, timeout_sec=0.1)
                pose_pub.publish(Pose2D(x=a.pose[0], y=a.pose[1], theta=math.radians(a.pose[2])))
                t0 = time.time()
                while time.time() - t0 < 0.5:
                    rclpy.spin_once(node, timeout_sec=0.05)
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=a.reply_timeout)
            res = fut.result()
            if res is not None and a.pose and f"({a.pose[0]:.2f}, {a.pose[1]:.2f})" not in res.message:
                print(f"[reset_sim] 출발 자세가 안 맞음 ({res.message[:60]}) — 다시 ({i + 1}/{a.retries})", flush=True)
                res = None
                continue
            if res is not None:
                break
            cli.remove_pending_request(fut)
            if i < a.retries:
                print(f"[reset_sim] {a.service} 응답 없음 ({a.reply_timeout:.0f} s) — 다시 부름 ({i + 1}/{a.retries})", flush=True)
        if res is None:
            print(f"[reset_sim] {a.service} 응답 없음 ({1 + a.retries} 번 불러도)", file=sys.stderr)
        else:
            ok = res.success
            print(f"[reset_sim] {'완료' if ok else '실패'}: {res.message}", file=sys.stdout if ok else sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)
