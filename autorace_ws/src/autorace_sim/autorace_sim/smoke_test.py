"""시뮬레이터(또는 실차) 인터페이스 점검: 토픽 주기, TF, /cmd_vel 구동, 센서 스냅샷.

  ros2 run autorace_sim smoke_test [--drive] [--out DIR]

--drive 를 주면 로봇을 실제로 움직인다 (직진 0.3 m/s 3 s → 제자리 회전 1.0 rad/s 3 s).
--out 에 color.png, depth.png, scan.png 를 저장한다.
"""
import argparse
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener

TOPICS = [
    ("/clock", Clock), ("/odom", Odometry), ("/imu", Imu), ("/scan", LaserScan),
    ("/camera/color/image_raw", Image), ("/camera/color/camera_info", CameraInfo),
    ("/camera/depth/image_raw", Image), ("/camera/depth/camera_info", CameraInfo),
    ("/camera/depth/points", PointCloud2),
]
TF_PAIRS = [("odom", "base_link"), ("base_link", "laser_link"), ("base_link", "imu_link"),
            ("base_link", "camera_link"), ("camera_link", "camera_color_optical_frame"),
            ("camera_link", "camera_depth_optical_frame")]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Smoke(Node):
    def __init__(self):
        super().__init__("autorace_smoke_test")
        self.count = {t: 0 for t, _ in TOPICS}
        self.last = {}
        for name, typ in TOPICS:
            self.create_subscription(typ, name, lambda m, n=name: self.on_msg(n, m), qos_profile_sensor_data)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def on_msg(self, name, msg):
        self.count[name] += 1
        self.last[name] = msg

    def spin_for(self, seconds, every=None):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if every:
                every()


def save_images(node, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out.mkdir(parents=True, exist_ok=True)
    c = node.last.get("/camera/color/image_raw")
    if c:
        img = np.frombuffer(c.data, np.uint8).reshape(c.height, c.step)[:, :c.width * 3].reshape(c.height, c.width, 3)
        plt.imsave(out / "color.png", img)
    d = node.last.get("/camera/depth/image_raw")
    if d:
        dep = np.frombuffer(d.data, np.uint16).reshape(d.height, d.step // 2)[:, :d.width]
        plt.imsave(out / "depth.png", dep, cmap="viridis", vmin=0, vmax=3000)
    s = node.last.get("/scan")
    if s:
        r = np.asarray(s.ranges)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r >= s.range_min) & (r <= s.range_max)
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.scatter(r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), s=3)
        ax.plot(0, 0, "r^")
        ax.set_aspect("equal"); ax.set_title(f"/scan ({ok.sum()}/{len(r)} valid), frame {s.header.frame_id}")
        fig.savefig(out / "scan.png", dpi=110)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drive", action="store_true")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--window", type=float, default=5.0)
    a, _ = ap.parse_known_args()
    rclpy.init()
    n = Smoke()
    n.spin_for(2.0)
    n.count = dict.fromkeys(n.count, 0)
    n.spin_for(a.window)
    print(f"== 토픽 (벽시계 {a.window:.0f} s 동안 수신)")
    for name, _ in TOPICS:
        m = n.last.get(name)
        extra = ""
        if isinstance(m, Image):
            extra = f"{m.width}x{m.height} {m.encoding} frame={m.header.frame_id}"
        elif isinstance(m, LaserScan):
            extra = (f"{len(m.ranges)} pts, {math.degrees(m.angle_min):.0f}..{math.degrees(m.angle_max):.0f} deg, "
                     f"range {m.range_min:.2f}-{m.range_max:.1f} m, frame={m.header.frame_id}")
        elif m is not None and hasattr(m, "header"):
            extra = f"frame={m.header.frame_id}" + (f" child={m.child_frame_id}" if isinstance(m, Odometry) else "")
        print(f"  {name:32s} {n.count[name] / a.window:6.1f} Hz  {extra}")
    print("== TF")
    for p, c in TF_PAIRS:
        try:
            t = n.tf.lookup_transform(p, c, rclpy.time.Time()).transform
            print(f"  {p} -> {c}: xyz=({t.translation.x:.3f}, {t.translation.y:.3f}, {t.translation.z:.3f}) "
                  f"q=({t.rotation.x:.3f}, {t.rotation.y:.3f}, {t.rotation.z:.3f}, {t.rotation.w:.3f})")
        except Exception as e:  # noqa: BLE001
            print(f"  {p} -> {c}: 없음 ({type(e).__name__})")
    if a.out:
        save_images(n, a.out)
        print("== 저장:", a.out)
    if a.drive:
        def pose():
            o = n.last["/odom"].pose.pose
            return o.position.x, o.position.y, yaw_of(o.orientation)

        def run(v, w, secs):
            msg = Twist()
            msg.linear.x, msg.angular.z = v, w
            p0 = pose()
            n.spin_for(secs, every=lambda: n.cmd.publish(msg))
            n.cmd.publish(Twist())
            n.spin_for(1.0, every=lambda: n.cmd.publish(Twist()))
            p1 = pose()
            dyaw = math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2]))
            return p1[0] - p0[0], p1[1] - p0[1], dyaw

        print("== 구동 (/odom 변화량)")
        dx, dy, dyaw = run(0.3, 0.0, 3.0)
        print(f"  직진 v=0.3 m/s × 3 s (기대 ≈ +0.90 m): dx={dx:+.3f} dy={dy:+.3f} dyaw={math.degrees(dyaw):+.1f} deg")
        dx, dy, dyaw = run(0.0, 1.0, 3.0)
        print(f"  회전 w=1.0 rad/s × 3 s (기대 ≈ +172 deg): dx={dx:+.3f} dy={dy:+.3f} dyaw={math.degrees(dyaw):+.1f} deg")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
