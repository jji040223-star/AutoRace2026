"""신호등 YOLO 학습 데이터 (시뮬) — 신호등 앞 여러 자세로 LIMO 를 옮기고 (/sim/start_pose + /sim/reset) 색을 바꿔 (/sim/traffic_light)
카메라 영상과 YOLO 라벨 (몸체 박스, 클래스 = 켜진 색) 을 저장한다.

  ros2 run autorace_sim light_dataset --out ~/datasets/traffic_light_sim --n 900 [--neg 0.15] [--seed 0] [--prefix a]

라벨 박스: 시뮬이 아는 신호등 앞면 (fake_traffic_light 와 같은 투영) — 화면 밖으로 반 넘게 잘리면 그 장은 라벨 없이 (배경) 저장.
검사: 라벨 색 칸이 세 칸 중 가장 밝고 두 번째보다 contrast_min 배 이상일 때만 저장 (렌더러가 멈춘 카메라에서 꺼진 램프 빛을 남기는 잔상 거르기).
광고 모니터 (광고 그림·방해 그래픽) 는 라벨을 안 붙인다 = 배경. 클래스: 0 red_light, 1 yellow_light, 2 green_light.
좌표·크기: LimoIsaacSIM/autorace_2026 config/missions.yaml traffic_light (앞면 x 3.95, y 4.47, 폭 0.12, z 0.10–0.305, −x 를 봄).
"""
import argparse
import math
import os
import random
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

COLORS = ('red', 'yellow', 'green')
FACE = (3.95, 4.47, 0.12, 0.10, 0.305)   # x, y 가운데, 폭, 아래·위 z
CAM_X, CAM_Z = 0.084, 0.175


def project(K, size, pose, face=FACE):
    """앞면 → 영상 박스 (x1, y1, x2, y2) 또는 None (뒤·반 넘게 잘림)."""
    fx, fy, cx, cy = K
    x, y, yaw = pose
    camx, camy = x + CAM_X * math.cos(yaw), y + CAM_X * math.sin(yaw)
    fxw, fyw, w, z0, z1 = face
    us, vs = [], []
    for py in (fyw - w / 2, fyw + w / 2):
        for pz in (z0, z1):
            dx, dy = fxw - camx, py - camy
            f = dx * math.cos(yaw) + dy * math.sin(yaw)
            left = -dx * math.sin(yaw) + dy * math.cos(yaw)
            if f < 0.1:
                return None
            us.append(cx - fx * left / f)
            vs.append(cy - fy * (pz - CAM_Z) / f)
    W, H = size
    x1, x2, y1, y2 = min(us), max(us), min(vs), max(vs)
    if x2 < 0 or x1 > W or y2 < 0 or y1 > H:
        return None
    vis = (min(x2, W) - max(x1, 0)) * (min(y2, H) - max(y1, 0)) / max(1.0, (x2 - x1) * (y2 - y1))
    if vis < 0.5:
        return None
    return max(0.0, x1), max(0.0, y1), min(W - 1.0, x2), min(H - 1.0, y2)


def cells(bgr, b):
    x1, y1, x2, y2 = (int(round(v)) for v in b)
    if x2 - x1 < 3 or y2 - y1 < 6:
        return None
    v = bgr[y1:y2, x1:x2].max(axis=2).astype(np.float32)
    h, w = v.shape
    mid = v[:, int(w * 0.25):max(int(w * 0.25) + 1, int(w * 0.75))]
    out = []
    for i in range(3):
        c = mid[int(h * i / 3):max(int(h * i / 3) + 1, int(h * (i + 1) / 3))].ravel()
        out.append(float(np.sort(c)[-max(1, len(c) // 10):].mean()))
    return out


class Collector(Node):
    def __init__(self):
        super().__init__('light_dataset')
        self.K = self.size = self.img = None
        self.clock = 0.0
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'clock', m.clock.sec + m.clock.nanosec * 1e-9), 10)
        self.light_pub = self.create_publisher(String, '/sim/traffic_light', 10)
        self.pose_pub = self.create_publisher(Pose2D, '/sim/start_pose', 10)
        self.reset_cli = self.create_client(Trigger, '/sim/reset')

    def on_info(self, m):
        if m.k[0] > 0:
            self.K, self.size = (m.k[0], m.k[4], m.k[2], m.k[5]), (m.width, m.height)

    def on_image(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        bgr = a[:, :, ::-1] if m.encoding.startswith('rgb') else a
        self.img = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, np.ascontiguousarray(bgr))

    def spin_for(self, s):
        t = time.time()
        while time.time() - t < s:
            rclpy.spin_once(self, timeout_sec=0.02)

    def shot(self, pose, color, settle):
        """그 자세·색으로 리셋한 뒤 settle (시뮬 s) 지난 영상."""
        self.light_pub.publish(String(data=color))
        self.pose_pub.publish(Pose2D(x=pose[0], y=pose[1], theta=pose[2]))
        self.spin_for(0.05)
        fut = self.reset_cli.call_async(Trigger.Request())
        t = time.time()
        while not fut.done() and time.time() - t < 5.0:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not fut.done() or not fut.result().success:
            return None
        t0 = self.clock
        t = time.time()
        while time.time() - t < 10.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.img is not None and self.img[0] >= t0 + settle:
                return self.img[1]
        return None


def sample_pose(rng, negative):
    if not negative:
        # 2차로 (y 4.57–5.11, 진행 +x) 커브 끝 ~ 횡단보도 앞: 신호등까지 약 0.8–3.2 m. 옆·방향을 흔든다
        x = rng.uniform(0.85, 3.10)
        y = rng.uniform(4.62, 5.06)
        yaw = math.radians(rng.uniform(-20, 20) + (rng.uniform(0, 35) if x < 1.4 else 0.0))
        return x, y, yaw
    # 배경: 신호등이 안 보이거나 잘 안 보이는 다른 구간 (3차선, 왼쪽 직선, 2차로를 거꾸로)
    k = rng.randrange(3)
    if k == 0:
        return rng.uniform(4.5, 8.0), rng.uniform(0.35, 1.5), math.radians(180 + rng.uniform(-20, 20))
    if k == 1:
        return rng.uniform(0.6, 0.85), rng.uniform(1.8, 3.8), math.radians(90 + rng.uniform(-20, 20))
    return rng.uniform(1.5, 3.1), rng.uniform(4.65, 5.05), math.radians(180 + rng.uniform(-25, 25))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--n', type=int, default=900)
    ap.add_argument('--neg', type=float, default=0.15, help='배경 자세 비율')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--prefix', default='s')
    ap.add_argument('--settle', type=float, default=0.6, help='리셋 뒤 기다릴 시뮬 시간 [s]')
    ap.add_argument('--contrast_min', type=float, default=1.6)
    a = ap.parse_args()
    out = os.path.expanduser(a.out)
    for d in ('images', 'labels'):
        os.makedirs(os.path.join(out, d), exist_ok=True)
    rclpy.init()
    n = Collector()
    rng = random.Random(a.seed)
    while (n.K is None or n.img is None) and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.reset_cli.wait_for_service(timeout_sec=10.0):
        print('/sim/reset 없음 — 시뮬 (bringup mode:=sim) 을 먼저 띄울 것')
        return
    stats = {'pos': [0, 0, 0], 'neg': 0, 'skip': 0}
    try:
        for i in range(a.n):
            neg = rng.random() < a.neg
            pose = sample_pose(rng, neg)
            color = rng.choice(COLORS)
            img = n.shot(pose, color, a.settle)
            if img is None:
                stats['skip'] += 1
                continue
            b = None if neg else project(n.K, n.size, pose)
            line = ''
            if b is not None:
                cl = cells(img, b)
                srt = sorted(cl) if cl else None
                ci = COLORS.index(color)
                if cl is None or cl.index(srt[2]) != ci or srt[2] / max(srt[1], 1.0) < a.contrast_min:
                    stats['skip'] += 1          # 잔상·너무 작음 — 버림
                    continue
                W, H = n.size
                x1, y1, x2, y2 = b
                line = f'{ci} {(x1 + x2) / 2 / W:.6f} {(y1 + y2) / 2 / H:.6f} {(x2 - x1) / W:.6f} {(y2 - y1) / H:.6f}\n'
                stats['pos'][ci] += 1
            else:
                stats['neg'] += 1
            name = f'{a.prefix}_{i:05d}'
            cv2.imwrite(os.path.join(out, 'images', name + '.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
            with open(os.path.join(out, 'labels', name + '.txt'), 'w') as f:
                f.write(line)
            if i % 50 == 0:
                print(f'{i}/{a.n} 빨강·노랑·초록 {stats["pos"]}, 배경 {stats["neg"]}, 버림 {stats["skip"]}', flush=True)
    finally:
        n.light_pub.publish(String(data='cycle'))
        print(f'끝: 빨강·노랑·초록 {stats["pos"]}, 배경 {stats["neg"]}, 버림 {stats["skip"]} → {out}', flush=True)
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
