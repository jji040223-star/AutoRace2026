"""카메라 자세 보정 — 직선 차로 가운데에 똑바로 세운 차의 카메라 사진 한 장에서 카메라 높이·숙인 각도·좌우 각도와 노란색 문턱을 잰다.

  ros2 run autorace_lane lane_calib 사진.png [--hfov 71] [--out 결과폴더]
  ros2 run autorace_lane lane_calib --topic /camera/color/image_raw --out 결과폴더     # 지금 카메라에서 한 장

원리: 바닥의 두 노란 선은 lane_m (선 중심 간 0.52 m) 간격의 평행선이다.
  1) 노란 픽셀 → 왼쪽·오른쪽 선을 직선 (u = a·v + b) 으로 맞춤
  2) 두 직선이 만나는 점 (소실점) = 차 진행 방향 → 카메라 숙인 각도 (pitch)·좌우 각도 (yaw)
  3) 두 선 간격이 lane_m 이 되는 카메라 높이 + 차가 차로 가운데에서 옆으로 벗어난 거리 (격자 탐색)
  4) 선 위 픽셀의 HSV 분포 → yellow_hsv_lo / hi 제안
결과: lane_follower.yaml (camera_height, camera_pitch_deg, camera_yaw_deg, yellow_hsv_*) 와
      depth_obstacle.yaml (cam_height, cam_pitch_deg, cam_yaw_deg) 에 넣을 값, 보정 전·후 BEV 그림 (--out).
가정: 카메라 좌우 기울기 (roll) 0, 렌즈 왜곡 없음, 주점 = 영상 가운데. 사진에서 두 노란 선이 화면 아래까지 보여야 한다.
주의: 잰 좌우 각도 = 카메라 장착 각도 + 차가 차로와 비뚤어진 각도. 차를 1° 비뚤게 세우면 1° 가 섞인다 →
      차를 다시 세워 2~3 장 찍고 평균을 쓴다 (높이·숙인 각도는 차 방향과 거의 무관).
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

from .lane_follower import DETECTOR_PARAMS, bev_src_from_camera
from .sliding_window import SlidingWindowLaneDetector


def cam_rows(pitch, yaw):
    """차 기준 (앞 X, 왼쪽 Y, 위 Z) → 카메라 광학 (오른쪽, 아래, 앞) 회전 행렬. sim2d·bev_src_from_camera 와 같은 정의."""
    cp, sp, cy, sy = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[sy, -cy, 0.0],
                     [-sp * cy, -sp * sy, -cp],
                     [cp * cy, cp * sy, -sp]])


def yellow_mask(bgr, lo, hi):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def fit_lines(mask, v_min):
    """화면 아래쪽 노란 덩어리 중 왼쪽·오른쪽 선을 골라 u = a·v + b 로 맞춤. 각 선: (a, b, 행별 점 [(v, u)])."""
    h, w = mask.shape
    m = mask.copy()
    m[:v_min] = 0
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    cands = []
    for k in range(1, n):
        if stats[k, cv2.CC_STAT_AREA] < 150 or stats[k, cv2.CC_STAT_HEIGHT] < 30:
            continue
        ys, xs = np.nonzero(lab == k)
        rows = np.unique(ys)
        pts = []
        for v in rows:
            xv = xs[ys == v]
            if xv.min() <= 1 or xv.max() >= w - 2:
                continue                    # 화면 옆 끝에 잘린 행은 가운데가 안쪽으로 치우친다
            pts.append((v, 0.5 * (xv.min() + xv.max())))
        if len(pts) < 20:
            continue
        pts = np.array(pts)
        a, b = np.polyfit(pts[:, 0], pts[:, 1], 1)
        resid = np.abs(pts[:, 1] - (a * pts[:, 0] + b))
        good = resid < max(3.0, 3 * np.median(resid))
        a, b = np.polyfit(pts[good, 0], pts[good, 1], 1)
        cands.append((stats[k, cv2.CC_STAT_AREA], a, b, pts[good]))
    # 화면 아래 끝에서 가운데 왼쪽 / 오른쪽에 있는 가장 큰 덩어리
    left = [c for c in cands if c[1] * (h - 1) + c[2] < w / 2]
    right = [c for c in cands if c[1] * (h - 1) + c[2] >= w / 2]
    if not left or not right:
        return None
    L = max(left, key=lambda c: c[0])
    R = max(right, key=lambda c: c[0])
    return (L[1], L[2], L[3]), (R[1], R[2], R[3])


def solve(lines, K, lane_m):
    (al, bl, pl), (ar, br, pr) = lines
    if abs(al - ar) < 1e-6:
        raise ValueError('두 선이 화면에서 평행 — 소실점을 못 구함')
    v_vp = (br - bl) / (al - ar)
    u_vp = al * v_vp + bl
    d = np.linalg.inv(K) @ np.array([u_vp, v_vp, 1.0])
    d /= np.linalg.norm(d)
    yaw = math.asin(max(-1.0, min(1.0, d[0])))
    pitch = math.atan2(-d[1], d[2])
    # 높이 h, 차의 옆 어긋남 c (차가 차로 가운데보다 왼쪽이면 +) — 두 선 점들과의 가로 오차 최소
    R = cam_rows(pitch, yaw)
    vs_l, us_l = pl[:, 0], pl[:, 1]
    vs_r, us_r = pr[:, 0], pr[:, 1]

    def err(h, c):
        out = 0.0
        for Y0, vs, us in ((lane_m / 2 - c, vs_l, us_l), (-lane_m / 2 - c, vs_r, us_r)):
            # 화면 행 v 를 지나는 바닥선 Y = Y0 위 점 → u 예측 (광선과 바닥 교점 대신: 선 위 X 를 풀어 투영)
            # 행 v 에서: v = cy + f·yc/zc 를 X 에 대해 풀면 (yc, zc 는 X 에 선형)
            P0 = R @ np.array([0.0, Y0, -h])
            P1 = R @ np.array([1.0, 0.0, 0.0])
            f, cx, cy = K[0, 0], K[0, 2], K[1, 2]
            t = (vs - cy) / f
            X = (t * P0[2] - P0[1]) / (P1[1] - t * P1[2])
            xc, zc = P0[0] + X * P1[0], P0[2] + X * P1[2]
            out += float(np.mean((cx + f * xc / zc - us) ** 2))
        return out
    best = None
    for h in np.arange(0.06, 0.45, 0.002):
        for c in np.arange(-0.20, 0.2001, 0.004):
            e = err(h, c)
            if best is None or e < best[0]:
                best = (e, h, c)
    # 네 값 (숙임·좌우·높이·어긋남) 을 같이 다듬기 — 좌표 하강, 걸음을 줄여 가며
    x = np.array([pitch, yaw, best[1], best[2]])
    steps = np.array([math.radians(0.5), math.radians(0.5), 0.004, 0.004])

    def cost(x):
        nonlocal R
        R = cam_rows(x[0], x[1])
        return err(x[2], x[3])
    e0 = cost(x)
    for _ in range(60):
        improved = False
        for i in range(4):
            for sgn in (1, -1):
                y = x.copy()
                y[i] += sgn * steps[i]
                e = cost(y)
                if e < e0:
                    x, e0, improved = y, e, True
        if not improved:
            steps /= 2
            if steps[0] < math.radians(0.005):
                break
    return dict(pitch_deg=math.degrees(x[0]), yaw_deg=math.degrees(x[1]), height_m=float(x[2]), offset_m=float(x[3]),
                rms_px=math.sqrt(e0 / 2), vp=(u_vp, v_vp))


def suggest_hsv(bgr, lines):
    """맞춘 선 위 (±1 px) 픽셀의 HSV 2~98 % 범위 + 여유 → 노란색 문턱 제안, 그리고 매트와 겹치는지."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]
    on = []
    for a, b, pts in lines:
        for v in pts[:, 0].astype(int):
            u = int(round(a * v + b))
            if 1 <= u < w - 1:
                on.append(hsv[v, u - 1:u + 2])
    on = np.concatenate(on)
    lo = np.percentile(on, 2, axis=0)
    hi = np.percentile(on, 98, axis=0)
    lo_s = [int(max(0, lo[0] - 4)), int(max(0, lo[1] * 0.7)), int(max(0, lo[2] * 0.7))]
    hi_s = [int(min(179, hi[0] + 4)), 255, 255]
    # 매트 (두 선 사이 아래쪽 가운데) 가 제안 범위에 드는 비율
    mid = hsv[int(h * 0.75):, int(w * 0.4):int(w * 0.6)].reshape(-1, 3)
    inside = np.all((mid >= lo_s) & (mid <= hi_s), axis=1).mean()
    return lo_s, hi_s, float(inside), (lo, hi)


def bev_check(bgr, src, process_width=480, dst=(0.3, 0.0, 0.7, 0.0, 0.7, 1.0, 0.3, 1.0), yellow=None):
    """lane_follower 와 같은 검출기로 BEV 를 만들어 차로 폭 (px) 과 중심 offset 을 본다."""
    h, w = bgr.shape[:2]
    ph = int(round(h * process_width / w))
    img = cv2.resize(bgr, (process_width, ph), interpolation=cv2.INTER_AREA)
    cfg = dict(DETECTOR_PARAMS)
    cfg.update(use_white=False, use_yellow=True, birdeye_src_ratio=list(src), birdeye_dst_ratio=list(dst), lane_width_tol=0.25)
    if yellow:
        cfg['yellow_hsv_lo'], cfg['yellow_hsv_hi'] = tuple(yellow[0]), tuple(yellow[1])
    for k in ('white_hsv_lo', 'white_hsv_hi', 'yellow_hsv_lo', 'yellow_hsv_hi'):
        cfg[k] = tuple(int(v) for v in cfg[k])
    det = SlidingWindowLaneDetector(**cfg)
    r = None
    for _ in range(8):                      # 몇 번 돌려 폭 학습·EMA 가 자리 잡게
        r = det.process(img, draw_debug=True)
    return r


def grab_topic(topic, timeout=10.0):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('lane_calib')
    got = {}

    def cb(m):
        ch = 4 if m.encoding.endswith('a8') else 3
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step // ch, ch)[:, :m.width, :3]
        got['img'] = np.ascontiguousarray(a[:, :, ::-1] if m.encoding.startswith('rgb') else a)
    node.create_subscription(Image, topic, cb, qos_profile_sensor_data)
    import time
    t0 = time.time()
    while 'img' not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    return got.get('img')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('image', nargs='?', help='카메라 사진 (png/jpg). 없으면 --topic 에서 한 장')
    ap.add_argument('--topic', default='/camera/color/image_raw')
    ap.add_argument('--hfov', type=float, default=71.0, help='가로 화각 [deg] (lane_follower camera_hfov_deg 와 같게)')
    ap.add_argument('--lane_m', type=float, default=0.52, help='두 노란 선 중심 간격 [m]')
    ap.add_argument('--yellow_lo', type=int, nargs=3, default=[10, 40, 90], help='선 찾기용 느슨한 노랑 HSV 하한')
    ap.add_argument('--yellow_hi', type=int, nargs=3, default=[40, 255, 255])
    ap.add_argument('--top', type=float, default=0.55, help='화면 위에서 이 비율 아래만 선 찾기 (먼 곳·배경 제외)')
    ap.add_argument('--near', type=float, default=0.37)
    ap.add_argument('--far', type=float, default=0.75)
    ap.add_argument('--out', default='', help='그림 저장 폴더')
    a, _ = ap.parse_known_args()
    bgr = cv2.imread(a.image) if a.image else grab_topic(a.topic)
    if bgr is None:
        print('사진을 못 읽음', file=sys.stderr)
        sys.exit(1)
    h, w = bgr.shape[:2]
    f = (w / 2.0) / math.tan(math.radians(a.hfov) / 2.0)
    K = np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1.0]])
    mask = yellow_mask(bgr, a.yellow_lo, a.yellow_hi)
    lines = fit_lines(mask, int(h * a.top))
    if lines is None:
        print('노란 선 두 개를 못 찾음 — 차를 직선 차로 가운데에 두고 양쪽 노란 선이 화면 아래까지 보이게 하거나, '
              '--yellow_lo/--yellow_hi 를 넓혀 보세요', file=sys.stderr)
        if a.out:
            os.makedirs(a.out, exist_ok=True)
            cv2.imwrite(os.path.join(a.out, 'mask.png'), mask)
        sys.exit(2)
    sol = solve(lines, K, a.lane_m)
    lo_s, hi_s, mat_in, raw = suggest_hsv(bgr, lines)
    src_new = bev_src_from_camera(480, int(round(h * 480 / w)), 0.35, a.hfov, sol['height_m'], sol['pitch_deg'],
                                  a.near, a.far, 0.26, sol['yaw_deg'])
    src_old = bev_src_from_camera(480, int(round(h * 480 / w)), 0.35, a.hfov, 0.175, 0.0, a.near, a.far, 0.26, 0.0)
    r_new = bev_check(bgr, src_new, yellow=(lo_s, hi_s))
    r_old = bev_check(bgr, src_old)
    print(f"소실점 ({sol['vp'][0]:.1f}, {sol['vp'][1]:.1f}) px, 선 맞춤 오차 {sol['rms_px']:.2f} px")
    print(f"카메라 높이 {sol['height_m']:.3f} m, 아래로 {sol['pitch_deg']:+.2f}°, 왼쪽으로 {sol['yaw_deg']:+.2f}°, "
          f"차가 차로 가운데에서 {sol['offset_m'] * 100:+.1f} cm ({'왼쪽' if sol['offset_m'] > 0 else '오른쪽'})")
    print(f"노란 선 HSV 2~98 %: H {raw[0][0]:.0f}–{raw[1][0]:.0f}, S {raw[0][1]:.0f}–{raw[1][1]:.0f}, V {raw[0][2]:.0f}–{raw[1][2]:.0f} "
          f"→ 제안 {lo_s} ~ {hi_s} (매트가 이 범위에 드는 비율 {mat_in * 100:.1f} %{' — 너무 큼, 조명 확인' if mat_in > 0.02 else ''})")
    for tag, r in (('지금 설정 (높이 0.175, 0°)', r_old), ('보정 뒤', r_new)):
        if r is not None:
            print(f"  BEV [{tag}]: 차로 폭 {r.lane_width_px:.0f} px (목표 192), offset {r.offset:+.3f}, 선 {'양쪽' if r.line_side == 2 else r.line_side}")
    print('\n# lane_follower.yaml')
    print(f"    camera_height: {sol['height_m']:.3f}\n    camera_pitch_deg: {sol['pitch_deg']:.2f}\n    camera_yaw_deg: {sol['yaw_deg']:.2f}")
    print(f"    yellow_hsv_lo: {lo_s}\n    yellow_hsv_hi: {hi_s}")
    print('# depth_obstacle.yaml')
    print(f"    cam_height: {sol['height_m']:.3f}\n    cam_pitch_deg: {sol['pitch_deg']:.2f}\n    cam_yaw_deg: {sol['yaw_deg']:.2f}")
    if abs(sol['offset_m']) > 0.03:
        print(f"\n참고: 차 (또는 카메라 장착) 가 차로 가운데에서 {abs(sol['offset_m']) * 100:.1f} cm 벗어나 있음. 차를 가운데에 다시 두고 "
              f"찍어도 남으면 카메라가 차 가운데에 안 달린 것.")
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        vis = bgr.copy()
        for al, bl, pts in lines:
            for v in pts[:, 0].astype(int)[::4]:
                cv2.circle(vis, (int(round(al * v + bl)), v), 1, (255, 0, 255), -1)
            v0, v1 = int(sol['vp'][1]), h - 1
            cv2.line(vis, (int(al * v0 + bl), v0), (int(al * v1 + bl), v1), (0, 255, 0), 1)
        cv2.circle(vis, (int(sol['vp'][0]), int(sol['vp'][1])), 5, (0, 0, 255), 2)
        cv2.imwrite(os.path.join(a.out, 'calib_lines.png'), vis)
        cv2.imwrite(os.path.join(a.out, 'calib_mask.png'), mask)
        for tag, r in (('before', r_old), ('after', r_new)):
            if r is not None and r.debug is not None:
                cv2.imwrite(os.path.join(a.out, f'bev_{tag}.png'), r.debug)
        print(f'\n그림: {a.out}/calib_lines.png, bev_before.png, bev_after.png')
