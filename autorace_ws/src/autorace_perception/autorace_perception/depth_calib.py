"""뎁스 카메라 자세 보정 — 평평한 바닥에 세운 차의 뎁스 영상에서 바닥 평면을 맞춰 카메라 높이·숙인 각도·좌우 기울기를 잰다.

  ros2 run autorace_perception depth_calib                          # 지금 뎁스 토픽에서 10 장 (차는 안 움직임)
  ros2 run autorace_perception depth_calib --lane_h 0.176 --lane_pitch 12.3   # lane_calib 결과와 비교
  ros2 run autorace_perception depth_calib --load 결과폴더/depth.npz           # 저장해 둔 것 다시 계산

원리: 뎁스 픽셀 → 카메라 광학 좌표 (오른쪽 x, 아래 y, 앞 z) 의 3D 점. 화면 아래쪽 (바닥이 보이는 곳) 점에 RANSAC 으로 평면
  n·p = d 를 맞추고 (상자·다리 같은 바닥 아닌 점은 빠짐), 남은 점으로 다시 최소제곱.
  카메라 높이 = d (광심 → 바닥 거리), n = 카메라 좌표에서 '아래' 방향 → 숙인 각도 = asin(n_z), 좌우 기울기 = atan2(n_x, n_y).
  좌우 각도 (yaw) 는 바닥 평면으로는 알 수 없다 → lane_calib (노란 선 소실점) 으로 잰다.
lane_calib 와 비교 (--lane_h): lane_calib 는 차로 폭을 도면 값 (선 중심 간 0.52 m) 으로 놓고 높이를 푼다.
  뎁스 높이로 거꾸로 풀면 차로 폭 = 0.52 × 뎁스 높이 ÷ lane_calib 높이 → 트랙이 도면대로인지, 둘 중 하나가 틀렸는지 확인.
주의: 뎁스가 컬러에 맞춰져 (registration) 있지 않으면 잰 높이는 뎁스 센서 기준. Dabai 는 두 센서가 옆으로 나란해서 높이 차이는
  거의 없지만, 숙인 각도는 같다. 뎁스 배율 오차 (보통 1 % 안쪽) 는 그대로 높이 오차가 된다 — 줄자로 한 번 비교해 둘 것.
"""
import argparse
import math
import os
import sys

import numpy as np


def intrinsics_from_fov(w, h, hfov_deg, vfov_deg=None):
    fx = (w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = fx if vfov_deg is None else (h / 2.0) / math.tan(math.radians(vfov_deg) / 2.0)
    return fx, fy, w / 2.0, h / 2.0


def fov_deg(K, w, h):
    fx, fy = K[0], K[1]
    return math.degrees(2 * math.atan(w / (2 * fx))), math.degrees(2 * math.atan(h / (2 * fy)))


def merge_frames(frames):
    """여러 장 (m, 0 = 없음) → 픽셀별 중앙값 (절반 넘게 값이 있는 픽셀만). 잡음·구멍을 줄인다."""
    a = np.stack(frames).astype(np.float32)
    a[a <= 0] = np.nan
    n_ok = np.sum(np.isfinite(a), axis=0)
    import warnings
    with warnings.catch_warnings(), np.errstate(all='ignore'):
        warnings.simplefilter('ignore', RuntimeWarning)       # 모든 장에서 빈 픽셀
        med = np.nanmedian(a, axis=0)
    med[(n_ok * 2 <= len(frames)) | ~np.isfinite(med)] = 0.0
    return med


def backproject(depth, K, top_ratio, z_min, z_max, step):
    fx, fy, cx, cy = K
    h, w = depth.shape
    v, u = np.mgrid[int(h * top_ratio):h:step, 0:w:step]
    z = depth[v, u]
    ok = (z > z_min) & (z < z_max)
    z, u, v = z[ok], u[ok], v[ok]
    return np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1), u, v


def plane_lsq(P):
    c = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    n = vt[2]
    if n[1] < 0:                          # '아래' 방향 (광학 y 가 아래) 으로 맞춤
        n = -n
    return n, float(n @ c)


def fit_floor(P, thresh=0.01, iters=400, max_pitch_deg=60.0, h_range=(0.05, 0.6), seed=0):
    """RANSAC 바닥 평면. 카메라 아래쪽에 있고 (d 범위), 너무 기울지 않은 (n_y) 평면만 후보."""
    rng = np.random.default_rng(seed)
    best, best_n = None, 0
    cos_lim = math.cos(math.radians(max_pitch_deg))
    for _ in range(iters):
        s = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        L = np.linalg.norm(n)
        if L < 1e-9:
            continue
        n /= L
        if n[1] < 0:
            n = -n
        d = float(n @ s[0])
        if n[1] < cos_lim or not (h_range[0] < d < h_range[1]):
            continue
        inl = np.abs(P @ n - d) < thresh
        k = int(inl.sum())
        if k > best_n:
            best, best_n = inl, k
    if best is None:
        return None
    inl = best
    for th in (thresh, thresh * 0.6):     # 다시 맞추고 문턱을 좁혀 한 번 더
        n, d = plane_lsq(P[inl])
        inl = np.abs(P @ n - d) < th
    n, d = plane_lsq(P[inl])
    res = P[inl] @ n - d
    return dict(n=n, d=d, inliers=inl, rms=float(np.sqrt(np.mean(res ** 2))))


def pose_from_plane(n, d):
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, n[2]))))          # 아래로 +
    roll = math.degrees(math.atan2(n[0], n[1]))                          # + = 화면 오른쪽이 아래로
    return dict(height_m=d, pitch_deg=pitch, roll_deg=roll)


def band_heights(P, inl, n, pose, bands=((0.3, 0.6), (0.6, 0.9), (0.9, 1.3))):
    """앞 거리 구간별 바닥 높이 — 구간마다 다르면 뎁스가 거리에 따라 휘어 있다 (먼 곳 배율 오차)."""
    th = math.radians(pose['pitch_deg'])
    x, y, z = P[inl, 0], P[inl, 1], P[inl, 2]
    fwd = z * math.cos(th) - y * math.sin(th)
    out = []
    for a, b in bands:
        m = (fwd >= a) & (fwd < b)
        if m.sum() >= 50:
            out.append((a, b, float(np.median(P[inl][m] @ n)), int(m.sum())))
    return out


def grab_topic(depth_topic, info_topic, frames, timeout):
    import time
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    rclpy.init()
    node = rclpy.create_node('depth_calib')
    got = dict(frames=[], K=None, frame_id='')

    def on_img(m):
        if len(got['frames']) >= frames:
            return
        if m.encoding == '16UC1':
            a = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width].astype(np.float32) * 0.001
        elif m.encoding == '32FC1':
            a = np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width].copy()
            a[~np.isfinite(a)] = 0.0
        else:
            print(f'모르는 뎁스 형식 {m.encoding}', file=sys.stderr)
            return
        got['frames'].append(a)
        got['frame_id'] = m.header.frame_id

    def on_info(m):
        got['K'] = (m.k[0], m.k[4], m.k[2], m.k[5])
        got['wh'] = (m.width, m.height)

    node.create_subscription(Image, depth_topic, on_img, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, info_topic, on_info, qos_profile_sensor_data)
    t0 = time.time()
    while (len(got['frames']) < frames or got['K'] is None) and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    return got


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--depth_topic', default='/camera/depth/image_raw')
    ap.add_argument('--info_topic', default='/camera/depth/camera_info')
    ap.add_argument('--frames', type=int, default=10, help='평균 낼 장 수')
    ap.add_argument('--timeout', type=float, default=10.0)
    ap.add_argument('--load', default='', help='저장해 둔 depth.npz (토픽 대신)')
    ap.add_argument('--hfov', type=float, default=0.0, help='camera_info 가 없을 때 가로 화각 [deg]')
    ap.add_argument('--vfov', type=float, default=0.0)
    ap.add_argument('--top', type=float, default=0.5, help='화면 위에서 이 비율 아래만 (바닥이 보이는 곳)')
    ap.add_argument('--z_min', type=float, default=0.25, help='이보다 가까운 뎁스는 안 씀 [m] (센서 최소 거리)')
    ap.add_argument('--z_max', type=float, default=1.5, help='이보다 먼 뎁스는 안 씀 [m] (멀수록 잡음)')
    ap.add_argument('--thresh', type=float, default=0.01, help='바닥으로 볼 평면 거리 [m]')
    ap.add_argument('--step', type=int, default=2)
    ap.add_argument('--lane_h', type=float, default=0.0, help='lane_calib 가 잰 카메라 높이 [m] — 넣으면 비교')
    ap.add_argument('--lane_pitch', type=float, default=float('nan'), help='lane_calib 가 잰 숙인 각도 [deg]')
    ap.add_argument('--lane_m', type=float, default=0.52, help='lane_calib 에 쓴 차로 폭 (도면: 선 중심 간 0.52 m)')
    ap.add_argument('--out', default='', help='결과 폴더 (depth.npz + 바닥 그림)')
    a, _ = ap.parse_known_args(argv)

    if a.load:
        z = np.load(a.load)
        depth, K, frame_id = z['depth'], tuple(float(v) for v in z['K']), str(z['frame_id'])
        n_frames = int(z['n_frames'])
    else:
        got = grab_topic(a.depth_topic, a.info_topic, a.frames, a.timeout)
        if not got['frames']:
            print(f'뎁스 영상을 못 받음 ({a.depth_topic}) — 카메라 드라이버가 켜져 있는지 확인', file=sys.stderr)
            return 1
        depth = merge_frames(got['frames'])
        n_frames, frame_id = len(got['frames']), got['frame_id']
        K = got['K']
    h, w = depth.shape
    if K is None:
        if a.hfov <= 0:
            print('camera_info 를 못 받음 — --hfov (가능하면 --vfov 도) 를 넣어 주세요', file=sys.stderr)
            return 1
        K = intrinsics_from_fov(w, h, a.hfov, a.vfov or None)
        print(f'camera_info 없음 → 화각으로 계산한 fx {K[0]:.1f}')

    P, u, v = backproject(depth, K, a.top, a.z_min, a.z_max, a.step)
    if len(P) < 500:
        print(f'쓸 수 있는 뎁스 점이 {len(P)} 개뿐 — 바닥이 화면 아래쪽에 보이는지, 바닥이 검거나 반짝여 뎁스가 비지 않는지 확인',
              file=sys.stderr)
        return 2
    fit = fit_floor(P, a.thresh)
    if fit is None:
        print('바닥 평면을 못 찾음 — 차 앞에 물건을 치우고 평평한 바닥이 보이게 해 주세요', file=sys.stderr)
        return 2
    pose = pose_from_plane(fit['n'], fit['d'])
    frac = fit['inliers'].mean()
    hfov, vfov = fov_deg(K, w, h)

    print(f'뎁스 {w}×{h}, {n_frames} 장 중앙값, frame {frame_id or "?"}, fx {K[0]:.1f} fy {K[1]:.1f} cx {K[2]:.1f} cy {K[3]:.1f}')
    print(f'화각: 가로 {hfov:.1f}°, 세로 {vfov:.1f}°')
    print(f'바닥 점 {int(fit["inliers"].sum())} 개 (쓴 점의 {frac * 100:.0f} %), 평면 맞춤 오차 {fit["rms"] * 1000:.1f} mm')
    print(f'카메라 높이 {pose["height_m"]:.3f} m, 아래로 {pose["pitch_deg"]:+.2f}°, 좌우 기울기 {pose["roll_deg"]:+.2f}° '
          f'({"오른쪽" if pose["roll_deg"] > 0 else "왼쪽"}이 아래로)')
    bands = band_heights(P, fit['inliers'], fit['n'], pose)
    if bands:
        print('거리별 바닥 높이: ' + ', '.join(f'{a_:.1f}–{b_:.1f} m {hh * 1000:.0f} mm' for a_, b_, hh, _ in bands))
        spread = max(b[2] for b in bands) - min(b[2] for b in bands)
        if spread > 0.008:
            print(f'  ⚠ 구간 차이 {spread * 1000:.0f} mm — 바닥이 평평하지 않거나 먼 곳 뎁스가 휘어 있음. --z_max 를 줄여 다시 해 보세요')
    warn = []
    if frac < 0.5:
        warn.append(f'바닥으로 맞은 점이 {frac * 100:.0f} % 뿐 — 앞에 물건이 많거나 바닥이 고르지 않음')
    if fit['rms'] > 0.006:
        warn.append(f'평면 오차 {fit["rms"] * 1000:.1f} mm 로 큼 — 뎁스 잡음·바닥 굴곡')
    if abs(pose['roll_deg']) > 1.0:
        warn.append(f'좌우 기울기 {pose["roll_deg"]:+.1f}° — lane_calib·depth_obstacle 는 기울기 0 을 가정. 카메라를 바로 다는 게 좋음')
    for s in warn:
        print('⚠ ' + s)

    if a.lane_h > 0:
        lane_w = a.lane_m * pose['height_m'] / a.lane_h
        print(f'\nlane_calib 비교: 높이 {a.lane_h:.3f} m (뎁스 {pose["height_m"]:.3f}, 차이 {(pose["height_m"] - a.lane_h) * 1000:+.0f} mm)')
        if math.isfinite(a.lane_pitch):
            print(f'  숙인 각도 {a.lane_pitch:+.2f}° (뎁스 {pose["pitch_deg"]:+.2f}°, 차이 {pose["pitch_deg"] - a.lane_pitch:+.2f}°)')
        print(f'  뎁스 높이로 풀면 차로 폭 {lane_w:.3f} m (도면 {a.lane_m:.2f} m, 차이 {(lane_w - a.lane_m) * 1000:+.0f} mm)')
        if abs(lane_w - a.lane_m) > 0.015:
            print('  ⚠ 1.5 cm 넘게 다름 — 트랙 폭을 줄자로 재 보고, 맞으면 카메라 높이를 줄자로 재서 둘 중 무엇이 틀렸는지 확인')
        else:
            print('  ✓ 도면 폭과 맞음 — 두 보정이 서로 맞는다')

    hv = pose['height_m']
    print('\n# depth_obstacle.yaml')
    print(f'    cam_height: {hv:.3f}\n    cam_pitch_deg: {pose["pitch_deg"]:.2f}')
    print('# lidar_processor.yaml')
    print(f'    tall_cam_h: {hv:.3f}\n    tall_cam_pitch_deg: {pose["pitch_deg"]:.2f}\n    tall_cam_vfov_deg: {vfov:.1f}')
    print('# mission_manager.yaml')
    print(f'    m7_cam_hfov_deg: {hfov:.1f}')
    print('# (cam_yaw_deg 는 lane_calib 값을 쓴다. 컬러 화각은 /camera/color/camera_info 로 따로 확인 → lane_follower camera_hfov_deg)')

    if a.out:
        os.makedirs(a.out, exist_ok=True)
        np.savez_compressed(os.path.join(a.out, 'depth.npz'), depth=depth, K=np.array(K), frame_id=frame_id,
                            n_frames=n_frames)
        try:
            import cv2
            vis = np.zeros((h, w, 3), np.uint8)
            dz = np.clip(depth / max(a.z_max, 1e-3) * 255, 0, 255).astype(np.uint8)
            vis[:] = dz[:, :, None] // 2
            inl = fit['inliers']
            vis[v[inl], u[inl]] = (0, 200, 0)
            vis[v[~inl], u[~inl]] = (0, 0, 220)
            cv2.imwrite(os.path.join(a.out, 'depth_floor.png'), vis)
            print(f'\n저장: {a.out}/depth.npz (다시 계산: --load), depth_floor.png (초록 = 바닥, 빨강 = 바닥 아님)')
        except ImportError:
            print(f'\n저장: {a.out}/depth.npz')
    return 0


if __name__ == '__main__':
    sys.exit(main())
