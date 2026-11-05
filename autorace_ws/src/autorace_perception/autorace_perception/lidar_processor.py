"""라이다 라바콘 노드 (v3 M8 — 라이다 주, 카메라 보조).

  sub  scan_topic   (기본 /scan, sensor_msgs/LaserScan, laser_link — base_link 와 같은 방향, 앞 laser_x)
  sub  odom_topic   (기본 /odom)
  pub  cones_topic  (기본 /perception/cones, geometry_msgs/PoseArray, odom 좌표) — 기억한 라바콘 중심 (min_hits 번 이상 본 것)
  pub  marker_topic (기본 /perception/cone_markers, visualization_msgs/MarkerArray) — RViz: 기억 (주황 원기둥), 이번 스캔 묶음 (노랑 점)

처리: 앞 ±fov_deg, range_min~range_max 의 점 → 각도 순으로 이웃 점 거리 cluster_gap_m 안이면 같은 묶음 → 점 min_points 이상,
크기 (양 끝 거리) max_size_m 이하만 라바콘 (정차 LIMO 0.22 m·벽은 버림). 중심 = 점 평균을 라바콘 반지름만큼 안쪽으로.
라이다 높이 (시뮬 0.111 m) 에서 라바콘 단면은 지름 약 4.3 cm, 0.96° 간격이면 0.5 m 앞 5 점, 1.5 m 앞 1~2 점.
기억: odom 좌표로 merge_m 안이면 같은 라바콘 (평균). 시간으로 지우지 않고, 차 진행 방향으로 pass_margin_m 넘게 뒤로 지나갔거나
max_keep_m 넘게 멀어지거나, 그 자리를 비추는 빔이 free_frames 번 연속 그 너머까지 비어 있으면 (옮긴 사람 발 자리) 지운다.
뎁스 장애물 (/perception/obstacles) 중 높고 좁은 것의 바닥 상자 (tall_keep_s 동안 기억) 안의 점은 내지 않는다 —
라이다 한 줄로는 사람 다리와 라바콘이 구분 안 되지만 높이는 다르다 (카메라 시야 앞 ±35° 안에서 본 것만). 리셋 (odom 이 0.5 m 넘게 튐) 이면 다 지운다.
"""
import bisect
import collections
import math
import signal

import numpy as np
import rclpy
from autorace_msgs.msg import ObstacleArray
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

CAR_HALF_L, CAR_HALF_W = 0.161, 0.109


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        p = self.declare_parameter
        p('scan_topic', '/scan')
        p('odom_topic', '/odom')
        p('cones_topic', '/perception/cones')
        p('marker_topic', '/perception/cone_markers')
        p('odom_frame', 'odom')
        p('laser_x', 0.103)                # base_link → 라이다 [m]
        p('fov_deg', 90.0)                 # 앞 ±
        p('range_min', 0.12)
        p('range_max', 1.5)
        p('cluster_gap_m', 0.06)           # 각도 순 이웃 점이 이 안이면 같은 묶음
        p('min_points', 1)
        p('max_size_m', 0.12)              # 묶음 양 끝 거리가 이보다 크면 라바콘 아님 (LIMO 0.22, 벽)
        p('cone_radius_m', 0.022)          # 라이다 높이 단면 반지름 — 점 평균을 이만큼 안쪽으로
        p('merge_m', 0.08)                 # 기억: 이 안이면 같은 라바콘 (라바콘 간격 0.3 m)
        p('min_hits', 2)                   # 이만큼 본 것만 낸다 (한 번 튄 점 거르기)
        p('pass_margin_m', 0.4)            # 진행 방향으로 이만큼 지나간 뒤에 지움
        p('max_keep_m', 3.0)
        p('free_frames', 3)                # 기억 자리 너머까지 빈 스캔이 이만큼 연속이면 지움 (0 = 끔)
        p('free_clear_m', 0.10)            #   '너머' = 기억 거리 + 이만큼보다 멀리 맞음 (또는 반사 없음)
        p('free_win_m', 0.04)              #   기억 자리 좌우 이 폭 안 빔을 모두 봄 (먼 콘은 빔 사이로 빠질 수 있어 이웃 빔까지)
        p('free_frames_inf', 6)            # 너머가 '반사 없음' 뿐인 증거는 이만큼 (실제 라이다는 먼 콘 반사가 가끔 빠짐)
        p('free_max_yaw_rate', 0.3)        # 이보다 빨리 돌면 지우기 판단 안 함 [rad/s] (스캔 한 바퀴 동안 돌아 방향이 비틀림)
        p('obstacles_topic', '/perception/obstacles')   # 뎁스 장애물 (높이 있음) — 높은 것의 앞면에 있는 라이다 점은 라바콘 아님
        p('tall_min_h', 0.35)              # 뎁스 장애물이 이보다 높으면 (사람 다리 등, 라바콘 0.19 m) (0 = 끔).
        p('tall_floor_h', 0.27)            #   가까운 다리는 화면 위가 잘려 낮게 재짐 → 기준 = min(tall_min_h, 화면 위 높이 − 0.03), 이보다 낮게는 안 내림
        p('tall_cam_h', 0.175)             #   뎁스 카메라 높이·세로 화각·숙임 (화면 위 높이 계산)
        p('tall_cam_vfov_deg', 45.0)
        p('tall_cam_pitch_deg', 0.0)
        p('tall_max_w', 0.25)              #   이보다 넓은 장애물은 안 씀 (합쳐진 묶음·벽·LIMO)
        p('tall_match_m', 0.04)            #   라이다 점이 그 장애물 바닥 상자 (이만큼 넓힘) 안이면 그 장애물의 점
        p('tall_keep_s', 15.0)             #   이만큼 기억 — 차 옆·뒤 (카메라 시야 밖) 에서 발을 옮겨도 이전에 본 자리는 남게
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self._tall = []                    # [(차 x, y, yaw, x_near, x_far, y_right, y_left, 시각)] — 본 때 차 기준 바닥 상자
        self._odom = collections.deque(maxlen=200)    # (시각, x, y, yaw)
        self._pose = None
        self._cones = []                   # {'x', 'y', 'n', 'hits'}
        self.pub = self.create_publisher(PoseArray, g('cones_topic'), 10)
        self.mk_pub = self.create_publisher(MarkerArray, g('marker_topic'), 2)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 50)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(ObstacleArray, g('obstacles_topic'), self.on_obstacles, 10)
        self._last_log = 0.0
        self.get_logger().info(f"lidar_processor: {g('scan_topic')} → {g('cones_topic')} (앞 ±{g('fov_deg'):.0f}°, "
                               f"{g('range_min')}–{g('range_max')} m, 묶음 ≤ {g('max_size_m')} m)")

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self._pose is not None and math.dist(self._pose[:2], (p.x, p.y)) > 0.5:
            self._cones = []               # 시뮬 리셋 (출발 자세로 텔레포트)
            self._odom.clear()
            self.get_logger().info('오도메트리가 튐 (리셋) — 라바콘 기억 지움')
        self._pose = (p.x, p.y, yaw)
        self._odom.append((stamp_s(msg.header), p.x, p.y, yaw))

    def pose_at(self, t):
        h = self._odom
        if not h:
            return self._pose
        i = bisect.bisect_left([e[0] for e in h], t)
        e = min((h[j] for j in (i - 1, i) if 0 <= j < len(h)), key=lambda e: abs(e[0] - t))
        return e[1:]

    def clusters(self, msg):
        """이번 스캔의 라바콘 후보 [(x, y, 점 수)] base_link 기준."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        r = np.asarray(msg.ranges, np.float32)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        a = np.arctan2(np.sin(a), np.cos(a))
        ok = (np.isfinite(r) & (r >= max(g('range_min'), msg.range_min)) & (r <= min(g('range_max'), msg.range_max))
              & (np.abs(a) <= math.radians(g('fov_deg'))))
        r, a = r[ok], a[ok]
        order = np.argsort(a)
        r, a = r[order], a[order]
        x = r * np.cos(a) + g('laser_x')
        y = r * np.sin(a)
        body = (np.abs(x) < CAR_HALF_L + 0.005) & (np.abs(y) < CAR_HALF_W + 0.005)
        x, y, r, a = x[~body], y[~body], r[~body], a[~body]
        out, start = [], 0
        for i in range(1, len(x) + 1):
            if i == len(x) or math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]) > g('cluster_gap_m'):
                n = i - start
                if n >= g('min_points') and math.hypot(x[i - 1] - x[start], y[i - 1] - y[start]) <= g('max_size_m'):
                    mx, my = float(x[start:i].mean()), float(y[start:i].mean())
                    # 보이는 면 → 중심: 라이다에서 본 방향으로 반지름만큼 더
                    dx, dy = mx - g('laser_x'), my
                    d = math.hypot(dx, dy) or 1.0
                    out.append((mx + g('cone_radius_m') * dx / d, my + g('cone_radius_m') * dy / d, n))
                start = i
        return out

    def on_scan(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        pose = self.pose_at(stamp_s(msg.header))
        if pose is None:
            return
        px, py, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        now_cl = []
        for bx, by, n in self.clusters(msg):
            wx, wy = px + c * bx - s * by, py + s * bx + c * by
            now_cl.append((wx, wy))
            best = min(self._cones, key=lambda k: math.dist((k['x'], k['y']), (wx, wy)), default=None)
            if best is not None and math.dist((best['x'], best['y']), (wx, wy)) < g('merge_m'):
                w = min(best['n'], 20)
                best['x'] = (best['x'] * w + wx) / (w + 1)
                best['y'] = (best['y'] * w + wy) / (w + 1)
                best['n'] += 1
            else:
                self._cones.append({'x': wx, 'y': wy, 'n': 1})
        self.clear_free(msg, pose)
        self.prune_tall(stamp_s(msg.header))
        # 지나간 것·멀어진 것만 지움 (시간으로 안 지움)
        if self._pose is not None:
            qx, qy, qyaw = self._pose
            hc, hs = math.cos(qyaw), math.sin(qyaw)
            self._cones = [k for k in self._cones
                           if (k['x'] - qx) * hc + (k['y'] - qy) * hs > -g('pass_margin_m')
                           and math.dist((k['x'], k['y']), (qx, qy)) < g('max_keep_m')]
        keep = [k for k in self._cones if k['n'] >= g('min_hits') and not self.is_tall(k['x'], k['y'])]
        pa = PoseArray()
        pa.header.stamp = msg.header.stamp
        pa.header.frame_id = g('odom_frame')
        for k in keep:
            ps = Pose()
            ps.position.x, ps.position.y = k['x'], k['y']
            ps.orientation.w = 1.0
            pa.poses.append(ps)
        self.pub.publish(pa)
        self.publish_markers(pa, now_cl)
        now = self.get_clock().now().nanoseconds * 1e-9
        if keep and now - self._last_log >= 2.0:
            self._last_log = now
            self.get_logger().info(f'라바콘 기억 {len(keep)} 개 (이번 스캔 묶음 {len(now_cl)} 개)')

    def on_obstacles(self, msg):
        """뎁스 장애물 중 높은 것 (사람 다리·의자 등 — 라이다로는 라바콘과 구분 안 되는 가는 기둥) 의 바닥 상자를 본 때 차 자세와 같이 기억."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        if g('tall_min_h') <= 0:
            return
        t = stamp_s(msg.header)
        pose = self.pose_at(t)
        if pose is None:
            return
        px, py, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        top = math.radians(g('tall_cam_vfov_deg') / 2.0 - g('tall_cam_pitch_deg'))
        for o in msg.obstacles:
            if o.y_left - o.y_right > g('tall_max_w'):
                continue
            h_top = g('tall_cam_h') + max(0.0, o.x_near) * math.tan(top)
            thr = max(g('tall_floor_h'), min(g('tall_min_h'), h_top - 0.03))
            if o.height < thr:
                continue
            self._tall.append((px, py, yaw, o.x_near, o.x_far, o.y_right, o.y_left, t))
        self.prune_tall(t)

    def prune_tall(self, t):
        self._tall = [b for b in self._tall if t - b[7] < self.get_parameter('tall_keep_s').value][-200:]

    def is_tall(self, x, y):
        """라이다 점 (odom) 이 기억한 높은 장애물의 바닥 상자 (본 때의 차 좌표 기준, tall_match_m 만큼 넓힘) 안 — 그 장애물의 점.
        폭이 넓은 묶음 (사람 + 콘이 합쳐진 것 등) 은 on_obstacles 에서 이미 뺐다 (tall_max_w)."""
        m = self.get_parameter('tall_match_m').value
        for px, py, yaw, xn, xf, yr, yl, _ in self._tall:
            c, s = math.cos(yaw), math.sin(yaw)
            bx, by = (x - px) * c + (y - py) * s, -(x - px) * s + (y - py) * c
            if xn - m <= bx <= max(xf, xn + 0.05) + m and yr - m <= by <= yl + m:
                return True
        return False

    def yaw_rate(self):
        h = self._odom
        if len(h) < 2:
            return 0.0
        t1, _, _, y1 = h[-1]
        for t0, _, _, y0 in reversed(h):
            if t1 - t0 >= 0.15:
                d = math.atan2(math.sin(y1 - y0), math.cos(y1 - y0))
                return d / (t1 - t0)
        return 0.0

    def clear_free(self, msg, pose):
        """기억한 점 자리를 이번 스캔이 비추는데 그 너머까지 비어 있으면 (그 방향 ±free_win_m 안 빔이 모두 기억 거리 + free_clear_m
        보다 멀거나 반사 없음) free_frames 번 연속이면 지운다 — 트랙 옆 사람이 발을 옮기면 이전 발 자리가 라바콘으로 남던 것.
        시야 밖·다른 콘에 가린 것 (빔이 더 가까이에서 맞음) 은 안 지운다 (시간으로 지우면 옆을 지나는 진짜 콘이 지워짐)."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        if g('free_frames') <= 0 or not self._cones:
            return
        r = np.asarray(msg.ranges, np.float32)
        if r.size == 0 or msg.angle_increment == 0.0:
            return
        r = np.where(np.isfinite(r) & (r >= msg.range_min), r, np.inf)
        if abs(self.yaw_rate()) > g('free_max_yaw_rate'):
            return                             # 도는 중: 스캔 한 바퀴 동안 방향이 비틀려 진짜 콘 옆을 볼 수 있음
        px, py, yaw = pose
        lx0, ly0 = px + g('laser_x') * math.cos(yaw), py + g('laser_x') * math.sin(yaw)
        rmin, rmax = max(g('range_min'), msg.range_min), min(g('range_max'), msg.range_max)
        keep = []
        for k in self._cones:
            dx, dy = k['x'] - lx0, k['y'] - ly0
            d = math.hypot(dx, dy)
            b = math.atan2(dy, dx) - yaw
            b = math.atan2(math.sin(b), math.cos(b))
            if not (rmin + 0.05 < d < rmax - 0.05) or abs(b) > math.radians(g('fov_deg')):
                k['miss'] = k['miss_fin'] = 0  # 안 보이는 자리 — 판단 안 함
                keep.append(k)
                continue
            half = max(1, int(math.ceil(math.atan2(g('free_win_m'), d) / abs(msg.angle_increment))))
            i = int(round((b - msg.angle_min) / msg.angle_increment)) % r.size
            idx = [(i + j) % r.size for j in range(-half, half + 1)]
            near_r = float(r[idx].min())
            if near_r > d + g('free_clear_m'):
                # 너머에서 맞은 빔이 있으면 확실한 빈 자리, 모두 '반사 없음' 이면 약한 증거 (실제 라이다는 먼 콘 반사가 가끔 빠짐)
                fin = math.isfinite(near_r) or np.isfinite(r[idx]).any()
                k['miss'] = k.get('miss', 0) + 1
                k['miss_fin'] = k.get('miss_fin', 0) + int(fin)
                if k['miss_fin'] >= g('free_frames') or k['miss'] >= g('free_frames_inf'):
                    continue                   # 지움
            else:
                k['miss'] = k['miss_fin'] = 0
            keep.append(k)
        self._cones = keep

    def publish_markers(self, pa, now_cl):
        ma = MarkerArray()
        ma.markers.append(Marker(header=pa.header, ns='cones', action=Marker.DELETEALL))
        for i, ps in enumerate(pa.poses):
            mk = Marker(header=pa.header, ns='cones', id=i, type=Marker.CYLINDER, action=Marker.ADD)
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = ps.position.x, ps.position.y, 0.1
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = 0.09
            mk.scale.z = 0.2
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.45, 0.0, 0.8
            ma.markers.append(mk)
        pts = Marker(header=pa.header, ns='scan_clusters', id=0, type=Marker.POINTS, action=Marker.ADD)
        pts.scale.x = pts.scale.y = 0.03
        pts.color.r, pts.color.g, pts.color.b, pts.color.a = 1.0, 1.0, 0.0, 1.0
        pts.points = [Point(x=x, y=y, z=0.11) for x, y in now_cl]
        ma.markers.append(pts)
        self.mk_pub.publish(ma)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LidarProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
