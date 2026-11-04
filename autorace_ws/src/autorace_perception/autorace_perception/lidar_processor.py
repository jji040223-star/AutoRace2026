"""라이다 라바콘 노드 (v3 M8 — 라이다 주, 카메라 보조).

  sub  scan_topic   (기본 /scan, sensor_msgs/LaserScan, laser_link — base_link 와 같은 방향, 앞 laser_x)
  sub  odom_topic   (기본 /odom)
  pub  cones_topic  (기본 /perception/cones, geometry_msgs/PoseArray, odom 좌표) — 기억한 라바콘 중심 (min_hits 번 이상 본 것)
  pub  marker_topic (기본 /perception/cone_markers, visualization_msgs/MarkerArray) — RViz: 기억 (주황 원기둥), 이번 스캔 묶음 (노랑 점)

처리: 앞 ±fov_deg, range_min~range_max 의 점 → 각도 순으로 이웃 점 거리 cluster_gap_m 안이면 같은 묶음 → 점 min_points 이상,
크기 (양 끝 거리) max_size_m 이하만 라바콘 (정차 LIMO 0.22 m·벽은 버림). 중심 = 점 평균을 라바콘 반지름만큼 안쪽으로.
라이다 높이 (시뮬 0.111 m) 에서 라바콘 단면은 지름 약 4.3 cm, 0.96° 간격이면 0.5 m 앞 5 점, 1.5 m 앞 1~2 점.
기억: odom 좌표로 merge_m 안이면 같은 라바콘 (평균). 시간으로 지우지 않고, 차 진행 방향으로 pass_margin_m 넘게 뒤로 지나갔거나
max_keep_m 넘게 멀어지면 지운다. 리셋 (odom 이 0.5 m 넘게 튐) 이면 다 지운다.
"""
import bisect
import collections
import math
import signal

import numpy as np
import rclpy
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
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self._odom = collections.deque(maxlen=200)    # (시각, x, y, yaw)
        self._pose = None
        self._cones = []                   # {'x', 'y', 'n', 'hits'}
        self.pub = self.create_publisher(PoseArray, g('cones_topic'), 10)
        self.mk_pub = self.create_publisher(MarkerArray, g('marker_topic'), 2)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 50)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data)
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
        # 지나간 것·멀어진 것만 지움 (시간으로 안 지움)
        if self._pose is not None:
            qx, qy, qyaw = self._pose
            hc, hs = math.cos(qyaw), math.sin(qyaw)
            self._cones = [k for k in self._cones
                           if (k['x'] - qx) * hc + (k['y'] - qy) * hs > -g('pass_margin_m')
                           and math.dist((k['x'], k['y']), (qx, qy)) < g('max_keep_m')]
        keep = [k for k in self._cones if k['n'] >= g('min_hits')]
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
