"""신호등 판단 노드 (M1) — 몸체 박스 (YOLO traffic_light 1 클래스, 시뮬은 fake_traffic_light) → 색 + 방해 모니터 거르기.

  sub  image_topic       (기본 /camera/color/image_raw)
  sub  info_topic        (기본 /camera/color/camera_info) — fx, fy, cx
  sub  detections_topic  (기본 /perception/traffic_light_detections, autorace_msgs/DetectionArray) — label 이 color_labels (YOLO 색 클래스
                         red_light·yellow_light·green_light) 또는 light_label (몸체 1 클래스) 인 박스
  sub  lane_state_topic  (기본 /lane/state) — 정지선 거리 (예상 크기용)
  sub  odom_topic        (기본 /odom)
  pub  light_topic       (기본 /perception/traffic_light, autorace_msgs/TrafficLight) — 영상마다
  pub  debug_topic       (기본 /perception/traffic_light_debug, debug_hz) — 고른 박스 초록, 버린 박스 빨강 + 이유

색: 박스를 세로 3 칸 (위 빨강, 가운데 노랑, 아래 초록) 으로 나눠 칸 가운데 폭의 밝기 (V = RGB 최대) 상위 10 % 평균이 가장 큰 칸.
색상 (H) 이 아니라 밝기로 비교한다 — 꺼진 칸도 색은 있다.
방해 모니터 (광고·오검출 유도 그래픽) 거르기 — 박스마다:
  ① 세로:가로 aspect_min~aspect_max (실물 20.5 × 12 cm ≈ 1.7)
  ② 한 칸만 켜짐: 가장 밝은 칸 ÷ 두 번째 칸 ≥ contrast_min. 모자라면 (램프가 막 바뀌어 꺼진 칸이 아직 밝음) 최근 3 s 동안 어두웠다가
     2 배 넘게 밝아진 칸이 하나뿐일 때만 그 칸 (just_lit), 아니면 애매 → 버림
  ③ 크기: 거리 = fy × 0.205 / 박스 높이. 정지선을 BEV 에서 봤으면 (오도메트리로 이어 감) 예상 거리 = 카메라 → 정지선 + 정지선 → 신호등
     (light_beyond_m 앞, light_lateral_m 옆) 와 size_tol 안이어야 → size_checked. 정지선을 아직 못 봤으면 색을 안 냄 (require_line).
     정지선은 처음 잡은 것만 따라가고, 차 뒤로 line_keep_m 지나거나 진행 방향이 line_turn_deg 넘게 바뀌면 잊음.
  ④ 위치 고정: 크기까지 확인한 박스의 위치 (오도메트리 좌표) 를 모아 두고, 그 뒤로는 track_radius_m 넘게 떨어진 박스는 버림.
  여러 개 통과하면 예상 크기에 가장 가까운 것 (예상을 모르면 모아 둔 위치에 가장 가까운 것, 그것도 없으면 가장 큰 것).
같은 색이 일정 시간 유지되는지는 mission_manager 가 연속 판정 수로 본다.
"""
import bisect
import collections
import math
import signal

import cv2
import numpy as np
import rclpy
from autorace_msgs.msg import DetectionArray, LaneState, TrafficLight
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image

COLORS = ('red', 'yellow', 'green')


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light')
        p = self.declare_parameter
        p('image_topic', '/camera/color/image_raw')
        p('info_topic', '/camera/color/camera_info')
        p('detections_topic', '/perception/traffic_light_detections')
        p('lane_state_topic', '/lane/state')
        p('odom_topic', '/odom')
        p('light_topic', '/perception/traffic_light')
        p('debug_topic', '/perception/traffic_light_debug')
        p('debug_hz', 2.0)
        p('light_label', 'traffic_light')          # 몸체 1 클래스 (색은 세 칸 밝기로)
        p('color_labels', ['red_light', 'yellow_light', 'green_light'])   # YOLO 색 클래스 (빨강·노랑·초록 순)
        p('body_h_m', 0.205)               # 실물 몸체 높이 (규정집 사진 20.5 × 12 cm)
        p('aspect_min', 1.4)
        p('aspect_max', 2.1)
        p('contrast_min', 1.6)
        p('range_min', 0.4)
        p('range_max', 4.0)
        p('require_line', True)            # 정지선을 봐서 예상 거리가 있을 때만 색을 냄 (그 전엔 크기로 모니터를 못 거름 — 3.4 m 에서
                                           # 진짜 신호등은 세 칸 대비가 낮아 버려지고 모니터 그림만 남은 적 있음)
        p('transition_window_s', 3.0)      # 대비가 낮을 때: 이 동안 어두웠다가
        p('transition_ratio', 2.0)         #   이 배 넘게 밝아진 칸이 하나뿐이면 그 칸이 켜진 것 (전환 직후 꺼진 칸 잔상)
        p('size_tol', 0.25)                # |크기 거리 − 예상 거리| ≤ 이 비율 × 예상 거리
        p('light_beyond_m', 1.65)          # 정지선 가까운 끝 → 신호등, 진행 방향 [m] (시뮬: 정지선 x 2.28, 신호등 x 3.95)
        p('light_lateral_m', -0.37)        # 진행 방향 왼쪽 + [m] (시뮬: 차로 가운데 y 4.84, 신호등 y 4.47 → 오른쪽)
        p('line_match_m', 0.10)            # 처음 잡은 정지선과 이만큼 넘게 다른 측정은 다른 선
        p('line_keep_m', 1.0)              # 정지선이 차 뒤로 이만큼 지나가면 잊음
        p('line_turn_deg', 45.0)           # 정지선을 본 뒤 진행 방향이 이만큼 바뀌면 잊음 (다른 도로)
        p('cam_x', 0.084)                  # base_link → 카메라 [m]
        p('track_radius_m', 0.35)
        p('track_min', 3)                  # 크기까지 확인한 박스가 이만큼 모이면 위치 검사 시작
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.K = None
        self._dets = collections.deque(maxlen=30)     # (시각, DetectionArray)
        self._odom = collections.deque(maxlen=300)    # (시각, x, y, yaw)
        self._line = None                  # 정지선 가까운 끝 (odom x, y) 과 그때 방향
        self._track = []                   # 크기까지 확인한 신호등 위치 (odom)
        self._cell_hist = collections.deque(maxlen=300)   # (시각, 박스, 세 칸 밝기) — 램프 전환 감지용
        self._last_dbg = 0.0
        self._last_log = 0.0
        self.pub = self.create_publisher(TrafficLight, g('light_topic'), 10)
        self.dbg_pub = self.create_publisher(Image, g('debug_topic'), 2) if g('debug_hz') > 0 else None
        self.create_subscription(CameraInfo, g('info_topic'), self.on_info, qos_profile_sensor_data)
        self.create_subscription(DetectionArray, g('detections_topic'), lambda m: self._dets.append((stamp_s(m.header), m)), 10)
        self.create_subscription(LaneState, g('lane_state_topic'), self.on_lane, 20)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 50)
        self.create_subscription(Image, g('image_topic'), self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f"traffic_light: {g('detections_topic')} ({g('light_label')}, {'/'.join(g('color_labels'))}) + 영상 → {g('light_topic')}")

    def on_info(self, msg):
        if self.K is None and msg.k[0] > 0:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self._odom and math.dist(self._odom[-1][1:3], (p.x, p.y)) > 0.5:
            self._odom.clear()             # 리셋
            self._line, self._track = None, []
        self._odom.append((stamp_s(msg.header), p.x, p.y, yaw))

    def pose_at(self, t):
        h = self._odom
        if not h:
            return None
        i = bisect.bisect_left([e[0] for e in h], t)
        e = min((h[j] for j in (i - 1, i) if 0 <= j < len(h)), key=lambda e: abs(e[0] - t))
        return e[1:]

    def on_lane(self, msg):
        """정지선 (카메라 → 가까운 끝) 을 odom 좌표로 — 처음 잡은 선만 따라감 (M3 와 같은 방식)."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        pose = self.pose_at(stamp_s(msg.header))
        if pose is None:
            return
        x, y, yaw = pose
        if self._line is not None:
            lx, ly, lyaw = self._line
            dyaw = abs((yaw - lyaw + math.pi) % (2 * math.pi) - math.pi)
            if ((lx - x) * math.cos(yaw) + (ly - y) * math.sin(yaw) < -g('line_keep_m')
                    or dyaw > math.radians(g('line_turn_deg'))):
                self._line = None          # 지나간 선, 또는 방향이 바뀐 (다른 도로의) 선 — 라바콘 직선 끝 정지선을 2차로까지 들고 왔었다
        if msg.stop_line_dist_m <= 0:
            return
        d = g('cam_x') + msg.stop_line_dist_m
        pt = (x + d * math.cos(yaw), y + d * math.sin(yaw), yaw)
        if self._line is None or math.dist(pt[:2], self._line[:2]) < g('line_match_m'):
            self._line = pt

    def expected_dist(self, pose):
        """카메라 → 신호등 예상 거리 (정지선을 봤을 때만), 아니면 None."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        if self._line is None or pose is None:
            return None
        x, y, yaw = pose
        lx, ly, lyaw = self._line
        cx, cy = x + g('cam_x') * math.cos(yaw), y + g('cam_x') * math.sin(yaw)
        along = (lx - cx) * math.cos(lyaw) + (ly - cy) * math.sin(lyaw) + g('light_beyond_m')
        lat = -(lx - cx) * math.sin(lyaw) + (ly - cy) * math.cos(lyaw) + g('light_lateral_m')
        return math.hypot(along, lat) if along > 0.1 else None

    def cells(self, bgr, d):
        """세 칸 밝기 (위·가운데·아래)."""
        x1, y1, x2, y2 = max(0, d.x1), max(0, d.y1), min(bgr.shape[1], d.x2), min(bgr.shape[0], d.y2)
        if x2 - x1 < 3 or y2 - y1 < 6:
            return None
        v = bgr[y1:y2, x1:x2].max(axis=2).astype(np.float32)
        h, w = v.shape
        mid = v[:, int(w * 0.25):max(int(w * 0.25) + 1, int(w * 0.75))]
        out = []
        for i in range(3):
            c = mid[int(h * i / 3):max(int(h * i / 3) + 1, int(h * (i + 1) / 3))].ravel()
            k = max(1, len(c) // 10)
            out.append(float(np.sort(c)[-k:].mean()))
        return out

    def just_lit(self, t, d, cl):
        """두 칸이 같이 밝아 애매할 때 (램프가 바뀐 직후 — 시뮬 렌더러는 멈춘 카메라에서 꺼진 램프 빛을 몇~십몇 초 남긴다):
        같은 박스 (IoU ≥ 0.5) 의 transition_window_s 동안 기록에서 가장 어두웠던 값보다 transition_ratio 배 넘게 밝아진 칸이
        하나뿐이고 그 칸이 가장 밝은 칸과 거의 같으면 (90 %) 그 칸이 켜진 것. → (칸, 밝아진 배율) 또는 None."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        w0, w1 = t - g('transition_window_s'), t - 0.3
        box = (d.x1, d.y1, d.x2, d.y2)

        def iou(a, b):
            ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
            iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
            u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
            return ix * iy / max(u, 1)
        past = [c for tt, b, c in self._cell_hist if w0 <= tt <= w1 and iou(b, box) >= 0.5]
        if len(past) < 5:
            return None
        lo = [min(c[i] for c in past) for i in range(3)]
        top = max(cl)
        jumped = [i for i in range(3) if cl[i] >= g('transition_ratio') * max(lo[i], 1.0) and cl[i] >= 0.9 * top]
        if len(jumped) != 1:
            return None
        i = jumped[0]
        return i, cl[i] / max(lo[i], 1.0)

    def on_image(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        if self.K is None or msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
            return
        t = stamp_s(msg.header)
        dets = min(self._dets, key=lambda e: abs(e[0] - t), default=None)
        labels = {g('light_label'): None, **{lb: i for i, lb in enumerate(g('color_labels'))}}
        boxes = [] if dets is None or abs(dets[0] - t) > 0.15 else [d for d in dets[1].detections if d.label in labels]
        ch = 4 if msg.encoding.endswith('a8') else 3
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // ch, ch)[:, :msg.width, :3]
        bgr = img[:, :, ::-1] if msg.encoding.startswith('rgb') else img
        fx, fy, cx, _ = self.K
        pose = self.pose_at(t)
        d_exp = self.expected_dist(pose)
        ok, bad = [], []
        for d in boxes:
            bw, bh = d.x2 - d.x1, d.y2 - d.y1
            cl = self.cells(bgr, d)
            if cl is None:
                bad.append((d, '작음'))
                continue
            srt = sorted(cl)
            contrast = srt[2] / max(srt[1], 1.0)
            dist = fy * g('body_h_m') / max(bh, 1)
            aspect = bh / max(bw, 1)
            if not g('aspect_min') <= aspect <= g('aspect_max'):
                bad.append((d, f'비 {aspect:.2f}'))
                continue
            if d_exp is not None and abs(dist - d_exp) > g('size_tol') * d_exp:
                bad.append((d, f'크기 거리 {dist:.2f} ≠ 예상 {d_exp:.2f}'))
                continue
            if not g('range_min') <= dist <= g('range_max'):
                bad.append((d, f'거리 {dist:.2f}'))
                continue
            self._cell_hist.append((t, (d.x1, d.y1, d.x2, d.y2), cl))
            lit = cl.index(srt[2])
            yci = labels[d.label]
            if yci is not None:
                # YOLO 색 클래스: 세 칸 밝기가 뚜렷하게 (대비 ≥ contrast_min) 다른 칸이면 버림, 애매하면 (램프 전환 잔상) YOLO 를 따름
                if contrast >= g('contrast_min') and lit != yci:
                    bad.append((d, f'YOLO {COLORS[yci]} ≠ 밝은 칸 {COLORS[lit]}'))
                    continue
                if contrast < g('contrast_min'):
                    # 잔상: 램프가 막 바뀌면 YOLO 가 같은 자리에 옛 색·새 색 박스를 둘 다 낸다 → 막 밝아진 칸과 다른 색 박스는 버림
                    jumped = self.just_lit(t, d, cl)
                    if jumped is not None and jumped[0] != yci:
                        bad.append((d, f'YOLO {COLORS[yci]} 잔상 (막 켜진 칸 {COLORS[jumped[0]]})'))
                        continue
                lit = yci
            elif contrast < g('contrast_min'):
                jumped = self.just_lit(t, d, cl)
                if jumped is None:
                    bad.append((d, f'켜진 칸 대비 {contrast:.2f}'))
                    continue
                lit, contrast = jumped
            wp = None
            if pose is not None:
                br = math.atan2(cx - (d.x1 + d.x2) / 2, fx)
                x, y, yaw = pose
                ax, ay = g('cam_x') + dist * math.cos(br), dist * math.sin(br)
                wp = (x + ax * math.cos(yaw) - ay * math.sin(yaw), y + ax * math.sin(yaw) + ay * math.cos(yaw))
            if wp is not None and len(self._track) >= g('track_min'):
                tx = float(np.median([p[0] for p in self._track]))
                ty = float(np.median([p[1] for p in self._track]))
                if math.dist(wp, (tx, ty)) > g('track_radius_m'):
                    bad.append((d, f'위치 {math.dist(wp, (tx, ty)):.2f} m 벗어남'))
                    continue
            ok.append((d, lit, contrast, dist, wp))
        # 같은 자리 (IoU ≥ 0.5) 에 색이 다른 박스가 둘 다 남으면 신뢰도가 높은 것만 (YOLO 가 한 신호등에 두 클래스를 낼 때)
        def iou(a, b):
            ix = max(0, min(a.x2, b.x2) - max(a.x1, b.x1))
            iy = max(0, min(a.y2, b.y2) - max(a.y1, b.y1))
            u = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - ix * iy
            return ix * iy / max(u, 1)
        ok = [o for o in ok if not any(p is not o and p[1] != o[1] and iou(p[0], o[0]) >= 0.5 and p[0].confidence > o[0].confidence
                                       for p in ok)]
        out = TrafficLight(header=msg.header, color='none', expected_dist_m=float(d_exp if d_exp is not None else -1.0),
                           candidates=len(boxes), rejected=len(bad))
        if ok and d_exp is None and g('require_line'):
            out.reason = (f'정지선 아직 안 봄 — 후보 {len(ok)} 개 (크기 확인 불가)' + ('; ' if bad else '') + '; '.join(r for _, r in bad))[:200]
        elif ok:
            if d_exp is not None:
                best = min(ok, key=lambda o: abs(o[3] - d_exp))
            elif len(self._track) >= g('track_min') and all(o[4] is not None for o in ok):
                tx = float(np.median([p[0] for p in self._track]))
                ty = float(np.median([p[1] for p in self._track]))
                best = min(ok, key=lambda o: math.dist(o[4], (tx, ty)))
            else:
                best = max(ok, key=lambda o: o[0].y2 - o[0].y1)
            d, ci, contrast, dist, wp = best
            out.color, out.contrast, out.dist_m = COLORS[ci], float(contrast), float(dist)
            out.x1, out.y1, out.x2, out.y2 = d.x1, d.y1, d.x2, d.y2
            out.size_checked = d_exp is not None
            if out.size_checked and wp is not None:
                self._track = (self._track + [wp])[-30:]
            out.reason = '; '.join(r for _, r in bad)[:200]
        else:
            out.reason = ('박스 없음' if not boxes else '모두 버림: ' + '; '.join(r for _, r in bad))[:200]
        self.pub.publish(out)
        now = self.get_clock().now().nanoseconds * 1e-9
        if bad and now - self._last_log >= 2.0:
            self._last_log = now
            self.get_logger().info(f'신호등 {out.color} (박스 {len(boxes)} 개, 버림 {len(bad)}: {out.reason[:120]})')
        if self.dbg_pub is not None and now - self._last_dbg >= 1.0 / g('debug_hz'):
            self._last_dbg = now
            dbg = np.ascontiguousarray(bgr).copy()
            for d, why in bad:
                cv2.rectangle(dbg, (d.x1, d.y1), (d.x2, d.y2), (0, 0, 255), 1)
                cv2.putText(dbg, why[:24], (d.x1, max(10, d.y1 - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
            if out.color != 'none':
                d = best[0]
                cv2.rectangle(dbg, (d.x1, d.y1), (d.x2, d.y2), (0, 255, 0), 2)
                cv2.putText(dbg, f'{out.color} {out.contrast:.1f} {out.dist_m:.2f}m{" ok" if out.size_checked else ""}',
                            (d.x1, max(10, d.y1 - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            self.dbg_pub.publish(Image(header=msg.header, height=dbg.shape[0], width=dbg.shape[1], encoding='bgr8',
                                       step=dbg.shape[1] * 3, data=dbg.tobytes()))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
