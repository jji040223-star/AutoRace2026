"""2D 시뮬 실시간 보기 — 트랙 위에 차 (참 위치)·지나온 길·라이다 점·뎁스 장애물·M7 정차 LIMO·유리벽 + 카메라 영상.

  ROS_DOMAIN_ID=<시뮬 도메인> FASTRTPS_DEFAULT_PROFILES_FILE=~/AutoRace2026/tools/simcmp/fastdds_shm.xml python3 view.py
  (q 또는 Esc 로 끝. 시뮬이 공유 메모리 설정 (run.sh) 으로 돌면 보기 창도 같은 설정이어야 연결됨)

색: 초록 = 차, 하늘색 선 (굵은) = 지나온 길 (미션 상태가 바뀐 곳에 글자), 빨강 점 = 라이다, 주황 상자 = 뎁스 장애물 (그 순간),
    회색 상자 = 시뮬 속 장애물 (정답), 하늘색 선 = 유리벽.
"""
import json
import math
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from autorace_msgs.msg import ObstacleArray
from geometry_msgs.msg import Pose2D
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

S, MG = 90.0, 0.6                      # px/m, 여백 [m]
LIDAR_X = 0.103
COL = {'road_edge': (0, 200, 240), 'zigzag': (0, 200, 240), 'lane_dash': (200, 200, 200), 'stop_line': (235, 235, 235),
       'crosswalk': (235, 235, 235), 'parking_bay': (235, 235, 235), 'parking_line': (235, 235, 235), 'island': (0, 200, 240)}


def to_bgr(msg):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else a.copy()


def px(x, y):
    return int(round((x + MG) * S)), int(round((6.0 + MG - y) * S))


class View(Node):
    def __init__(self):
        super().__init__('sim_view')
        self.pose, self.scan, self.obs, self.img = None, None, None, None
        self.state, self.trail, self.marks = '', [], []
        self.create_subscription(Pose2D, '/sim2d/pose', self.on_pose, 10)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(ObstacleArray, '/perception/obstacles', lambda m: setattr(self, 'obs', m), 10)
        self.create_subscription(Image, '/camera/color/image_raw', lambda m: setattr(self, 'img', m), qos_profile_sensor_data)
        self.create_subscription(String, '/mission/state', self.on_state, 10)
        self.bg = self.background()
        self.boxes, self.walls = [], []
        self.cli = self.create_client(GetParameters, '/sim2d/get_parameters')
        self.asked = False

    def background(self):
        t = json.load(open(os.path.join(get_package_share_directory('autorace_track'), 'data', 'track_elements.json')))
        w, h = int((10 + 2 * MG) * S), int((6 + 2 * MG) * S)
        img = np.full((h, w, 3), (128, 135, 138), np.uint8)
        cv2.rectangle(img, px(0, 6), px(10, 0), (46, 52, 54), -1)
        for e in t['elements']:
            c = COL.get(e['class'])
            if c is None or not e.get('painted'):
                continue
            polys = [np.array([px(x / 1000, y / 1000) for x, y in p], np.int32) for p in [e['exterior']] + e.get('holes', [])]
            cv2.fillPoly(img, polys, c, lineType=cv2.LINE_AA)
        return img

    def ask_params(self):
        if self.asked or not self.cli.service_is_ready():
            return
        self.asked = True
        req = GetParameters.Request(names=['static_limos', 'static_layout', 'static_layout_xy', 'glass_walls', 'glass_wall_segs'])
        self.cli.call_async(req).add_done_callback(self.on_params)

    def on_params(self, fut):
        v = fut.result().values
        on, lay, xy, gw, segs = v[0].bool_value, v[1].string_value, list(v[2].double_array_value), v[3].bool_value, list(v[4].double_array_value)
        layout = [[7.85, 0.94], [7.85, 1.46], [6.95, 0.42], [6.95, 1.46], [6.05, 0.94], [5.15, 0.42], [5.15, 1.46]]
        if len(xy) >= 2:
            layout = [[xy[i], xy[i + 1]] for i in range(0, len(xy) - 1, 2)]
        elif lay == 'mirror':
            layout = [[x, 1.88 - y] for x, y in layout]
        self.boxes = (layout if on else []) + [[6.36, 4.27, 1], [7.40, 4.27, 1]]
        self.walls = [segs[i:i + 4] for i in range(0, len(segs) - 3, 4)] if gw else []

    def on_pose(self, m):
        self.pose = (m.x, m.y, m.theta)
        if not self.trail or math.hypot(m.x - self.trail[-1][0], m.y - self.trail[-1][1]) > 0.02:
            self.trail.append((m.x, m.y))

    def on_state(self, m):
        s = m.data.split()[0] if m.data else ''
        if s != self.state and self.pose:
            self.marks.append((self.pose[0], self.pose[1], s))
        self.state = s

    def draw(self):
        self.ask_params()
        img = self.bg.copy()
        for wl in self.walls:
            cv2.line(img, px(wl[0], wl[1]), px(wl[2], wl[3]), (240, 210, 140), 3)
        for b in self.boxes:
            rot = len(b) > 2                 # 주차 LIMO 는 세로
            hl, hw = (0.109, 0.161) if rot else (0.161, 0.109)
            cv2.rectangle(img, px(b[0] - hl, b[1] + hw), px(b[0] + hl, b[1] - hw), (170, 170, 170), -1)
        if len(self.trail) > 1:
            cv2.polylines(img, [np.array([px(x, y) for x, y in self.trail], np.int32)], False, (255, 255, 0), 2, cv2.LINE_AA)
        for x, y, s in self.marks[-12:]:
            cv2.circle(img, px(x, y), 3, (255, 255, 255), -1)
            cv2.putText(img, s, (px(x, y)[0] + 4, px(x, y)[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
        if self.pose:
            x, y, th = self.pose
            c, s = math.cos(th), math.sin(th)
            if self.scan is not None:
                r = np.asarray(self.scan.ranges)
                a = self.scan.angle_min + np.arange(len(r)) * self.scan.angle_increment
                ok = np.isfinite(r) & (r < 4.0)
                lx, ly = x + LIDAR_X * c, y + LIDAR_X * s
                for rr, aa in zip(r[ok], a[ok]):
                    cv2.circle(img, px(lx + rr * math.cos(th + aa), ly + rr * math.sin(th + aa)), 2, (40, 40, 255), -1)
            if self.obs is not None:
                for o in self.obs.obstacles:
                    pts = [(o.x_near, o.y_right), (o.x_far, o.y_right), (o.x_far, o.y_left), (o.x_near, o.y_left)]
                    poly = np.array([px(x + bx * c - by * s, y + bx * s + by * c) for bx, by in pts], np.int32)
                    cv2.polylines(img, [poly], True, (0, 140, 255), 2)
                    cv2.putText(img, f'{o.height:.2f}', tuple(poly[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 140, 255), 1)
            car = [(0.161, 0.109), (0.161, -0.109), (-0.161, -0.109), (-0.161, 0.109)]
            cv2.fillPoly(img, [np.array([px(x + bx * c - by * s, y + bx * s + by * c) for bx, by in car], np.int32)], (60, 200, 60))
            cv2.line(img, px(x, y), px(x + 0.3 * c, y + 0.3 * s), (255, 255, 255), 2)
        cv2.putText(img, f'state: {self.state}', (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if self.img is not None:
            cam = cv2.resize(to_bgr(self.img), (img.shape[0] * 4 // 3 // 2, img.shape[0] // 2))
            side = np.zeros((img.shape[0], cam.shape[1], 3), np.uint8)
            side[:cam.shape[0]] = cam
            img = np.hstack([img, side])
        return img


def main():
    rclpy.init()
    n = View()
    cv2.namedWindow(f'sim2d view (domain {os.environ.get("ROS_DOMAIN_ID", "0")})', cv2.WINDOW_NORMAL)
    # 창 크기·위치: 사용자가 맞춰 둔 크기 (11-08). SIMVIEW_GEOM=폭x높이+x+y 로 바꿀 수 있음
    gw, gh, gx, gy = map(int, os.environ.get('SIMVIEW_GEOM', '1302x978+36+80').replace('x', '+').split('+'))
    cv2.resizeWindow(f'sim2d view (domain {os.environ.get("ROS_DOMAIN_ID", "0")})', gw, gh)
    cv2.moveWindow(f'sim2d view (domain {os.environ.get("ROS_DOMAIN_ID", "0")})', gx, gy)
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.03)
        cv2.imshow(f'sim2d view (domain {os.environ.get("ROS_DOMAIN_ID", "0")})', n.draw())
        if cv2.waitKey(1) in (27, ord('q')):
            break
    cv2.destroyAllWindows()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
