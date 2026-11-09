"""가벼운 2D 시뮬 (Isaac Sim 없이) — 트랙 도면을 위에서 본 그림으로 깔고, 카메라 영상은 바닥 평면 원근 변환 + 물체 광선 추적,
라이다·뎁스는 같은 물체 목록으로 계산, 차는 /cmd_vel 기구학 (1 차 지연) 으로 움직인다. Isaac 시뮬과 같은 토픽·프레임·리셋 서비스를 낸다.

  sub  /cmd_vel (Twist)
  pub  /camera/color/image_raw (rgb8), /camera/color/camera_info, /camera/depth/image_raw (16UC1 mm), /camera/depth/camera_info,
       /scan (LaserScan, laser_link), /odom (odom → base_link, 출발 자세 기준), /imu, /tf, /tf_static, /clock
       /sim2d/pose (geometry_msgs/Pose2D, world 참값), /sim2d/status (std_msgs/String JSON, 1 Hz — 선까지 최소 거리 (전체·1 m 칸별)·충돌)
  srv  /sim/reset (std_srvs/Trigger), sub /sim/start_pose (Pose2D, 다음 리셋 한 번만) — reset_sim 과 같은 방식

실차 오차 흉내 (기본은 모두 0 = 이상적인 차):
  cam_pitch_deg / cam_yaw_deg / cam_roll_deg / cam_height — 실제 카메라 자세 (인식 노드 설정과 다르게 주면 '설정이 실차와 다를 때' 를 재현)
  odom_w_scale  — 오도메트리 회전 배율 (스키드 미끄러짐: 1.2 = 실제 90° 돌 때 odom 은 108°)
  v_deadband / w_deadband — 이보다 작은 명령은 안 움직임 (모터 데드존)
  scan_flip     — 라이다 좌우 반전
  depth_dropout — 검은 LIMO 표면 뎁스 구멍 비율 (0~1)
"""
import importlib.util
import json
import math
import os
import random
import signal

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from autorace_msgs.msg import Detection, DetectionArray
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose2D, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

CAR_HL, CAR_HW = 0.161, 0.109          # avoid.CAR_HALF_L / W 와 같은 차 사각형 (base_link 가운데)
WHEELS = [(0.1, 0.0875), (0.1, -0.0875), (-0.1, 0.0875), (-0.1, -0.0875)]
BASE_H = 0.145                         # 바닥 → base_link
LIDAR_Z = 0.111                        # 바닥 → 라이다

COLORS = {'mat': (0x36, 0x34, 0x2e), 'venue': (0x8a, 0x87, 0x80), 'white': (0xdf, 0xda, 0xca),
          'yellow': (0xe5, 0xaa, 0x43), 'black': (0x11, 0x11, 0x11), 'wall': (0xb8, 0xb6, 0xb0)}
CLASS_COLOR = {'road_edge': 'yellow', 'zigzag': 'yellow', 'lane_dash': 'white', 'stop_line': 'white', 'crosswalk': 'white',
               'slow_diamond': 'white', 'slow_text': 'white', 'branch_letter': 'white', 'parking_line': 'white',
               'start_white': 'white', 'start_black': 'black'}


def stamp(t):
    s = int(t)
    return Time(sec=s, nanosec=int((t - s) * 1e9))


def quat_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Box:
    """바닥에 선 직육면체 (world): 가운데 (x, y), 방향 yaw, 반 길이·반 폭, z0~z1, 색 (RGB), kind."""

    def __init__(self, kind, x, y, yaw, hl, hw, z0, z1, color, name=''):
        self.kind, self.x, self.y, self.yaw = kind, x, y, yaw
        self.hl, self.hw, self.z0, self.z1 = hl, hw, z0, z1
        self.color = np.array(color, np.float32)
        self.name = name
        self.alive = True

    def corners2d(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [(self.x + sx * self.hl * c - sy * self.hw * s, self.y + sx * self.hl * s + sy * self.hw * c)
                for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def rects_overlap(a, b):
    """두 볼록 사각형 (꼭짓점 4 개씩) 이 겹치는지 (분리축)."""
    for poly in (a, b):
        for i in range(4):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % 4]
            nx, ny = y2 - y1, x1 - x2
            pa = [nx * x + ny * y for x, y in a]
            pb = [nx * x + ny * y for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


class Sim2D(Node):
    def __init__(self):
        super().__init__('sim2d')
        p = self.declare_parameter
        p('start_x', 0.74)
        p('start_y', 5.58)
        p('start_yaw_deg', 0.0)
        p('physics_hz', 100.0)
        p('camera_hz', 20.0)
        p('depth_hz', 10.0)
        p('scan_hz', 8.0)
        p('odom_hz', 50.0)
        p('imu_hz', 100.0)
        p('pose_hz', 20.0)                 # /sim2d/pose (참 위치) — 평가 기록용 (예전 1 Hz 는 회전·근접 순간을 못 봄)
        p('raster_res', 0.0025)            # 위에서 본 트랙 그림 [m/px]
        # 카메라 (실제 자세 — 인식 노드 설정과 다르게 주면 실차 설정 오차 재현)
        p('width', 640)
        p('height', 480)
        p('hfov_deg', 71.0)
        p('cam_x', 0.084)
        p('odom_yaw0_deg', 0.0)            # 출발 때 오도메트리 yaw (실차는 센서를 켠 뒤 차를 옮기면 트랙과 틀어짐)
        p('wheel_r', 0.045)                # 바퀴 반지름 — 정지선 앞 정지 판정 (바퀴 맨 앞 = 앞바퀴 축 + 반지름)
        p('cam_y', 0.0)
        p('cam_height', 0.175)
        p('cam_pitch_deg', 0.0)            # 아래로 +
        p('cam_yaw_deg', 0.0)              # 왼쪽으로 +
        p('cam_roll_deg', 0.0)
        p('image_noise', 3.0)              # 화소 잡음 (표준편차, 0~255)
        p('brightness', 1.0)
        p('depth_min', 0.3)
        p('depth_max', 3.0)
        p('depth_dropout', 0.0)            # 검은 LIMO 면 뎁스 구멍 비율
        # 차
        p('tau_v', 0.15)
        p('tau_w', 0.10)
        p('max_v', 1.0)
        p('max_w', 2.5)
        p('v_deadband', 0.0)
        p('w_deadband', 0.0)
        p('cmd_timeout', 0.5)
        # 아커만 (LIMO 앞바퀴 조향 모드): limo_base 처럼 r = v/w 를 최소 회전 반경으로 자르고 조향각으로. v = 0 이면 바퀴만 꺾이고 안 돎
        p('drive_mode', 'diff')            # diff (네 바퀴 차동, 제자리 회전) | ackermann
        p('wheelbase', 0.2)                # LIMO 축간거리 [m]
        p('ack_min_radius', 0.47)          # 최소 회전 반경 [m] (안쪽 바퀴 최대 28°, 윤거 0.172 → 차 중심 반경 ≈ 0.46~0.48)
        p('steer_tau', 0.08)               # 조향 서보 지연 [s]
        # 기준점: rear = 뒷축 가운데가 v 로 가고 뒷바퀴는 옆으로 안 미끄러짐 (실제 앞바퀴 조향 차. base_link = 뒷축 + L/2 앞이라
        # 몸이 궤적보다 안쪽을 봄 — 최대 조향에서 ~12°). center = 예전 모델 (base_link 가 자전거 기준점, 미끄럼각 0) — 비교용.
        # limo_base 의 r = v / w 도 뒷축 기준으로 봄 (agilex limo_driver 의 안쪽 바퀴 각 atan(L / (r − 윤거/2)) — 실차에서 확인할 것)
        p('ack_ref', 'rear')
        p('ack_steer_scale', 1.0)          # 실제 조향각 = 명령 조향각 × 이것 (서보·링크 오차로 덜 꺾임 < 1 — '실차가 덜 도는' 상황 재현).
                                           # 최대 조향도 같이 줄어 실제 최소 반경 = L / tan(scale · atan(L / ack_min_radius))
        p('odom_v_scale', 1.0)
        p('odom_w_scale', 1.0)
        p('imu_noise', 0.002)
        p('imu_bias', 0.0)                 # [rad/s] 자이로 치우침 (멈춰 있어도 도는 것처럼) — 가혹 조건 시험
        p('imu_w_scale', 1.0)              # 자이로 배율 오차
        p('act_v_gain', 1.0)               # 실제 속도 = 명령 × 이것 (배터리 약함 등)
        p('act_w_gain', 1.0)               # 실제 회전 = 명령 × 이것 (스키드 미끄러짐)
        # 라이다
        p('laser_x', 0.103)
        p('scan_points', 375)
        p('scan_min', 0.12)
        p('scan_max', 8.0)
        p('scan_noise', 0.005)
        p('scan_flip', False)
        # 시설물
        p('static_limos', True)
        p('static_layout', 'default')       # default (첫 줄 차로 0 으로 진입) | mirror (차로 2 로 진입)
        p('static_layout_xy', [0.0])        # 직접 배치 [x1, y1, x2, y2, ...] (yaw 180) — 길이 2 이상이면 static_layout 대신 (랜덤 간격 시험)
        p('parking_limos', True)
        p('cones', True)
        p('cone_layout', 'default')
        p('traffic_light', True)
        p('light_s', [10.0, 10.0, 3.0])    # 빨강, 초록, 노랑 [s]
        p('light_mode', 'cycle')           # cycle = light_s 대로 돌기 | wait_car = 차가 정지선 앞에 완전히 설 때까지 빨강, 선 뒤 초록 (계속)
        p('light_stop_box', [2.70, 4.59, 3.25, 5.09])  # wait_car: 차 가운데가 이 사각형 [x0, y0, x1, y1] 안 = 신호등 앞 횡단보도 정지선 (x 3.20) 앞 (x 2.30 서행 구간 정지선은 아님)
        p('light_green_after_s', 4.0)      # wait_car: 그 안에서 멈춘 뒤 이만큼 지나면 초록
        p('branch_sign', 'random')         # A | B | random (리셋마다)
        p('seed', 0)
        p('fake_detections', True)         # YOLO 대신: 보이는 표지판·신호등 박스를 /perception/detections, /perception/traffic_light_detections 로
        p('sign_range_m', 4.5)
        # 실차 모드 (실차 run1~5, 11-05 에서 본 것). 기본값은 모두 꺼짐 = 예전 시뮬과 같음
        p('det_miss', 0.0)                 # 보이는 표지판을 그 장에서 놓칠 확률 (실차: 띄엄띄엄 잡혀 '연속 3 장' 을 못 채움, run5)
        p('det_miss_stick', 0.0)           # 앞 장에서 놓쳤으면 이번 장도 놓칠 확률 (놓침이 몰려서 옴)
        p('det_conf', [0.9, 0.9])          # 확신도 범위 (실차 0.51~0.91 — 멀면 낮게)
        p('det_false_rate', 0.0)           # 장마다 가짜 표지판 박스가 나올 확률 (실차 run4: 3차선에서 화면 가득 478px 과속방지턱)
        p('det_false_labels', ['speed_bump', 'slow', 'crosswalk', 'parking'])
        p('det_false_box_px', [30, 480])   # 가짜 박스 높이 범위
        p('det_false_frames', [1, 5])      # 가짜 박스가 이어지는 장 수 (실차 run4: 몇 장 연속)
        p('cam_stall_rate', 0.0)           # 장마다 영상이 끊기기 시작할 확률 (실차 run2: 차선 입력 끊김)
        p('cam_stall_s', [0.2, 0.6])       # 끊기는 시간 범위 [s] — 그동안 영상·뎁스·검출 안 나감
        p('sign_posts', False)             # 표지판 지주를 라이다에 보이게 (실차 run2: 받침대 점 3 개 → 주차 오작동)
        p('bystanders', [0.0])             # 트랙 옆 사람 다리 [x, y, x, y, …] (각 점 = 다리 하나, 지름 bystander_d). [0.0] = 없음
        p('bystander_d', 0.09)
        p('bystander_step', [0.0, 0.0])    # 사람이 발을 옮김: bystander_step_s 마다 다리가 이만큼 [dx, dy] 갔다 왔다 (0 = 가만히)
        p('bystander_step_s', 2.0)
        # 실차 (11-06 연습장): 트랙 밖 유리벽·유리문 — 뎁스엔 장애물로 잡힘 (M3 뒤 코너 정면, 3차선 왼쪽 옆)
        p('glass_walls', False)
        p('glass_wall_segs', [3.5, -0.40, 10.3, -0.40])   # 벽 선분 [x1, y1, x2, y2, ...] (트랙 판 밖). 실차: 코너 정면 1.6–2.5 m (M7 켜는 1.2 m 보다 멂), 3차선 왼쪽 0.75–1.6 m
        p('glass_wall_h', 2.0)
        p('glass_lidar', False)            # 라이다에도 보이나 (유리는 레이저가 대부분 통과 — 실차 확인 안 됨)
        # 실차: 현수막 위에서 빠른 제자리 회전이면 바퀴가 헛돎 (휠 odom 132° / IMU ≈ 0°, 명령 w 1.5)
        p('pivot_slip_w', 1.2)             # 이 회전 [rad/s] 이상 + 거의 멈춘 채면 헛돎 후보
        p('pivot_slip_v', 0.05)
        p('pivot_slip_ratio', 1.0)         # 헛돌 때 몸체 회전 = 바퀴 회전 × 이것 (1.0 = 끔)
        p('pivot_slip_prob', 1.0)          # 제자리 회전 한 번마다 헛돌 확률
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.g = g
        self.rng = random.Random(g('seed') or None)
        self.np_rng = np.random.default_rng(g('seed') or None)

        share = get_package_share_directory('autorace_sim')
        self.track = json.load(open(os.path.join(get_package_share_directory('autorace_track'),
                                                 'data', 'track_elements.json')))
        spec = importlib.util.spec_from_file_location('cone_layouts', os.path.join(share, 'isaac', 'cone_layouts.py'))
        self.cone_layouts = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.cone_layouts)
        self.build_raster()
        # 정지선: stop_line + 횡단보도 앞뒤 가로줄 (crosswalk 중 차로를 가로지르는 얇고 긴 것 — M3 는 이 줄 앞에 선다)
        def crossing_bar(e):
            b = e['bbox']
            w, h = (b[2] - b[0]) / 1000.0, (b[3] - b[1]) / 1000.0
            return min(w, h) <= 0.06 and max(w, h) >= 0.3
        self.stop_lines = [np.asarray(e['exterior'], np.float64) / 1000.0 for e in self.track['elements']
                           if e['class'] == 'stop_line' or (e['class'] == 'crosswalk' and crossing_bar(e))]
        self.build_camera()

        self.sim_t = 0.0
        self.next_start = None
        self.cmd = (0.0, 0.0)
        self.cmd_t = -1e9
        self.stats_reset()
        self.reset_world()

        sd = qos_profile_sensor_data
        self.pub_img = self.create_publisher(Image, '/camera/color/image_raw', sd)
        self.pub_cinfo = self.create_publisher(CameraInfo, '/camera/color/camera_info', sd)
        self.pub_depth = self.create_publisher(Image, '/camera/depth/image_raw', sd)
        self.pub_dinfo = self.create_publisher(CameraInfo, '/camera/depth/camera_info', sd)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', sd)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 50)
        self.pub_imu = self.create_publisher(Imu, '/imu', sd)
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_pose = self.create_publisher(Pose2D, '/sim2d/pose', 10)
        self.pub_det = self.create_publisher(DetectionArray, '/perception/detections', 10)
        self.pub_light_det = self.create_publisher(DetectionArray, '/perception/traffic_light_detections', 10)
        self.pub_status = self.create_publisher(String, '/sim2d/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_static_tf()
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Pose2D, '/sim/start_pose', self.on_start_pose, 10)
        self.create_service(Trigger, '/sim/reset', self.on_reset)

        self.wall_last = None
        self.create_timer(1.0 / g('physics_hz'), self.step)
        self.due = {'cam': 0.0, 'depth': 0.0, 'scan': 0.0, 'odom': 0.0, 'imu': 0.0, 'status': 0.0, 'pose': 0.0}
        self.get_logger().info(
            f"sim2d: 트랙 {self.raster.shape[1]}×{self.raster.shape[0]} px ({g('raster_res') * 1000:.1f} mm/px), "
            f"시설물 {sum(b.alive for b in self.boxes)} 개, 카메라 높이 {g('cam_height')} m 기울기 {g('cam_pitch_deg')}° "
            f"yaw {g('cam_yaw_deg')}° roll {g('cam_roll_deg')}°, odom 회전 배율 {g('odom_w_scale')}, 데드존 v {g('v_deadband')} w {g('w_deadband')}")

    # ------------------------------------------------------------------ 세계
    def build_raster(self):
        res = self.g('raster_res')
        self.margin = 1.0
        W, H = 10.0 + 2 * self.margin, 6.0 + 2 * self.margin
        w, h = int(round(W / res)), int(round(H / res))
        img = np.zeros((h, w, 3), np.uint8)
        img[:] = COLORS['venue']
        self.x0, self.ytop, self.res = -self.margin, 6.0 + self.margin, res

        def px(pts_mm):
            a = np.asarray(pts_mm, np.float64) / 1000.0
            return np.stack([(a[:, 0] - self.x0) / res, (self.ytop - a[:, 1]) / res], 1).round().astype(np.int32)

        cv2.fillPoly(img, [px([[0, 0], [10000, 0], [10000, 6000], [0, 6000]])], COLORS['mat'], lineType=cv2.LINE_AA)
        yellow = np.zeros((h, w), np.uint8)
        for e in self.track['elements']:
            col = CLASS_COLOR.get(e['class'])
            if not e.get('painted') or col is None:
                continue
            polys = [px(e['exterior'])] + [px(hl) for hl in e.get('holes', [])]
            cv2.fillPoly(img, polys, COLORS[col], lineType=cv2.LINE_AA)
            if col == 'yellow':
                cv2.fillPoly(yellow, polys, 255)
        self.raster = img
        # 노란 선까지 거리 [m] (바퀴가 선을 밟는지 판정)
        self.yellow_dist = cv2.distanceTransform(255 - yellow, cv2.DIST_L2, 5).astype(np.float32) * res
        # 선 밖 (트랙 데이터 off_road·island = 노란 선 바깥) — 규정 차선 이탈 판정 (한쪽 두 바퀴가 선 밖) 용
        off = np.zeros((h, w), np.uint8)
        for e in self.track['elements']:
            if e['class'] in ('off_road', 'island'):
                cv2.fillPoly(off, [px(e['exterior'])] + [px(hl) for hl in e.get('holes', [])], 255)
        self.off_mask = off > 0

    def world_px(self, x, y):
        return (x - self.x0) / self.res, (self.ytop - y) / self.res

    def reset_world(self):
        g = self.g
        if self.next_start is not None:
            self.start = self.next_start
            self.next_start = None
        else:
            self.start = (g('start_x'), g('start_y'), math.radians(g('start_yaw_deg')))
        self.x, self.y, self.yaw = self.start
        self.v = self.w = 0.0
        self.steer = 0.0
        self.ox = self.oy = 0.0                     # 오도메트리 (출발 자세 기준, 오차 포함)
        self.oyaw = math.radians(self.g('odom_yaw0_deg'))   # 실차처럼 오도메트리 0° 가 트랙과 틀어져 있을 때 흉내
        self.imu_yaw = 0.0
        self.light_t0 = self.sim_t
        self._light_green, self._light_wait = False, None
        self.branch = self.rng.choice('AB') if g('branch_sign') == 'random' else str(g('branch_sign')).upper()
        self.boxes = []
        limo = dict(hl=0.161, hw=0.109, z0=0.0, z1=0.252, color=(0x22, 0x22, 0x24))
        if g('static_limos'):
            layout = [[7.85, 0.94, 180], [7.85, 1.46, 180], [6.95, 0.42, 180], [6.95, 1.46, 180],
                      [6.05, 0.94, 180], [5.15, 0.42, 180], [5.15, 1.46, 180]]     # default: 첫 줄은 차로 0 (왼쪽) 만 빔
            xy = list(g('static_layout_xy'))
            if len(xy) >= 2:
                layout = [[xy[i], xy[i + 1], 180] for i in range(0, len(xy) - 1, 2)]
            elif g('static_layout') == 'mirror':
                layout = [[x, 1.88 - y, yaw] for x, y, yaw in layout]         # 가운데 차로 기준 좌우 뒤집음: 첫 줄은 차로 2 (오른쪽) 만 빔
            for i, (x, y, yaw) in enumerate(layout):
                self.boxes.append(Box('limo', x, y, math.radians(yaw), name=f'M7_LIMO_{i}', **limo))
        if g('parking_limos'):
            for i, (x, y, yaw) in enumerate([[6.36, 4.27, -90], [7.40, 4.27, -90]]):
                self.boxes.append(Box('limo', x, y, math.radians(yaw), name=f'M4_LIMO_{i}', **limo))
        if g('cones'):
            base = dict(start=[0.72, 2.05], heading=90, pairs=7, pitch=0.30, half_gap=0.25, sway=0.12, sway_period=1.8)
            layout = g('cone_layout')
            if layout == 'random':
                layout = self.rng.choice(self.cone_layouts.NAMES)
            for i, (x, y) in enumerate(self.cone_layouts.positions(layout, base, self.rng)):
                # 몸통 (라이다 높이 0.111 m 에서 지름 약 4.3 cm) + 바닥판
                self.boxes.append(Box('cone', x, y, 0.0, 0.0215, 0.0215, 0.012, 0.19, (0xff, 0x6a, 0x13), name=f'cone_{i}'))
                self.boxes.append(Box('cone_base', x, y, 0.0, 0.055, 0.055, 0.0, 0.012, (0x22, 0x22, 0x22), name=f'cone_{i}'))
        self._leg_home = []
        bs = list(g('bystanders'))
        for i in range(len(bs) // 2):
            h = g('bystander_d') / 2
            self.boxes.append(Box('leg', bs[2 * i], bs[2 * i + 1], 0.0, h, h, 0.0, 0.9, (0x30, 0x40, 0x70), name=f'leg_{i}'))
        if g('glass_walls'):
            ws = list(g('glass_wall_segs'))
            for i in range(len(ws) // 4):
                x1, y1, x2, y2 = ws[4 * i:4 * i + 4]
                self.boxes.append(Box('glass', (x1 + x2) / 2, (y1 + y2) / 2, math.atan2(y2 - y1, x2 - x1),
                                      math.hypot(x2 - x1, y2 - y1) / 2, 0.02, 0.0, g('glass_wall_h'),
                                      (0xa8, 0xb4, 0xb8), name=f'glass_{i}'))
        self._slip = False
        if g('traffic_light'):
            self.boxes.append(Box('light', 3.95, 4.47, math.pi, 0.035, 0.06, 0.10, 0.305, (0x15, 0x15, 0x15), name='traffic_light'))
            self.boxes.append(Box('pole', 3.95, 4.47, 0.0, 0.01, 0.01, 0.0, 0.10, (0x30, 0x30, 0x30), name='light_pole'))
            self.boxes.append(Box('monitor', 3.95, 4.20, math.pi, 0.01, 0.16, 0.10, 0.28, (0x40, 0x48, 0x60), name='ad_monitor'))
        # 표지판 (판면만 — 라이다 높이보다 위, 지주는 얇아 생략)
        for name, x, y, face, col in (('M5_slow', 2.75, 5.95, 180, (0xd0, 0x30, 0x30)), ('M5_bump', 3.45, 5.95, 180, (0xe0, 0xc0, 0x20)),
                                      ('M3_ped', 9.20, 2.50, 90, (0x20, 0x60, 0xd0)), ('M4_parking', 5.63, 4.45, 180, (0x20, 0x50, 0xc0)),
                                      ('M2_branch', 4.20, 0.94, 0, (0x20, 0x60, 0xd0))):
            self.boxes.append(Box('sign', x, y, math.radians(face), 0.005, 0.06, 0.14, 0.26, col, name=name))
            if g('sign_posts'):
                self.boxes.append(Box('pole', x, y, 0.0, 0.015, 0.015, 0.0, 0.14, (0x60, 0x60, 0x60), name=name + '_post'))
        self.hits = {}
        self.knocked = set()

    def move_bystanders(self):
        st = list(self.g('bystander_step'))
        if len(st) < 2 or (st[0] == 0.0 and st[1] == 0.0) or self.g('bystander_step_s') <= 0:
            return
        k = int((self.sim_t - self.light_t0) / self.g('bystander_step_s')) % 2
        legs = [b for b in self.boxes if b.kind == 'leg']
        if not hasattr(self, '_leg_home') or len(self._leg_home) != len(legs):
            self._leg_home = [(b.x, b.y) for b in legs]
        for b, (hx, hy) in zip(legs, self._leg_home):
            b.x, b.y = hx + k * st[0], hy + k * st[1]

    def light_color(self):
        if self.g('light_mode') == 'wait_car':
            return 'green' if getattr(self, '_light_green', False) else 'red'
        r, gr, ye = self.g('light_s')
        t = (self.sim_t - self.light_t0) % max(1e-3, r + gr + ye)
        return 'red' if t < r else ('green' if t < r + gr else 'yellow')

    # ------------------------------------------------------------------ 카메라
    def build_camera(self):
        g = self.g
        W, H = g('width'), g('height')
        f = (W / 2.0) / math.tan(math.radians(g('hfov_deg')) / 2.0)
        self.K = np.array([[f, 0, W / 2.0], [0, f, H / 2.0], [0, 0, 1]])
        u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
        self.rays = np.stack([(u - W / 2.0) / f, (v - H / 2.0) / f, np.ones_like(u)], -1).astype(np.float32)   # 광학 좌표, z = 1

    def cam_pose(self):
        """카메라 위치 C (world) 와 R (world → 광학: 행 = x 오른쪽, y 아래, z 앞)."""
        g = self.g
        yaw = self.yaw + math.radians(g('cam_yaw_deg'))
        th, rho = math.radians(g('cam_pitch_deg')), math.radians(g('cam_roll_deg'))
        f = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        lft = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        up = np.array([0.0, 0.0, 1.0])
        z = math.cos(th) * f - math.sin(th) * up
        y = -(math.sin(th) * f + math.cos(th) * up)
        x = -lft
        x, y = math.cos(rho) * x + math.sin(rho) * y, -math.sin(rho) * x + math.cos(rho) * y
        R = np.stack([x, y, z])
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        C = np.array([self.x + g('cam_x') * c - g('cam_y') * s, self.y + g('cam_x') * s + g('cam_y') * c, g('cam_height')])
        return C, R

    def render(self, want_depth):
        g = self.g
        W, H = g('width'), g('height')
        C, R = self.cam_pose()
        # 바닥: 트랙 그림 픽셀 → 영상 픽셀 호모그래피
        t = -R @ C
        A = np.array([[self.res, 0, self.x0], [0, -self.res, self.ytop], [0, 0, 1]])
        Hm = self.K @ np.column_stack([R[:, 0], R[:, 1], t]) @ A
        img = cv2.warpPerspective(self.raster, Hm, (W, H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                                  borderValue=COLORS['venue']).astype(np.float32)
        dz = self.rays @ R[:, 2].astype(np.float32)                  # 광선의 world z 성분
        ground = dz < -1e-4
        depth = np.where(ground, -C[2] / np.where(ground, dz, -1.0), np.inf).astype(np.float32)
        far = depth > 12.0
        img[far] = COLORS['wall']
        depth[far] = np.inf
        holes = np.zeros((H, W), bool)
        Rw = R.astype(np.float32)                                     # 광학 → world: d_world = d_cam @ R
        light = self.light_color()
        for b in self.boxes:
            if not b.alive:
                continue
            rect = self.box_rect(b, C, R, W, H)
            if rect is None:
                continue
            x1, y1, x2, y2 = rect
            d = self.rays[y1:y2, x1:x2] @ Rw                         # world 방향 (광학 z = 1 기준 길이)
            c, s = math.cos(b.yaw), math.sin(b.yaw)
            o = np.array([C[0] - b.x, C[1] - b.y, C[2]], np.float32)
            ol = np.array([o[0] * c + o[1] * s, -o[0] * s + o[1] * c, o[2]], np.float32)
            dl = np.stack([d[..., 0] * c + d[..., 1] * s, -d[..., 0] * s + d[..., 1] * c, d[..., 2]], -1)
            lo = np.array([-b.hl, -b.hw, b.z0], np.float32)
            hi = np.array([b.hl, b.hw, b.z1], np.float32)
            with np.errstate(divide='ignore', invalid='ignore'):
                inv = 1.0 / dl
                t1, t2 = (lo - ol) * inv, (hi - ol) * inv
            tmin, tmax = np.minimum(t1, t2), np.maximum(t1, t2)
            tn = np.nanmax(tmin, -1)
            tf = np.nanmin(tmax, -1)
            axis = np.nanargmax(tmin, -1)
            hit = (tn <= tf) & (tf > 0) & (tn > 0.01)
            sub = depth[y1:y2, x1:x2]
            hit &= tn < sub
            if not hit.any():
                continue
            sub[hit] = tn[hit]
            shade = np.choose(axis, [1.0, 0.8, 0.6]).astype(np.float32)[..., None]
            col = np.broadcast_to(b.color, d.shape).copy() * shade
            if b.kind == 'light':
                # 앞면 (local +x 쪽 = 방향 yaw 쪽, 차를 보는 면) 의 등 세 칸: 위 빨강, 가운데 노랑, 아래 초록
                p = ol + dl * tn[..., None]
                zc = (b.z0 + b.z1) / 2
                for k, (name, rgb) in enumerate((('red', (255, 42, 26)), ('yellow', (255, 176, 0)), ('green', (25, 224, 122)))):
                    lz = zc + (1 - k) * 0.065
                    lamp = (axis == 0) & (p[..., 0] > 0) & ((p[..., 1]) ** 2 + (p[..., 2] - lz) ** 2 < 0.028 ** 2)
                    on = np.array(rgb, np.float32) * (1.0 if name == light else 0.18)
                    col[lamp] = on
            elif b.kind == 'monitor':
                p = ol + dl * tn[..., None]
                scr = (axis == 0) & (np.abs(p[..., 1]) < 0.15) & (p[..., 2] > b.z0 + 0.01) & (p[..., 2] < b.z1 - 0.01)
                stripes = ((np.floor((p[..., 1] + 0.2) / 0.04) % 3)[..., None] == np.arange(3)).astype(np.float32) * 180 + 40
                col[scr] = stripes[scr]
            elif b.kind == 'limo':
                if self.g('depth_dropout') > 0:
                    holes[y1:y2, x1:x2] |= hit & (self.np_rng.random(hit.shape) < self.g('depth_dropout'))
            img[y1:y2, x1:x2][hit] = col[hit]
        img *= g('brightness')
        if g('image_noise') > 0:
            img += self.np_rng.normal(0, g('image_noise'), img.shape).astype(np.float32)
        img = cv2.GaussianBlur(np.clip(img, 0, 255).astype(np.uint8), (3, 3), 0)
        dimg = None
        if want_depth:
            dm = np.where(np.isfinite(depth) & (depth >= g('depth_min')) & (depth <= g('depth_max')) & ~holes, depth * 1000.0, 0)
            dimg = dm.astype(np.uint16)
        return img, dimg

    def box_rect(self, b, C, R, W, H):
        """물체가 화면에 차지하는 사각형 (여유 2 px). 안 보이면 None."""
        if b.kind != 'glass' and (b.x - C[0]) ** 2 + (b.y - C[1]) ** 2 > max(16.0, self.g('sign_range_m') ** 2):
            return None
        pts = np.array([(x, y, z) for x, y in b.corners2d() for z in (b.z0, b.z1)])
        pc = (pts - C) @ R.T
        front = pc[:, 2] > 0.02
        if not front.any():
            return None
        u = self.K[0, 0] * pc[front, 0] / pc[front, 2] + self.K[0, 2]
        v = self.K[1, 1] * pc[front, 1] / pc[front, 2] + self.K[1, 2]
        x1, x2 = int(max(0, u.min() - 2)), int(min(W, u.max() + 3))
        y1, y2 = int(max(0, v.min() - 2)), int(min(H, v.max() + 3))
        if not front.all():
            # 카메라 옆에 걸친 물체: 뒤쪽 꼭짓점은 화면 밖 — 그쪽 가장자리까지 넓힌다
            back = pc[~front]
            if (back[:, 0] < 0).any():
                x1 = 0
            if (back[:, 0] > 0).any():
                x2 = W
            if (back[:, 1] < 0).any():
                y1 = 0
            if (back[:, 1] > 0).any():
                y2 = H
        if x1 >= x2 or y1 >= y2:
            return None
        return x1, y1, x2, y2

    # ------------------------------------------------------------------ 라이다
    def scan_ranges(self):
        g = self.g
        n = int(g('scan_points'))
        a = -math.pi + 2 * math.pi * np.arange(n) / n
        lx = self.x + g('laser_x') * math.cos(self.yaw)
        ly = self.y + g('laser_x') * math.sin(self.yaw)
        dx, dy = np.cos(a + self.yaw), np.sin(a + self.yaw)
        r = np.full(n, np.inf)
        for b in self.boxes:
            if not b.alive or not (b.z0 <= LIDAR_Z <= b.z1) or (b.kind == 'glass' and not g('glass_lidar')):
                continue
            c, s = math.cos(b.yaw), math.sin(b.yaw)
            ox, oy = (lx - b.x) * c + (ly - b.y) * s, -(lx - b.x) * s + (ly - b.y) * c
            ddx, ddy = dx * c + dy * s, -dx * s + dy * c
            with np.errstate(divide='ignore', invalid='ignore'):
                tx1, tx2 = (-b.hl - ox) / ddx, (b.hl - ox) / ddx
                ty1, ty2 = (-b.hw - oy) / ddy, (b.hw - oy) / ddy
            tn = np.nanmax([np.minimum(tx1, tx2), np.minimum(ty1, ty2)], 0)
            tf = np.nanmin([np.maximum(tx1, tx2), np.maximum(ty1, ty2)], 0)
            hit = (tn <= tf) & (tn > 0)
            r = np.where(hit & (tn < r), tn, r)
        r = r + self.np_rng.normal(0, g('scan_noise'), n)
        r[(r < g('scan_min')) | (r > g('scan_max'))] = np.inf
        if g('scan_flip'):
            r = np.roll(r[::-1], 1).copy()     # 각 a → −a (인덱스 i ↔ (n − i) % n)
        return r.astype(np.float32)

    # ------------------------------------------------------------------ 물리
    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_t = self.sim_t

    def on_start_pose(self, msg):
        self.next_start = (msg.x, msg.y, msg.theta)

    def on_reset(self, _req, res):
        self.reset_world()
        self.stats_reset()
        self.cmd = (0.0, 0.0)
        res.success = True
        res.message = (f'sim2d 리셋: LIMO ({self.x:.2f}, {self.y:.2f}) yaw {math.degrees(self.yaw):.0f}°, 신호등 빨강부터, '
                       f'갈림길 표지판 {self.branch}')
        self.get_logger().info(res.message)
        return res

    def car_rect(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + sx * CAR_HL * c - sy * CAR_HW * s, y + sx * CAR_HL * s + sy * CAR_HW * c)
                for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]

    def step(self):
        self.move_bystanders()
        import time
        g = self.g
        now = time.monotonic()
        if self.wall_last is None:
            self.wall_last = now
            return
        dt = min(0.05, now - self.wall_last)
        self.wall_last = now
        self.sim_t += dt
        vc, wc = self.cmd if self.sim_t - self.cmd_t < g('cmd_timeout') else (0.0, 0.0)
        vc = max(-g('max_v'), min(g('max_v'), vc))
        wc = max(-g('max_w'), min(g('max_w'), wc))
        if abs(vc) < g('v_deadband') and abs(wc) < g('w_deadband'):
            vc = wc = 0.0
        elif abs(vc) < g('v_deadband'):
            vc = 0.0
        elif abs(wc) < g('w_deadband') and abs(vc) < 1e-3:
            wc = 0.0
        vc, wc = vc * g('act_v_gain'), wc * g('act_w_gain')   # 실제 차가 명령보다 덜·더 감/돎 (스키드·배터리)
        self.v += (vc - self.v) * min(1.0, dt / g('tau_v'))
        if g('drive_mode') == 'ackermann':
            # limo_base 아커만: r = v / w → |r| ≥ 최소 반경, 조향 = atan(L / r). v = 0 이고 w ≠ 0 이면 최대로 꺾음 (안 돎)
            L, dmax = g('wheelbase'), math.atan(g('wheelbase') / g('ack_min_radius'))
            if abs(wc) < 1e-4:
                dc = 0.0
            elif abs(vc) < 1e-4:
                dc = math.copysign(dmax, wc)
            else:
                dc = max(-dmax, min(dmax, math.atan(L * wc / vc)))
            dc *= g('ack_steer_scale')
            self.steer = getattr(self, 'steer', 0.0)
            self.steer += (dc - self.steer) * min(1.0, dt / g('steer_tau'))
            self.w = self.v * math.tan(self.steer) / L     # rear: v = 뒷축 속도 → 명령 w 가 그대로 방향 변화율 (잘리지 않으면)
        else:
            self.w += (wc - self.w) * min(1.0, dt / g('tau_w'))
        # 바퀴 헛돎: 빠른 제자리 회전이면 바퀴는 도는데 몸체는 덜 돎 (휠 odom 은 바퀴, IMU 는 몸체)
        pivot = abs(self.w) >= g('pivot_slip_w') and abs(self.v) < g('pivot_slip_v')
        if pivot and not getattr(self, '_pivot', False):
            self._slip = g('pivot_slip_ratio') < 1.0 and self.rng.random() < g('pivot_slip_prob')
            if self._slip:
                self.get_logger().warn(f'[sim2d] 바퀴 헛돎 시작: w {self.w:+.2f} → 몸체 ×{g("pivot_slip_ratio")} '
                                       f'— 차 ({self.x:.2f}, {self.y:.2f})')
        self._pivot = pivot
        bw = self.w * g('pivot_slip_ratio') if (pivot and self._slip) else self.w
        nyaw = wrap(self.yaw + bw * dt)
        if g('drive_mode') == 'ackermann' and g('ack_ref') == 'rear':
            # 뒷축 가운데를 v 로 적분하고 base_link 는 그 L/2 앞 — 몸 가운데는 옆으로도 움직임 (미끄럼각 atan(tanδ / 2))
            hl = 0.5 * g('wheelbase')
            rx, ry = self.x - hl * math.cos(self.yaw), self.y - hl * math.sin(self.yaw)
            rx += self.v * dt * math.cos(self.yaw + 0.5 * bw * dt)
            ry += self.v * dt * math.sin(self.yaw + 0.5 * bw * dt)
            nx, ny = rx + hl * math.cos(nyaw), ry + hl * math.sin(nyaw)
        else:
            nx = self.x + self.v * dt * math.cos(self.yaw + 0.5 * bw * dt)
            ny = self.y + self.v * dt * math.sin(self.yaw + 0.5 * bw * dt)
        rect = self.car_rect(nx, ny, nyaw)
        blocked = None
        for b in self.boxes:
            if b.alive and b.kind in ('limo', 'cone', 'cone_base', 'light', 'pole', 'monitor', 'leg', 'glass') and \
                    (b.kind == 'glass' or (b.x - nx) ** 2 + (b.y - ny) ** 2 < 0.6) and rects_overlap(rect, b.corners2d()):
                if b.kind.startswith('cone'):
                    if b.name not in self.knocked:
                        self.knocked.add(b.name)
                        self.get_logger().warn(f'[sim2d] 라바콘 {b.name} 건드림 (넘어짐) — 차 ({nx:.2f}, {ny:.2f})')
                    for bb in self.boxes:
                        if bb.name == b.name:
                            bb.alive = False
                    continue
                blocked = b
                break
        if blocked is not None:
            if blocked.name not in self.hits:
                self.get_logger().error(f'[sim2d] 충돌: {blocked.name} — 차 ({self.x:.2f}, {self.y:.2f}) yaw {math.degrees(self.yaw):.0f}°')
            self.hits[blocked.name] = self.hits.get(blocked.name, 0) + 1
            self.v = self.w = 0.0
            moved_v, moved_w, body_w = 0.0, 0.0, 0.0
        else:
            self.x, self.y, self.yaw = nx, ny, nyaw
            moved_v, moved_w, body_w = self.v, self.w, bw
        # 오도메트리 (배율 오차 포함, 출발 자세 기준)
        ov, ow = moved_v * g('odom_v_scale'), moved_w * g('odom_w_scale')
        self.ox += ov * dt * math.cos(self.oyaw + 0.5 * ow * dt)
        self.oy += ov * dt * math.sin(self.oyaw + 0.5 * ow * dt)
        self.oyaw = wrap(self.oyaw + ow * dt)
        self.odom_tw = (ov, ow)
        self.true_w = body_w
        self.imu_yaw = wrap(self.imu_yaw + body_w * dt)
        self.track_stats(dt)
        if g('light_mode') == 'wait_car' and not self._light_green:
            x0, y0, x1, y1 = g('light_stop_box')
            if x0 <= self.x <= x1 and y0 <= self.y <= y1 and abs(self.v) < 0.01:
                if self._light_wait is None:
                    self._light_wait = self.sim_t
                    self.get_logger().info(f'[sim2d] 신호등: 차가 정지선 앞에 섬 ({self.x:.2f}, {self.y:.2f}) — '
                                           f'{g("light_green_after_s"):.0f} s 뒤 초록')
                elif self.sim_t - self._light_wait >= g('light_green_after_s'):
                    self._light_green = True
                    self.get_logger().info('[sim2d] 신호등: 초록으로 바뀜')
            else:
                self._light_wait = None
        self.pub_clock.publish(Clock(clock=stamp(self.sim_t)))
        st = stamp(self.sim_t)
        for key, hz, fn in (('odom', g('odom_hz'), self.publish_odom), ('imu', g('imu_hz'), self.publish_imu),
                            ('scan', g('scan_hz'), self.publish_scan), ('status', 1.0, self.publish_status),
                            ('pose', g('pose_hz'), self.publish_pose)):
            if self.sim_t >= self.due[key]:
                self.due[key] = self.sim_t + 1.0 / hz
                fn(st)
        if self.sim_t >= self.due['cam']:
            self.due['cam'] = self.sim_t + 1.0 / g('camera_hz')
            if self.sim_t < getattr(self, '_stall_until', 0.0):
                return                       # 실차 모드: 영상 끊김
            if g('cam_stall_rate') > 0 and self.rng.random() < g('cam_stall_rate'):
                lo, hi = g('cam_stall_s')
                self._stall_until = self.sim_t + self.rng.uniform(lo, hi)
                return
            want_depth = self.sim_t >= self.due['depth']
            if want_depth:
                self.due['depth'] = self.sim_t + 1.0 / g('depth_hz')
            self.publish_camera(st, want_depth)

    # ------------------------------------------------------------------ 기록
    def stats_reset(self):
        self.stats = {'min_line_m': float('inf'), 'touch_s': 0.0, 'dist_m': 0.0, 'since': getattr(self, 'sim_t', 0.0),
                      'lane_out': 0, 'lane_out_s': 0.0,   # 규정 차선 이탈 (한쪽 두 바퀴가 선 밖) 횟수·시간 — 라바콘 구간 면제는 위치로 거를 것
                      'lane_outs': [],       # [x, y, 쪽 L/R] (처음 30 번)
                      'cells': {},           # 1 m 칸 'x,y' → [그 칸에서 바퀴~노란 선 최소 거리, 선 밟은 시간]
                      'stops': []}           # 정지선 앞에서 멈출 때마다 바퀴 맨 앞 → 정지선 가까운 끝 [m] (음수 = 밟음)
        self._still_s, self._stop_logged = 0.0, False
        self.prev_xy = None
        self._out_armed, self._out_side = False, None

    def track_stats(self, dt):
        dmin = float('inf')
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        out = []                             # WHEELS 순서: 앞왼, 앞오, 뒤왼, 뒤오
        for wx, wy in WHEELS:
            px, py = self.world_px(self.x + wx * c - wy * s, self.y + wx * s + wy * c)
            i, j = int(py), int(px)
            if 0 <= i < self.yellow_dist.shape[0] and 0 <= j < self.yellow_dist.shape[1]:
                dmin = min(dmin, float(self.yellow_dist[i, j]))
                out.append(bool(self.off_mask[i, j]))
            else:
                out.append(True)
        # 규정 차선 이탈: 한쪽 (왼쪽 두 바퀴 또는 오른쪽 두 바퀴) 이 모두 선 밖. 출발 칸은 트랙 데이터에서 off_road 라
        # 네 바퀴가 한 번 다 트랙 안에 들어온 뒤부터 셈
        if not getattr(self, '_out_armed', False):
            self._out_armed = not any(out)
        else:
            side = 'L' if (out[0] and out[2]) else 'R' if (out[1] and out[3]) else None
            if side is not None:
                self.stats['lane_out_s'] += dt
            if side is not None and getattr(self, '_out_side', None) is None:
                self.stats['lane_out'] += 1
                if len(self.stats['lane_outs']) < 30:
                    self.stats['lane_outs'].append([round(self.x, 2), round(self.y, 2), side])
                self.get_logger().warn(f'[sim2d] 차선 이탈 (규정: {"왼" if side == "L" else "오른"}쪽 두 바퀴가 선 밖) — 차 '
                                       f'({self.x:.2f}, {self.y:.2f}) yaw {math.degrees(self.yaw):.0f}°, {self.stats["lane_out"]} 번째')
            self._out_side = side
        self.last_line = dmin
        self.stats['min_line_m'] = min(self.stats['min_line_m'], dmin)
        touching = dmin < 0.02              # 바퀴 접지 반폭 (~2 cm) 안에 노란 선
        if touching:
            self.stats['touch_s'] += dt
        cell = self.stats['cells'].setdefault(f'{int(self.x)},{int(self.y)}', [float('inf'), 0.0])
        cell[0] = min(cell[0], dmin)
        cell[1] += dt if touching else 0.0
        if touching != getattr(self, '_touching', False):
            self._touching = touching
            if touching:
                self.get_logger().warn(f'[sim2d] 바퀴가 노란 선을 밟음 — 차 ({self.x:.2f}, {self.y:.2f}) yaw {math.degrees(self.yaw):.0f}°')
            else:
                self.get_logger().info(f'[sim2d] 노란 선에서 벗어남 — 차 ({self.x:.2f}, {self.y:.2f})')
        if abs(self.v) < 0.003 and abs(self.w) < 0.01:
            self._still_s += dt
        else:
            self._still_s, self._stop_logged = 0.0, False
        if self._still_s > 0.5 and not self._stop_logged:
            self._stop_logged = True
            gap = self.stop_line_gap()
            if gap is not None:
                self.stats['stops'].append(round(gap, 4))
                msg = f'[sim2d] 정지선 앞 정지: 바퀴 맨 앞 → 정지선 {gap * 100:+.1f} cm — 차 ({self.x:.2f}, {self.y:.2f})'
                if gap < 0:   # 한 줄에서 심각도를 바꾸면 rclpy (Jazzy) 가 ValueError 로 노드를 죽임
                    self.get_logger().warning(msg + ' — 정지선 밟음')
                else:
                    self.get_logger().info(msg)
        if self.prev_xy is not None:
            self.stats['dist_m'] += math.dist(self.prev_xy, (self.x, self.y))
        self.prev_xy = (self.x, self.y)

    def stop_line_gap(self):
        """차 앞 가까이 차로를 가로지르는 정지선이 있으면 바퀴 맨 앞 → 그 선 가까운 끝 [m] (진행 방향). 없으면 None."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ax, ay = self.x + WHEELS[0][0] * c, self.y + WHEELS[0][0] * s
        best = None
        for P in self.stop_lines:
            along = (P[:, 0] - ax) * c + (P[:, 1] - ay) * s
            lat = -(P[:, 0] - ax) * s + (P[:, 1] - ay) * c
            if lat.min() > 0.1 or lat.max() < -0.1 or along.max() - along.min() > 0.15:
                continue                    # 차 앞을 가로지르지 않음 (옆 차로·진행 방향으로 긴 선)
            gap = float(along.min()) - self.g('wheel_r')
            if -0.25 < gap < 0.5 and (best is None or abs(gap) < abs(best)):
                best = gap
        return best

    def publish_pose(self, _st):
        self.pub_pose.publish(Pose2D(x=self.x, y=self.y, theta=self.yaw))

    def publish_status(self, _st):
        d = dict(t=round(self.sim_t, 2), x=round(self.x, 3), y=round(self.y, 3), yaw_deg=round(math.degrees(self.yaw), 1),
                 v=round(self.v, 3), w=round(self.w, 3), line_now_m=round(self.last_line, 3),
                 min_line_m=round(self.stats['min_line_m'], 3), touch_s=round(self.stats['touch_s'], 2),
                 lane_out=self.stats['lane_out'], lane_out_s=round(self.stats['lane_out_s'], 2), lane_outs=self.stats['lane_outs'],
                 dist_m=round(self.stats['dist_m'], 2), collisions=dict(self.hits), knocked=sorted(self.knocked),
                 cells={k: [round(v[0], 3), round(v[1], 2)] for k, v in self.stats['cells'].items()},
                 stops=self.stats['stops'],
                 light=self.light_color(), branch=self.branch,
                 odom_err_m=round(self.odom_error(), 3))
        self.pub_status.publish(String(data=json.dumps(d)))

    def odom_error(self):
        sx, sy, syaw = self.start
        wx = sx + self.ox * math.cos(syaw) - self.oy * math.sin(syaw)
        wy = sy + self.ox * math.sin(syaw) + self.oy * math.cos(syaw)
        return math.dist((wx, wy), (self.x, self.y))

    # ------------------------------------------------------------------ 발행
    def publish_odom(self, st):
        m = Odometry()
        m.header.stamp, m.header.frame_id, m.child_frame_id = st, 'odom', 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = self.ox, self.oy
        q = quat_yaw(self.oyaw)
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        m.twist.twist.linear.x, m.twist.twist.angular.z = self.odom_tw
        self.pub_odom.publish(m)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = st, 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.ox, self.oy
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = q
        self.tf.sendTransform(t)

    def publish_imu(self, st):
        m = Imu()
        m.header.stamp, m.header.frame_id = st, 'imu_link'
        q = quat_yaw(self.imu_yaw)
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
        m.angular_velocity.z = self.true_w * self.g('imu_w_scale') + self.g('imu_bias') + float(self.np_rng.normal(0, self.g('imu_noise')))
        m.linear_acceleration.z = 9.81
        self.pub_imu.publish(m)

    def publish_scan(self, st):
        g = self.g
        n = int(g('scan_points'))
        m = LaserScan()
        m.header.stamp, m.header.frame_id = st, 'laser_link'
        m.angle_min, m.angle_max = -math.pi, math.pi - 2 * math.pi / n
        m.angle_increment = 2 * math.pi / n
        m.scan_time = 1.0 / g('scan_hz')
        m.range_min, m.range_max = g('scan_min'), g('scan_max')
        m.ranges = self.scan_ranges().tolist()
        self.pub_scan.publish(m)

    def cam_info(self, st, frame):
        g = self.g
        m = CameraInfo()
        m.header.stamp, m.header.frame_id = st, frame
        m.width, m.height = g('width'), g('height')
        m.distortion_model = 'plumb_bob'
        m.d = [0.0] * 5
        m.k = [float(v) for v in self.K.flatten()]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]   # humble: float 만
        m.p = [float(v) for v in (self.K[0, 0], 0.0, self.K[0, 2], 0.0, 0.0, self.K[1, 1], self.K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0)]
        return m

    def publish_camera(self, st, want_depth):
        img, dimg = self.render(want_depth)
        m = Image()
        m.header.stamp, m.header.frame_id = st, 'camera_color_optical_frame'
        m.height, m.width = img.shape[:2]
        m.encoding, m.step = 'rgb8', img.shape[1] * 3
        m.data = img.tobytes()
        self.pub_img.publish(m)
        self.pub_cinfo.publish(self.cam_info(st, 'camera_color_optical_frame'))
        if dimg is not None:
            d = Image()
            d.header.stamp, d.header.frame_id = st, 'camera_depth_optical_frame'
            d.height, d.width = dimg.shape
            d.encoding, d.step = '16UC1', dimg.shape[1] * 2
            d.data = dimg.tobytes()
            self.pub_depth.publish(d)
            self.pub_dinfo.publish(self.cam_info(st, 'camera_depth_optical_frame'))
        if self.g('fake_detections'):
            self.publish_detections(m.header)

    SIGN_LABELS = {'M5_slow': 'slow', 'M5_bump': 'speed_bump', 'M3_ped': 'crosswalk', 'M4_parking': 'parking', 'M2_branch': None}

    def face_box(self, b, C, R, half_w, z0, z1):
        """물체 앞면 (local +x 쪽, 방향 b.yaw) 의 영상 박스. 앞면이 카메라 쪽이 아니거나 반 넘게 잘리면 None."""
        W, H = self.g('width'), self.g('height')
        fx, fy = math.cos(b.yaw), math.sin(b.yaw)
        vx, vy = C[0] - b.x, C[1] - b.y
        dist = math.hypot(vx, vy)
        if dist > self.g('sign_range_m') or (fx * vx + fy * vy) < math.cos(math.radians(75)) * dist:
            return None
        cx, cy = b.x + fx * b.hl, b.y + fy * b.hl
        pts = np.array([(cx - fy * sw * half_w, cy + fx * sw * half_w, z) for sw in (-1, 1) for z in (z0, z1)])
        pc = (pts - C) @ R.T
        if (pc[:, 2] < 0.1).any():
            return None
        u = self.K[0, 0] * pc[:, 0] / pc[:, 2] + self.K[0, 2]
        v = self.K[1, 1] * pc[:, 1] / pc[:, 2] + self.K[1, 2]
        x1, x2, y1, y2 = u.min(), u.max(), v.min(), v.max()
        vis = max(0.0, min(x2, W) - max(x1, 0)) * max(0.0, min(y2, H) - max(y1, 0)) / max(1.0, (x2 - x1) * (y2 - y1))
        if vis < 0.5:
            return None
        return int(max(0, x1)), int(max(0, y1)), int(min(W - 1, x2)), int(min(H - 1, y2))

    def publish_detections(self, header):
        W, H = self.g('width'), self.g('height')
        C, R = self.cam_pose()
        det = DetectionArray(header=header, image_width=W, image_height=H)
        light = DetectionArray(header=header, image_width=W, image_height=H)
        for b in self.boxes:
            if not b.alive:
                continue
            if b.kind == 'sign':
                label = self.SIGN_LABELS.get(b.name, '')
                if label is None:
                    label = 'turn_a' if self.branch == 'A' else 'turn_b'
                bb = self.face_box(b, C, R, b.hw, b.z0, b.z1)
                if label and bb and not self.det_missed(b.name):
                    det.detections.append(Detection(label=label, confidence=self.det_confidence(bb), x1=bb[0], y1=bb[1], x2=bb[2], y2=bb[3]))
            elif b.kind == 'light':
                bb = self.face_box(b, C, R, b.hw, b.z0, b.z1)
                if bb:
                    light.detections.append(Detection(label='traffic_light', confidence=0.9, x1=bb[0], y1=bb[1], x2=bb[2], y2=bb[3]))
        fk = getattr(self, '_false', None)
        if fk is None and self.g('det_false_rate') > 0 and self.rng.random() < self.g('det_false_rate'):
            lo, hi = self.g('det_false_box_px')
            h = int(self.rng.uniform(lo, hi))
            w = max(10, int(h * self.rng.uniform(0.6, 1.2)))
            x1 = int(self.rng.uniform(0, max(1, W - w)))
            y1 = int(self.rng.uniform(0, max(1, H - h)))
            n0, n1 = self.g('det_false_frames')
            fk = self._false = [self.rng.choice(list(self.g('det_false_labels'))), (x1, y1, min(W - 1, x1 + w), min(H - 1, y1 + h)),
                                self.rng.randint(int(n0), int(n1))]
            self.get_logger().info(f'[sim2d] 실차 모드: 가짜 {fk[0]} 박스 높이 {fk[1][3] - fk[1][1]}px, {fk[2]} 장')
        if fk is not None:
            det.detections.append(Detection(label=fk[0], confidence=float(self.rng.uniform(0.5, 0.7)),
                                            x1=fk[1][0], y1=fk[1][1], x2=fk[1][2], y2=fk[1][3]))
            fk[2] -= 1
            if fk[2] <= 0:
                self._false = None
        self.pub_det.publish(det)
        self.pub_light_det.publish(light)

    def det_missed(self, name):
        """실차 모드: 이 장에서 표지판을 놓쳤나 (놓침이 몰려 오게 앞 장 결과를 기억)."""
        p = self.g('det_miss')
        if p <= 0:
            return False
        last = self._det_last_miss.get(name, False) if hasattr(self, '_det_last_miss') else False
        if not hasattr(self, '_det_last_miss'):
            self._det_last_miss = {}
        miss = self.rng.random() < (max(p, self.g('det_miss_stick')) if last else p)
        self._det_last_miss[name] = miss
        return miss

    def det_confidence(self, bb):
        """실차 모드: 확신도 — 박스가 작을수록 (멀수록) 범위 아래쪽."""
        lo, hi = self.g('det_conf')
        if hi <= lo:
            return float(lo)
        near = min(1.0, (bb[3] - bb[1]) / 80.0)
        return float(lo + (hi - lo) * (0.5 * near + 0.5 * self.rng.random()))

    def publish_static_tf(self):
        g = self.g
        out = []

        def tf(parent, child, xyz, q=(0.0, 0.0, 0.0, 1.0)):
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            r = t.transform.rotation
            r.x, r.y, r.z, r.w = q
            out.append(t)
        tf('base_link', 'laser_link', (g('laser_x'), 0.0, LIDAR_Z - BASE_H))
        tf('base_link', 'camera_link', (g('cam_x'), g('cam_y'), g('cam_height') - BASE_H))
        tf('base_link', 'imu_link', (0.0, 0.0, 0.0))
        opt = (-0.5, 0.5, -0.5, 0.5)       # camera_link (x 앞) → 광학 (z 앞, x 오른쪽, y 아래)
        tf('camera_link', 'camera_color_optical_frame', (0.0, 0.0, 0.0), opt)
        tf('camera_link', 'camera_depth_optical_frame', (0.0, 0.0, 0.0), opt)
        self.static_tf.sendTransform(out)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Sim2D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
