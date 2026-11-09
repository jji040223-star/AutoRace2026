"""슬라이딩 윈도우 차선 추종 노드 (LIMO Pro, 실차·시뮬 공통).

  sub  image_topic   (기본 /camera/color/image_raw, sensor_msgs/Image rgb8|bgr8)  — 실차 Orbbec Dabai / autorace_sim 같은 이름
  pub  cmd_vel_topic (기본 /cmd_vel, geometry_msgs/Twist)                          — 실차 limo_base / autorace_sim 이 구독
  pub  debug_topic   (기본 /lane/debug, sensor_msgs/Image bgr8, debug_hz 로 제한)    — 왼쪽 BEV + 상자·피팅, 오른쪽 마스크
  sub  lane_cmd_topic (기본 /mission/lane_cmd, autorace_msgs/LaneCmd)               — mission_manager 지시: max_speed·stop·reset_window·follow (M2 갈림길: 한 선만)
                      (속도 상한은 speed_ramp 로 서서히, stop 은 바로. lane_cmd_timeout_s 동안 안 오면 둘 다 해제)
  pub  state_topic    (기본 /lane/state, autorace_msgs/LaneState)                   — 영상마다: 중심 오차, 차로 폭 (지그재그), 정지선 거리

처리: 카메라 영상을 process_width 로 줄여 SlidingWindowLaneDetector (sliding_window.py) → offset·curvature
→ 원본 state_machine._lane_steer 와 같은 조향: 목표 = offset + curve_steer_bias × curvature → PID (kp/ki/kd 원본 값),
  차선 소실 = 직전 조향 EMA (±0.30 클램프) 를 유지하며 3 s 시정수로 직진 쪽으로 (직진 폴백은 커브를 뚫고 나가므로)
→ 원본은 서보 조향 (steer = steer_center + steer_scale × PID) 이라 PID 출력 = 경로 곡률. LIMO /cmd_vel 에서도 같게
  angular.z = −angular_scale × PID × (v / linear_speed) — 직선 속도에서는 angular_scale × PID, 감속하면 각속도도 같이 줄어
  곡률이 유지된다 (각속도를 그대로 두면 코너에서 감속할 때 반경이 작아져 안쪽 선을 밟는다).
  linear.x = v = linear_speed × (1 − curve_slow × |curvature|) (원본 스로틀 감속과 같은 식, 하한 slow_speed).
  원본에 없는 것: lost_stop_s 동안 계속 못 찾으면 정지 (실차 안전).

BEV 점 (birdeye_src_ratio) 을 비워 두면 카메라 높이·기울기·화각으로 계산한다: 카메라 앞 bev_near_m ~ bev_far_m,
좌우 ±bev_half_width_m 의 바닥 사다리꼴 → birdeye_dst_ratio 사각형. 반폭 = 차로 반폭이면 BEV 에서 차로 폭이
0.6 × process_width px (320 → 192 px = 원본 lane_width_init) 가 되어 원본의 픽셀 파라미터를 그대로 쓸 수 있다.
"""
import math
import signal
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from autorace_msgs.msg import LaneCmd, LaneState
from sensor_msgs.msg import Image

from .pid import PID
from .sliding_window import SlidingWindowLaneDetector

# 검출기 파라미터 = SlidingWindowLaneDetector.__init__ 인자 (이름·기본값 원본과 같음)
DETECTOR_PARAMS = {
    'roi_top_ratio': 0.35, 'min_pixels': 40, 'single_line_offset': 0.25,
    'mask_mode': 'hsv', 'bright_thresh': 160, 'use_white': True, 'use_yellow': True,
    'white_hsv_lo': [0, 0, 180], 'white_hsv_hi': [179, 60, 255],
    'yellow_hsv_lo': [22, 45, 140], 'yellow_hsv_hi': [40, 255, 255],
    'num_bands': 4, 'morph_kernel': 3, 'width_ema': 0.1, 'smooth_alpha': 0.5,
    'use_birdeye': True, 'birdeye_dst_ratio': [0.20, 0.00, 0.80, 0.00, 0.80, 1.00, 0.20, 1.00],
    'use_guided_band': True, 'guide_margin_px': 60, 'guide_margin_growth_px': 10,
    'guide_min_pixels': 20, 'guide_use_previous_frame': True, 'guide_max_jump_px': 80,
    'use_lookahead_control': True, 'near_weight': 0.7, 'lookahead_weight': 0.3,
    'lookahead_band_index': -1, 'adaptive_lookahead': False,
    'curve_lookahead_weight': 0.4, 'curve_lookahead_thresh': 0.25,
    'center_jump_max_ratio': 0.10, 'lane_width_init': 192.0, 'lane_width_tol': 0.0,
    'sw_enabled': True, 'sw_num_boxes': 9, 'sw_box_margin': 55, 'sw_max_shift': 35,
    'sw_min_box_px': 8, 'sw_min_boxes': 3, 'sw_min_pixels': 50, 'sw_max_resid_px': 14.0,
    'sw_peak_min_px': 40, 'sw_max_peaks': 3, 'sw_cross_row_frac': 0.45, 'sw_side_default': -1,
    # LIMO 추가 (원본에 없음): 정지선 검출
    'sw_single_normal': False,          # LIMO 추가 (시험용): 단선 중심을 선의 법선 방향으로 반 차폭. 2D 시뮬에서 3 차선 진입 코너는
                                        # 실차형 카메라에서 8 cm 로 좋아졌지만 기본 카메라는 나빠지고 왼쪽 위 코너 14.5 → 9 cm — 기본은 원본 (가로)
    'stop_line_enabled': False, 'stop_line_min_cov': 0.9, 'stop_line_min_rows': 10, 'stop_line_max_deg': 15.0,
}


def bev_src_from_camera(width, height, roi_top_ratio, hfov_deg, cam_height, pitch_deg, near, far, half_width, yaw_deg=0.0):
    """바닥 사다리꼴 (카메라 앞 near~far m, 좌우 ±half_width m) 을 핀홀로 투영 → ROI 기준 비율
    [좌상, 우상, 우하, 좌하] (원본 birdeye_src_ratio 순서). 정사각 픽셀, 주점 = 영상 가운데, pitch + = 아래로,
    yaw + = 카메라가 차 진행 방향보다 왼쪽을 봄 (사다리꼴은 차 진행 방향 기준이라 BEV 의 선이 차 축과 나란해진다). yaw 0 = 예전과 같음."""
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    th, ps = math.radians(pitch_deg), math.radians(yaw_deg)
    roi_top = height * roi_top_ratio
    pts = []
    for d, x in ((far, -half_width), (far, half_width), (near, half_width), (near, -half_width)):
        X, Y = d, -x                                             # 차 기준 앞, 왼쪽 (카메라 원점)
        xc = X * math.sin(ps) - Y * math.cos(ps)                 # 카메라 오른쪽
        yc = -(X * math.sin(th) * math.cos(ps) + Y * math.sin(th) * math.sin(ps)) + cam_height * math.cos(th)   # 아래
        zc = X * math.cos(th) * math.cos(ps) + Y * math.cos(th) * math.sin(ps) + cam_height * math.sin(th)      # 광축
        u = width / 2.0 + fx * xc / zc
        v = height / 2.0 + fx * yc / zc
        pts += [u / width, (v - roi_top) / (height - roi_top)]
    return pts


class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        p = self.declare_parameter
        p('image_topic', '/camera/color/image_raw')
        p('cmd_vel_topic', '/cmd_vel')
        p('debug_topic', '/lane/debug')
        p('debug_hz', 5.0)                 # 0 = 디버그 영상 안 냄
        p('enable', True)                  # false = 인식만 하고 /cmd_vel 은 0 (튜닝용)
        p('wait_mission', False)           # true = /mission/lane_cmd 를 처음 받을 때까지 출발 안 함 (drive.launch — 실차 Jetson 에서
                                           #   미션 노드가 늦게 켜져 차가 먼저 출발, 출발선 옆 과속방지턱 표지판 (M5) 을 놓쳤다 11-05)
        p('process_width', 320)            # 원본 카메라 폭 (320×160). 원본 픽셀 파라미터가 이 폭 기준
        # BEV — birdeye_src_ratio 를 비우면 아래 카메라 기하로 계산
        p('birdeye_src_ratio', [0.0])
        p('camera_hfov_deg', 71.0)         # Orbbec Dabai 컬러 (실차·autorace_sim)
        p('camera_height', 0.175)          # 바닥 → 카메라 광심 [m] (sensor_mounts camera_link z 0.03 + base_link 0.145)
        p('camera_pitch_deg', 0.0)         # 아래로 + [deg]
        p('camera_yaw_deg', 0.0)           # 카메라가 차 진행 방향보다 왼쪽을 보면 + [deg] (lane_calib 으로 잼)
        p('bev_near_m', 0.37)              # 카메라 앞 [m] — 차로 양 끝(±반폭)이 화면 안에 드는 거리부터
        p('bev_far_m', 0.75)              # 멀리 볼수록 코너 안쪽을 파고든다
        p('bev_half_width_m', 0.26)        # 차로 반폭 (선 안쪽 500 mm + 선 20 mm → 선 중심 간 520 mm)
        for k, v in DETECTOR_PARAMS.items():
            p(k, v)
        # 제어 (PID 기본값 = 원본 decision_node)
        p('kp', 0.6)
        p('ki', 0.0)
        p('kd', 0.15)
        p('angular_scale', 3.0)            # PID 출력 1 → linear_speed 로 달릴 때의 각속도 [rad/s] (곡률 = angular_scale / linear_speed)
        p('max_angular', 1.5)              # [rad/s]
        p('linear_speed', 0.3)             # [m/s] 규정집 권고 0.3~0.4
        p('slow_speed', 0.2)               # 감속 하한·차선 소실 중 속도 [m/s] (규정집 서행 0.2 이하)
        p('curve_slow', 0.5)               # 원본과 같음: v = linear_speed × (1 − curve_slow × |curvature|)
        p('curve_steer_bias', 0.5)         # 원본과 같은 식: 곡률 feed-forward (목표 offset 에 + bias × curvature). 원본 기본 0
        p('lost_stop_s', 20.0)             # 차선을 이만큼 계속 못 찾으면 정지 (원본에 없음, 실차 안전). 0 = 안 멈춤
        # 아커만 (제자리 회전 없음) 용 — 기본은 예전 동작. 2D 시뮬 11-09 e1: M8 뒤 코너 안에서 차선을 잃고 직진 폴백으로
        # 20 s (4 m) 직진해 트랙 밖
        p('lost_hold_mode', 'decay')       # decay = 원본 (직전 조향이 3 s 시정수로 직진 쪽으로) | hold = 직전 조향 그대로 유지 (코너 계속 돎)
        p('lost_stop_m', 0.0)              # 차선을 잃은 뒤 이 거리 [m] (명령 속도 적분) 를 가면 정지. 0 = 끔 (lost_stop_s 만)
        p('image_timeout_s', 0.5)          # 영상이 이만큼 안 오면 정지
        p('lane_cmd_topic', '/mission/lane_cmd')
        p('state_topic', '/lane/state')
        p('lane_cmd_timeout_s', 1.0)       # mission_manager 지시가 이만큼 끊기면 속도 상한 해제
        p('speed_ramp', 0.5)               # 속도 상한이 바뀔 때 [m/s²] — 급감속·급가속 방지 (v3: 가감속 램프)

        g = lambda k: self.get_parameter(k).value   # noqa: E731
        cfg = {k: g(k) for k in DETECTOR_PARAMS}
        for k in ('white_hsv_lo', 'white_hsv_hi', 'yellow_hsv_lo', 'yellow_hsv_hi'):
            cfg[k] = tuple(int(v) for v in cfg[k])
        self._src_cfg = [float(v) for v in g('birdeye_src_ratio')]
        self._det_cfg = cfg
        self.det = None                     # 첫 영상 크기를 보고 만든다 (BEV 자동 계산에 영상 비율이 필요)
        self.pid = PID(g('kp'), g('ki'), g('kd'), out_limit=1.0)
        self.cmd_pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.dbg_pub = self.create_publisher(Image, g('debug_topic'), 2) if g('debug_hz') > 0 else None
        self.create_subscription(Image, g('image_topic'), self.on_image, qos_profile_sensor_data)
        self.create_subscription(LaneCmd, g('lane_cmd_topic'), self.on_lane_cmd, 10)
        self.state_pub = self.create_publisher(LaneState, g('state_topic'), 10)
        self._stop = False                 # mission_manager 의 정지 지시
        self._reset_prev = False
        self._cap_target = None            # mission_manager 가 준 속도 상한 [m/s] (None = 없음)
        self._cap_stamp = None
        self._waiting_logged = False
        self._cap = float(g('linear_speed'))   # 지금 적용 중인 상한 (speed_ramp 로 _cap_target 을 따라감)
        self.create_timer(0.05, self.watchdog)
        self._last_img = self._last_found = None
        self._steer_hold = None            # 소실 폴백용 직전 조향 EMA (PID 출력 단위)
        self._lost_m = 0.0                 # 차선을 잃은 뒤 간 거리 (명령 속도 적분) [m]
        self._lost_logged = False
        self._last_t = None
        self._last_cmd = Twist()
        self._last_dbg = 0.0
        self._stat = {'n': 0, 'sw': 0, 'found': 0, 't': time.monotonic()}
        self.get_logger().info(f"lane_follower: {g('image_topic')} → {g('cmd_vel_topic')}, enable={g('enable')}")

    def clock_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_lane_cmd(self, msg):
        self._cap_target = float(msg.max_speed) if msg.max_speed > 0 else None
        self._stop = bool(msg.stop)
        if self.det is not None:
            self.det.follow = msg.follow if msg.follow in ('left', 'right') else 'both'   # M2 갈림길: 한쪽 선만
        if msg.reset_window and not self._reset_prev and self.det is not None:
            # 차선을 떠났다 돌아올 때 (M7 회피 뒤 등): 직전 피팅·중심을 버리고 새로 찾는다 (v3: 윈도우 리셋)
            self.det.reset(center_hint=0.5)    # 미션이 차를 차로 가운데쯤에 두고 넘김 (M7 변경 끝·M8 통로 끝)
            self.pid.reset()
            self._steer_hold = None
            self._lost_m, self._lost_logged = 0.0, False   # 미션 중 (내 명령이 안 쓰일 때) 쌓인 소실 거리는 버림
            self.get_logger().info('슬라이딩 윈도우 리셋 (mission_manager 지시)')
        self._reset_prev = bool(msg.reset_window)
        self._cap_stamp = self.clock_s()

    def make_detector(self, w, h):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        cfg = dict(self._det_cfg)
        if len(self._src_cfg) == 8:
            cfg['birdeye_src_ratio'] = self._src_cfg
        else:
            cfg['birdeye_src_ratio'] = bev_src_from_camera(
                w, h, cfg['roi_top_ratio'], g('camera_hfov_deg'), g('camera_height'), g('camera_pitch_deg'),
                g('bev_near_m'), g('bev_far_m'), g('bev_half_width_m'), g('camera_yaw_deg'))
        self.get_logger().info(f"처리 영상 {w}×{h}, birdeye_src_ratio = [{', '.join(f'{v:.4f}' for v in cfg['birdeye_src_ratio'])}]")
        cfg['stop_line_bev_depth_m'] = float(g('bev_far_m')) - float(g('bev_near_m'))
        cfg['stop_line_lane_m'] = 2.0 * float(g('bev_half_width_m'))
        return SlidingWindowLaneDetector(**cfg)

    def on_image(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
            self.get_logger().warn(f'지원하지 않는 영상 형식 {msg.encoding}', throttle_duration_sec=5.0)
            return
        ch = 4 if msg.encoding.endswith('a8') else 3
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // ch, ch)[:, :msg.width, :3]
        bgr = img[:, :, ::-1] if msg.encoding.startswith('rgb') else img
        pw = int(self.get_parameter('process_width').value)
        ph = int(round(msg.height * pw / msg.width))
        bgr = cv2.resize(np.ascontiguousarray(bgr), (pw, ph), interpolation=cv2.INTER_AREA)
        if self.det is None:
            self.det = self.make_detector(pw, ph)

        now = self.clock_s()               # 노드 시계 (use_sim_time 이면 시뮬 시각) — 시간 제한·디버그 주기
        # PID dt 는 영상 시각 (시뮬레이션이면 /clock) — 시뮬이 실시간보다 느려도 미분항이 맞게
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = stamp if stamp > 0 else now
        dt = 1.0 / 30.0 if self._last_t is None or t <= self._last_t else t - self._last_t
        self._last_t, self._last_img = t, now
        want_dbg = self.dbg_pub is not None and now - self._last_dbg >= 1.0 / float(self.get_parameter('debug_hz').value)
        r = self.det.process(bgr, draw_debug=want_dbg)
        self.publish_state(msg.header, r)
        self.control(r, dt, now)
        if want_dbg and r.debug is not None:
            self._last_dbg = now
            out = Image(header=msg.header, height=r.debug.shape[0], width=r.debug.shape[1], encoding='bgr8',
                        step=r.debug.shape[1] * 3, data=r.debug.tobytes())
            self.dbg_pub.publish(out)
        s = self._stat
        s['n'] += 1
        s['sw'] += r.sw_locked
        s['found'] += r.found
        wall = time.monotonic()            # 로그 주기·fps 는 벽시계
        if wall - s['t'] >= 2.0:
            self.get_logger().info(
                f"{s['n'] / (wall - s['t']):.1f} fps, 차선 {100 * s['found'] / s['n']:.0f}%, SW 락 {100 * s['sw'] / s['n']:.0f}%, "
                f"offset {r.offset:+.2f} curv {r.curvature:+.2f} nl {r.num_lanes} W {r.lane_width_px:.0f}px → "
                f"v {self._last_cmd.linear.x:.2f} w {self._last_cmd.angular.z:+.2f}"
                + (f" (상한 {self._cap:.2f})" if self._cap < float(self.get_parameter('linear_speed').value) - 1e-3 else ""))
            self._stat = {'n': 0, 'sw': 0, 'found': 0, 't': wall}

    def publish_state(self, header, r):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        # 차선 가운데까지 옆 거리: offset 은 화면 반폭 기준 → BEV 축척 (차로 폭 lane_width_init px = 2 × bev_half_width m)
        px_per_m = float(g('lane_width_init')) / (2.0 * float(g('bev_half_width_m')))
        center_m = -float(r.offset) * (float(g('process_width')) / 2.0) / px_per_m
        # 차선 방향: BEV 에서 위(앞)로 갈수록 x 가 줄면 (dx/dy > 0) 선이 왼쪽으로 간다. 세로 축척 = BEV 높이 / (far − near)
        heading = 0.0
        if r.line_side != 0 and r.bev_h > 0:
            py_per_m = float(r.bev_h) / (float(g('bev_far_m')) - float(g('bev_near_m')))
            heading = math.atan(float(r.line_slope) * py_per_m / px_per_m)
        st = LaneState(header=header, found=bool(r.found), offset=float(r.offset), curvature=float(r.curvature),
                       center_offset_m=center_m, lane_heading_rad=heading, yellow_side=int(r.line_side),
                       lane_width_px=float(r.lane_width_px), width_near_px=float(r.width_near_px),
                       width_far_px=float(r.width_far_px), line_resid_px=float(r.line_resid_px),
                       stop_line_dist_m=-1.0, stop_line_cov=float(r.stop_cov))
        if r.stop_row >= 0 and r.bev_h > 0:
            # BEV 는 바닥 직사각형(카메라 앞 near~far, 좌우 같은 폭)을 그대로 편 것이라 행 ↔ 거리가 선형. 맨 위 행 = far
            near, far = float(g('bev_near_m')), float(g('bev_far_m'))
            st.stop_line_dist_m = far - (r.stop_row + 1) / float(r.bev_h) * (far - near)
        self.state_pub.publish(st)

    def control(self, r, dt, now):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        cmd = Twist()
        lost_far = g('lost_stop_m') > 0 and self._lost_m >= g('lost_stop_m')
        if r.found:
            self._last_found = now
            self._lost_m, self._lost_logged = 0.0, False
            u = self.pid.update(r.offset + g('curve_steer_bias') * r.curvature, dt)
            hold = max(-0.30, min(0.30, u))   # 원본: 소실 폴백 EMA 는 과도 스파이크 완화를 위해 ±0.30 클램프
            self._steer_hold = hold if self._steer_hold is None else 0.3 * hold + 0.7 * self._steer_hold
            cmd.linear.x = float(max(g('slow_speed'), g('linear_speed') * (1.0 - g('curve_slow') * abs(r.curvature))))
        elif (self._steer_hold is not None and self._last_found is not None and not lost_far
              and (g('lost_stop_s') <= 0 or now - self._last_found < g('lost_stop_s'))):
            if g('lost_hold_mode') != 'hold':
                self._steer_hold += (0.0 - self._steer_hold) * min(1.0, dt / 3.0)   # 원본: 3 s 시정수로 직진 쪽으로
            u = self._steer_hold
            cmd.linear.x = float(g('slow_speed'))
        elif lost_far:
            u = 0.0
            self.pid.reset()
            if not self._lost_logged:
                self._lost_logged = True
                self.get_logger().warn(f'차선을 잃은 뒤 {self._lost_m:.2f} m 감 (lost_stop_m {g("lost_stop_m"):.2f}) — 정지, 차선이 다시 보이면 출발')
        else:
            u = 0.0
            self.pid.reset()
            self._steer_hold = None
        # 미션 속도 상한 (M5 서행 등). 각속도는 아래 식에서 v 에 비례하므로 곡률은 그대로 유지된다
        fresh = self._cap_stamp is not None and now - self._cap_stamp < g('lane_cmd_timeout_s')
        target = self._cap_target if (fresh and self._cap_target is not None) else float(g('linear_speed'))
        step = float(g('speed_ramp')) * dt
        self._cap += max(-step, min(step, target - self._cap))
        cmd.linear.x = float(min(cmd.linear.x, self._cap))
        if fresh and self._stop:           # 정지선 정지 (M3·M1): 바로 0
            cmd.linear.x = 0.0
        # offset + = 차선 중심이 오른쪽 → 오른쪽으로 (각속도 −). 원본 steer_scale −1 과 같은 규약
        w = -g('angular_scale') * u * cmd.linear.x / max(1e-3, g('linear_speed'))
        cmd.angular.z = float(np.clip(w, -g('max_angular'), g('max_angular')))
        if not r.found:
            self._lost_m += abs(cmd.linear.x) * dt
        if not g('enable') or (g('wait_mission') and self._cap_stamp is None):
            if g('enable') and not self._waiting_logged:
                self._waiting_logged = True
                self.get_logger().info('미션 노드 지시 (/mission/lane_cmd) 를 기다림 — 받으면 출발')
            cmd = Twist()
        self._last_cmd = cmd
        self.cmd_pub.publish(cmd)

    def watchdog(self):
        if self._last_img is not None and self.clock_s() - self._last_img > self.get_parameter('image_timeout_s').value:
            self._last_cmd = Twist()
            self.cmd_pub.publish(self._last_cmd)


def main():
    # rclpy 기본 SIGINT 처리기는 컨텍스트를 먼저 닫아 끌 때 정지 명령을 못 보낸다 → 끄고 KeyboardInterrupt 로 받는다
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl+C 가 터미널과 launch 양쪽에서 와서 두 번째가 정리 도중 KeyboardInterrupt 를 낸다 → 정리하는 동안은 무시
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.cmd_pub.publish(Twist())   # 끌 때 정지
        node.destroy_node()
        rclpy.try_shutdown()
