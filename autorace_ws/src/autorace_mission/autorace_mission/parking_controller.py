"""M4 T자 주차·탈출 (parking_controller) — 디퍼렌셜(스키드) LIMO. 돌 때 각도는 IMU (자이로 적분), 곧게 갈 때 거리는 휠 오도메트리.

  sub  /parking/start (std_msgs/Empty, mission_manager 가 M1 뒤에) — standalone:=true 면 안 기다리고 바로 시작
  sub  /scan, /odom, /imu, /lane/state (정지선 거리 — 칸 끝선)
  pub  /parking/cmd_vel (Twist) → cmd_vel_mux (/parking/state 가 진행 중이면 이 명령을 고르고 라이다 범퍼도 켠다)
  pub  /parking/state (String, 10 Hz): IDLE | WAIT | APPROACH | ALIGN | ROTATE_IN | MEASURE | ENTER | CHECK | HOLD | EXIT |
                       ROTATE_OUT | DONE | FAIL
  pub  /parking/done (std_msgs/Empty) — 빠져나와 처음 방향으로 돈 뒤
  pub  /parking/result (String) — 주차 끝에 네 모서리 여유·회전 오차·시간 (로그와 같은 내용)
  pub  /parking/markers (visualization_msgs/MarkerArray, odom) — 빈 칸, 목표 자세, 차 사각형, 네 모서리 여유

단계 (bay_side 쪽 = 주차 칸 줄, 기본 오른쪽):
  WAIT       (/parking/start 뒤, 통합 모드) 차선 추종이 몰고 가는 동안 앞옆에 주차 LIMO 가 보이면 APPROACH (mux 는 차선 명령을 고른다)
  APPROACH   approach_speed 로 IMU 방향 유지 + 주차 LIMO 줄과 옆 거리 유지. 차 중심 옆 라이다 (side_probe) 가 가까움 → 멂 → 가까움
             (LIMO → 빈 칸 → LIMO) 이면 빈 폭을 확인하고 칸 가운데 (진행 방향 거리) 와 깊이 (LIMO 끝 + LIMO 반 길이) 를 odom 에 저장
  ALIGN      차 중심을 칸 가운데선으로 (앞뒤로), 제자리 회전 반경 (0.194 m) + 여유를 쓸기 검사로 확인
  ROTATE_IN  IMU 로 제자리 90° (±rot_tol_deg, 목표 근처 감속)
  MEASURE    돌면서 위치가 밀리므로 라이다로 다시 잼 — 양옆 LIMO 안쪽 면의 가운데 (옆 오차) 와 면 방향. 옆 오차가 크면 돌아 나와 ALIGN 부터
  ENTER      enter_speed 로 전진, 양옆 거리 같게 P 제어 + 칸 축 방향, 각속도 enter_max_w 로 제한. 칸 깊이 가운데에서 정지
             (끝선: lane_follower 정지선 거리 → odom 으로 이어 감 (M3·M1 과 같은 방식, stopline.StopLine) / 못 보면 LIMO 로 잰 깊이)
  CHECK      네 모서리와 칸 선 여유 확인 → 앞뒤가 모자라면 앞뒤 보정 (max_fix 번)
  HOLD       hold_s 정지
  EXIT       들어간 거리만큼 후진 (라이다로 양옆 유지)
  ROTATE_OUT IMU 로 반대 90° → DONE, /parking/done
막힘: 움직이라는데 stuck_s 동안 못 움직이면 (칸 안이면 먼저 빠져나와) ALIGN 부터 다시 (max_retry 번, 넘으면 FAIL 정지).
"""
import math
import signal

import numpy as np
import rclpy
from autorace_msgs.msg import LaneState
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import ColorRGBA, Empty, String
from visualization_msgs.msg import Marker, MarkerArray

from autorace_mission import avoid
from autorace_mission.avoid import StuckMonitor
from autorace_mission.stopline import StopLine

HL, HW = avoid.CAR_HALF_L, avoid.CAR_HALF_W
ROT_R = math.hypot(HL, HW)                 # 제자리 회전 반경 0.194 m


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller')
        p = self.declare_parameter
        p('standalone', False)             # true = /parking/start 없이 바로 APPROACH (주차 구역 앞에서 시작)
        p('bay_side', 'right')             # 주차 칸 줄이 진행 방향 어느 쪽
        p('bay_w', 0.50)                   # 칸 안쪽 폭·깊이 [m] (시뮬 도면 500 × 600 — 선 안쪽)
        p('bay_d', 0.60)
        p('approach_speed', 0.10)
        p('wait_detect_m', 0.9)            # WAIT: 앞옆 (x 0.1~이만큼) 에 주차 LIMO 점이 보이면 APPROACH
        p('wait_min_pts', 10)              # 주차 LIMO 로 볼 최소 라이다 점 수
        p('wait_min_len_m', 0.15)          # 그 점들의 앞뒤 길이 (LIMO 옆면 0.32 m)
        p('wait_confirm', 3)               # 이만큼 연속 스캔
        p('row_keep_k', 1.5)               # 주차 LIMO 줄과 옆 거리 유지 [rad/m] (방향 보정, ±row_keep_max_deg)
        p('row_keep_max_deg', 6.0)
        p('side_probe_dx', 0.04)           # 차 중심 옆 라이다 띠 (진행 방향 ±)
        p('side_near_m', 0.75)             # 옆 이 안 = 가까움 (LIMO)
        p('side_far_m', 0.95)              #   이 밖·없음 = 멂 (빈 칸)
        p('limo_min_len_m', 0.20)          # 옆 '가까움' 구간이 진행 방향으로 이보다 짧으면 LIMO 아님 (표지판 기둥 등). LIMO 옆에서 보이는 폭 0.22  # 0.12 → 0.20 (2D 시뮬 11-09 g2: 기둥이 0.129 m 로 재져 LIMO 로 착각. 기둥 실차 0.06–0.08 · 시뮬 ~0.10, LIMO 0.29–0.34)
        p('gap_min_m', 0.45)               # 빈 칸 폭 (LIMO 옆면 사이) 최소 — 차 폭 0.22 + 여유
        p('gap_max_m', 1.30)
        p('limo_half_l', 0.161)            # 이웃 LIMO 반 길이 (끝면 → 칸 깊이 가운데)
        p('align_tol_m', 0.008)
        p('align_speed', 0.06)
        p('rot_margin_m', 0.03)            # 제자리 회전 쓸기 검사 여유
        p('rot_tol_deg', 1.5)
        p('rot_max_w', 0.8)
        p('rot_min_w', 0.2)
        p('rot_aim_deg', 0.7)              # 회전 멈추는 목표 (예측 오차가 이 안이면 0 명령). 끝 인정은 rot_tol_in/out_deg
        p('rot_tol_in_deg', 2.5)           # 들어갈 때 허용 오차 (칸 여유 각 ~14 cm, 3° = 모서리 ~0.9 cm, ENTER 가 방향도 맞춤)
        p('rot_tol_out_deg', 4.0)          # 나올 때 (차선 추종이 이어받음)
        p('rot_kick_s', 0.15)              # 명령은 도는데 |wz| < 0.05 가 이만큼 → 각속도 rot_kick_step 씩 올림 (현수막: 정지마찰 > 동마찰)
        p('rot_kick_step', 0.1)
        p('rot_timeout_s', 8.0)            # 회전 단계 시간 한도 → 오차가 rot_accept_deg 안이면 끝으로 침 (0 ↔ 최소 각속도 반복 라이브락 방지)
        p('rot_accept_deg', 4.0)
        p('rot_contact_s', 1.5)            # 최대 각속도로도 이만큼 안 돌면 접촉으로 보고 멈춤 → 막힘 처리
        p('align_head_gate_deg', 8.0)      # ALIGN: 처음 방향과 이보다 틀어져 있으면 앞뒤 이동 전에 제자리 회전부터
        p('align_min_speed', 0.04)
        p('scan_t_offset', 0.0)            # 스캔 시각 = stamp + 이것 × scan_time (시뮬 스캔은 순간 → 0. 실차는 드라이버가 시작/끝 중 무엇을 찍는지 보고 +0.5/−0.5)         # ALIGN 최소 속도 (0.02 는 막힘 감시에도 안 걸리고 현수막에서 안 움직임)
        p('rot_slow_deg', 25.0)            # 목표 이만큼 안에서 감속
        p('rot_lag_s', 0.45)               # 각속도 명령 → 실제 회전 지연 (시뮬 구동 PI ~0.45 s, 실차는 잴 것)
        p('measure_s', 0.5)                # 돈 뒤 라이다를 모으는 시간
        p('measure_tol_m', 0.05)           # 다시 잰 옆 오차가 이보다 크면 돌아 나와 ALIGN 부터
        p('enter_speed', 0.09)
        p('enter_min_speed', 0.03)
        p('enter_kp', 0.5)                 # 남은 깊이 비례 감속 [1/s]
        p('enter_k_lat', 2.0)              # 양옆 거리 차 → 각속도 [rad/s/m]
        p('enter_k_head', 1.5)
        p('enter_max_w', 0.25)             # 칸 안 각속도 한도
        p('cam_x', 0.084)                  # base_link → 카메라 [m] (정지선 거리 기준)
        p('line_match_m', 0.10)
        p('depth_tol_m', 0.02)             # 깊이 가운데에서 이만큼 안이면 정지 완료
        p('corner_min_m', 0.03)            # 모서리 → 선 여유가 이보다 작으면 앞뒤 보정
        p('max_fix', 2)
        p('hold_s', 2.5)
        p('park_exit', True)               # 주차 뒤 빠져나오기 — 빠져나와 차로로 나오기까지가 M4 (11-09 사용자). false = 칸 안에서 끝
        # 아커만 (앞바퀴 조향 — 제자리 회전 없음): 칸 가운데보다 반경만큼 앞에서 전진 원호로 90° 들어가고 (ARC_IN),
        # 들어간 만큼 후진한 뒤 후진 원호로 차로 가운데로 나온다 (ARC_OUT). 원호 중에는 남은 거리로 반경을 다시 계산
        p('drive_mode', 'diff')            # diff | ackermann
        p('ack_min_radius_m', 0.47)        # 최소 회전 반경 (limo_base 가 자르는 값)
        p('ack_park_radius_m', 0.50)       # 들어가기 원호 계획 반경 (칸 가운데 − 이만큼 앞에서 꺾기 시작)
        p('ack_arc_speed', 0.08)
        p('ack_arc_end_deg', 2.0)          # 90° 에서 이만큼 안이면 원호 끝
        p('exit_speed', 0.09)
        p('stuck_s', 1.0)
        p('max_retry', 3)
        p('laser_x', 0.103)
        p('scan_topic', '/scan')
        p('odom_topic', '/odom')
        p('imu_topic', '/imu')
        p('lane_state_topic', '/lane/state')
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.sgn = -1.0 if g('bay_side') == 'right' else 1.0
        self._wait_hits = 0
        self.state = 'IDLE'
        self.pose = None                   # odom (x, y, yaw)
        self.speed = 0.0
        self.odom_hist = []
        self.imu = None                    # (t, 적분 yaw, wz)
        self.imu_off = None                # odom yaw − 적분 yaw (시작할 때 맞춤)
        self.scan = None                   # (t, x[], y[]) base_link
        self.line = StopLine(g('line_match_m'))
        self.reset_plan()
        self.cmd_pub = self.create_publisher(Twist, '/parking/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/parking/state', 10)
        self.done_pub = self.create_publisher(Empty, '/parking/done', 10)
        self.result_pub = self.create_publisher(String, '/parking/result', 10)
        self.mk_pub = self.create_publisher(MarkerArray, '/parking/markers', 2)
        self.create_subscription(Empty, '/parking/start', self.on_start, 10)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 50)
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaneState, g('lane_state_topic'), self.on_lane, 10)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.2, self.markers)
        self.get_logger().info(f"parking_controller: {'단독 (바로 시작)' if g('standalone') else '/parking/start 기다림'}, "
                               f"칸 {g('bay_w')} × {g('bay_d')} m, {g('bay_side')} 쪽")

    def reset_plan(self):
        self.t0 = None
        self.h0 = None                     # 처음 진행 방향 (IMU)
        self.prof = {'phase': 'near1', 'far0': None, 'far1': None, 'near_d': []}
        self.bay = None                    # {'c': (x, y) odom, 'axis': 칸 안쪽 방향 yaw}
        self.row_ref = None
        self.rot_target = None
        self.rot_err = {}                  # 회전 오차 [deg] 기록
        self.enter_from = None
        self.enter_len = 0.0
        self.meas = []
        self.fix_n = 0
        self.retry = 0
        self.mon = StuckMonitor(self.get_parameter('stuck_s').value, 0.01, 2.0, 0.015, 0.1)
        self.phase_t = None
        self.after_rot = None
        self.exit_to = None
        self.result = ''
        self.rot_w = 0.0
        self.rot_still = None
        self.rot_boost, self.rot_nomove, self.rot_full, self.rot_contact = 0.0, None, None, False
        self.rot_t0 = None                 # 이번 회전 시작 시각 (시간 한도)
        self.rot_settled, self.rot_coast = False, None
        self.align_rot = False             # ALIGN 방향 게이트 안 (래치)
        self.scan_t = None                 # 마지막 스캔의 메시지 시각 (칸 폭 잴 때 그 시각 위치)

    # ---------------------------------------------------------------- 입력
    def on_start(self, _msg):
        if self.state in ('IDLE', 'DONE', 'FAIL'):
            self.reset_plan()
            self.set_state('WAIT', '/parking/start — 주차 구역 (앞옆 LIMO) 을 기다림')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.pose is not None and math.dist(self.pose[:2], (p.x, p.y)) > 0.5:
            self.odom_hist.clear()         # 시뮬 리셋
        self.pose = (p.x, p.y, yaw)
        self.speed = msg.twist.twist.linear.x
        self.odom_hist.append((stamp_s(msg.header), p.x, p.y, yaw))
        if len(self.odom_hist) > 400:
            del self.odom_hist[:100]

    def pose_at(self, t):
        h = self.odom_hist
        if not h:
            return self.pose
        e = min(h[-60:], key=lambda e: abs(e[0] - t))
        return e[1:]

    def on_imu(self, msg):
        t, wz = stamp_s(msg.header), msg.angular_velocity.z
        if self.imu is None:
            self.imu = (t, 0.0, wz)
            return
        t0, yaw, w0 = self.imu
        dt = t - t0
        if 0.0 < dt < 0.5:
            yaw += 0.5 * (w0 + wz) * dt
        self.imu = (t, yaw, wz)

    def heading(self):
        """IMU 방향 (처음 odom yaw 에 맞춘 자이로 적분). IMU 가 없으면 odom yaw."""
        if self.imu is None:
            return self.pose[2]
        if self.imu_off is None:
            self.imu_off = wrap(self.pose[2] - self.imu[1])
        return wrap(self.imu[1] + self.imu_off)

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, np.float32)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        x = r[ok] * np.cos(a[ok]) + self.get_parameter('laser_x').value
        y = r[ok] * np.sin(a[ok])
        body = (np.abs(x) < HL + 0.005) & (np.abs(y) < HW + 0.005)
        self.scan = (self.get_clock().now().nanoseconds * 1e-9, x[~body], y[~body])
        self.scan_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + self.get_parameter('scan_t_offset').value * msg.scan_time

    def on_lane(self, msg):
        if self.state in ('ENTER', 'CHECK') and msg.stop_line_dist_m > 0 and self.bay is not None:
            pose = self.pose_at(stamp_s(msg.header))
            first = self.line.pt is None
            if self.line.update(pose, msg.stop_line_dist_m + self.get_parameter('cam_x').value) and first:
                self.get_logger().info(f'[M4] 칸 끝선 보임: 차 중심에서 {msg.stop_line_dist_m + self.get_parameter("cam_x").value:.3f} m '
                                       f'(덮은 비율 {msg.stop_line_cov:.2f})')

    # ---------------------------------------------------------------- 도움
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s, why=''):
        if s != self.state:
            self.get_logger().info(f'[M4] {self.state} → {s}' + (f' ({why})' if why else ''))
            self.state = s
            self.phase_t = self.now()
            self.mon.reset()
            self.rot_reset()
            self.align_rot = False

    def pub(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.cmd_pub.publish(m)

    def pts(self):
        if self.scan is None or self.now() - self.scan[0] > 0.5:
            return None
        return self.scan[1], self.scan[2]

    def side_dist(self):
        """차 중심 옆 (bay_side 쪽) 라이다 띠의 가장 가까운 거리 [m] 또는 inf."""
        pt = self.pts()
        if pt is None:
            return None
        x, y = pt
        if self.h0 is not None:
            # 처음 방향 h0 기준으로 돌려서 띠를 고름 — 차가 몇 ° 꺾여 있으면 0.37 m 옆 점이 0.37·ψ 만큼 앞뒤로 보여
            # LIMO 끝 (칸 경계) 이 2~3 cm 늦게 잡혔다 (다른 세션 검토 11-06: 줄 맞추기로 LIMO 끝에서 −3.5° 꺾임)
            p = wrap(self.heading() - self.h0)
            c, sn = math.cos(p), math.sin(p)
            x, y = x * c - y * sn, x * sn + y * c
        sel = (np.abs(x) < self.get_parameter('side_probe_dx').value) & (self.sgn * y > HW)
        return float((self.sgn * y[sel]).min()) if sel.any() else math.inf

    def along(self, pos=None):
        """처음 방향 h0 축으로 잰 거리 (odom)."""
        x, y = (pos or self.pose)[:2]
        return (x - self.start_xy[0]) * math.cos(self.h0) + (y - self.start_xy[1]) * math.sin(self.h0)

    def lateral(self, pos=None):
        x, y = (pos or self.pose)[:2]
        return -(x - self.start_xy[0]) * math.sin(self.h0) + (y - self.start_xy[1]) * math.cos(self.h0)

    def bay_frame(self, pos=None):
        """칸 좌표: (깊이 방향 — 칸 안쪽 +, 옆 — 칸 축 왼쪽 +, 칸 축 대비 방향)."""
        x, y = (pos or self.pose)[:2]
        cx, cy = self.bay['c']
        a = self.bay['axis']
        dx, dy = x - cx, y - cy
        return dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a), wrap(self.heading() - a)

    def corners_margin(self):
        """네 모서리 → 칸 선 (안쪽 경계) 여유 [m] (앞왼, 앞오, 뒤오, 뒤왼). 음수 = 선 밖."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        d, l, h = self.bay_frame()
        out = []
        for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
            cd = d + sx * HL * math.cos(h) - sy * HW * math.sin(h)
            cl = l + sx * HL * math.sin(h) + sy * HW * math.cos(h)
            out.append(min(g('bay_d') / 2 - abs(cd), g('bay_w') / 2 - abs(cl)))
        return out

    def faces(self, x0, x1):
        """차 좌표 x0~x1 띠에서 양옆 가장 가까운 점 (왼쪽 y, 오른쪽 y) — 칸 양옆 LIMO 안쪽 면."""
        pt = self.pts()
        if pt is None:
            return None, None
        x, y = pt
        band = (x > x0) & (x < x1) & (np.abs(y) < 0.6)
        lp, rp = y[band & (y > HW)], y[band & (y < -HW)]
        return (float(lp.min()) if lp.size else None), (float(rp.max()) if rp.size else None)

    def rot_reset(self, now=None):
        """회전 보조 상태 초기화 (새 회전 시작)."""
        self.rot_boost, self.rot_nomove, self.rot_still, self.rot_full, self.rot_contact = 0.0, None, None, None, False
        self.rot_settled, self.rot_coast = False, None
        self.rot_t0 = now if now is not None else self.now()

    def rotate(self, target, now, then, tol_deg=None):
        """IMU 로 target 까지 제자리 회전. 끝나면 True.
        현수막 트랙 (11-06 사용자): 천이 구겨져 느린 제자리 회전이 걸림 → ① 명령은 도는데 안 돌면 각속도를 조금씩 올림 (킥),
        ② 단계 시간 한도 (rot_timeout_s) 넘으면 오차가 rot_accept_deg 안일 때 끝으로 침 — 예측 정지 (0) ↔ 최소 각속도가 번갈아
        나오면 막힘 감시가 매번 리셋돼 끝나지도 막히지도 않음 (다른 세션 검토 11-06)."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        err = wrap(target - self.heading())
        wz = self.imu[2] if self.imu is not None else 0.0
        tol = math.radians(tol_deg if tol_deg is not None else g('rot_tol_deg'))
        if now - self.rot_t0 > g('rot_timeout_s') and abs(err) < math.radians(g('rot_accept_deg')):
            self.pub(0.0, 0.0)
            self.rot_w = 0.0
            if abs(wz) < 0.05:
                self.get_logger().warn(f'[M4] 회전 {g("rot_timeout_s"):.0f} s 넘음 — 오차 {math.degrees(err):+.1f}° 로 끝냄')
                return True
            return False
        # 멈추는 목표 (rot_aim_deg, 작게) 와 끝 인정 (tol) 을 나눔: tol 경계에서 바로 멈추면 한쪽에서 다가오니 늘 경계 안쪽에
        # 서서 '덜 돎' (시뮬 참값: in +0.9~+2.2°, out −2.8~−4.0° — 다른 세션 검토 11-06)
        aim = math.radians(g('rot_aim_deg'))
        # 구동 지연 (rot_lag_s) 동안 지금 각속도로 더 돌 것을 빼고 본 오차 — 0 을 명령해도 그만큼 더 돈다
        pred = err - wz * g('rot_lag_s')
        if abs(pred) < aim or pred * err < 0 or (self.rot_settled and abs(err) < tol):
            # 멈춤: 흔들림이 가라앉을 때까지 (0.3 s 동안 |wz| < 0.05) 기다린 뒤 tol 안이면 끝. 한 번 aim 으로 멈춘 뒤 (rot_settled) 엔
            # tol 안이면 더 밀지 않음 (안 그러면 실질 허용 오차가 aim 이라 짧은 펄스 반복 — 다른 세션 검토 11-06)
            if self.rot_coast is None and abs(wz) > 0.1:
                self.rot_coast = (self.heading(), wz)       # 0 명령 순간 — 정지 뒤 더 돈 각도로 실제 관성 지연을 잼
            self.rot_settled = True        # 한 번 끊었으면 관성으로 더 도는 동안·정지 확인 동안 tol 안이면 다시 밀지 않음
            self.pub(0.0, 0.0)
            self.rot_w = 0.0
            self.rot_nomove = None
            self.rot_boost *= 0.5          # 다시 출발할 때 전 킥을 그대로 쓰면 평평한 바닥에서 지나침
            if abs(wz) >= 0.05:
                self.rot_still = None
                return False
            self.rot_still = self.rot_still or now
            if now - self.rot_still < 0.3:
                return False
            if self.rot_coast is not None:
                h1, w1 = self.rot_coast
                dh = wrap(self.heading() - h1)
                self.get_logger().info(f'[M4] 회전 관성: wz {w1:+.2f} → 0 명령 뒤 {math.degrees(dh):+.1f}° 더 돎 = 실제 지연 '
                                       f'{dh / w1:.2f} s (설정 rot_lag_s {g("rot_lag_s"):.2f})')
                self.rot_coast = None
            return abs(err) < tol
        self.rot_still = None
        self.rot_settled = False
        self.rot_coast = None
        # 오차 비례 (rot_slow_deg 에서 rot_max_w), 최소 rot_min_w (스키드는 작은 각속도로는 안 돈다) + 킥
        mag = max(g('rot_min_w'), min(g('rot_max_w'), g('rot_max_w') * abs(pred) / math.radians(g('rot_slow_deg'))))
        if abs(wz) > 0.15:
            self.rot_boost, self.rot_nomove = 0.0, None
        elif abs(wz) < 0.05:
            if self.rot_nomove is None:
                self.rot_nomove = now + g('rot_lag_s')          # 첫 킥은 구동 지연이 지난 뒤부터 (그 전엔 원래 안 돎)
            elif now - self.rot_nomove > g('rot_kick_s'):
                self.rot_boost = min(g('rot_max_w'), self.rot_boost + g('rot_kick_step'))
                self.rot_nomove = now
                self.get_logger().info(f'[M4] 킥: 안 돎 → 각속도 +{self.rot_boost:.2f} (오차 {math.degrees(err):+.1f}°, {self.state})')
        if mag + self.rot_boost >= g('rot_max_w') and abs(wz) < 0.05:
            # 최대 각속도인데도 안 돎 = 마찰이 아니라 옆 LIMO 에 닿은 것 — 계속 밀지 않는다
            self.rot_full = self.rot_full or now + g('rot_lag_s')   # 출발 직후 구동 지연은 빼고 셈
            if now - self.rot_full > g('rot_contact_s'):
                self.pub(0.0, 0.0)
                self.rot_w = 0.0
                if not self.rot_contact:
                    self.get_logger().warn(f'[M4] 최대 각속도로 {g("rot_contact_s"):.1f} s 안 돎 — 접촉으로 보고 멈춤 (오차 {math.degrees(err):+.1f}°)')
                self.rot_contact = True
                return False
        else:
            self.rot_full = None
        mag = min(max(g('rot_max_w'), g('rot_min_w')), mag + self.rot_boost)
        w = math.copysign(mag, pred)
        self.pub(0.0, w)
        self.rot_w = w
        return False

    def rotate_stuck(self, now, final):
        """회전 단계 막힘: 정렬부터 다시 하지 않는다 (60~80° 돈 상태에서 ALIGN 은 칸 축 쪽으로 감, ROTATE_OUT 이면 다시 주차함).
        시간 한도 두 배 넘거나 접촉 (최대 각속도로도 안 돎) 이면 — 나올 때 (final) 는 그대로 끝 (차선 추종이 이어받음),
        들어갈 때는 StuckMonitor 를 거치지 않고 바로 재시도 (명령이 0 ↔ w 로 오가면 감시가 리셋돼 안 걸림)."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        why = ('최대 각속도로도 안 돎 (접촉)' if self.rot_contact
               else f'{2 * g("rot_timeout_s"):.0f} s 넘게 안 끝남' if now - self.rot_t0 >= 2 * g('rot_timeout_s') else None)
        if why is None:
            return False
        if final:
            return True
        self.retry += 1
        self.pub(0.0, 0.0)
        if self.retry > g('max_retry'):
            self.set_state('FAIL', f'회전 막힘: {why} — 재시도 {self.retry - 1} 번 넘음, 멈춤')
            self.get_logger().error('[M4] 주차 실패 — 멈춤')
            return True
        self.after_rot = 'ALIGN'
        self.rot_target = self.h0
        self.set_state('ROTATE_OUT_RETRY', f'회전 막힘: {why} → 처음 방향으로 돌아서 다시 정렬 ({self.retry} 번째)')
        return True

    def stuck(self, now, v, w):
        if self.phase_t is not None and now - self.phase_t < 2.0 and self.state == 'APPROACH':
            return False                   # 출발 (리셋) 직후 — 차가 아직 안 움직일 수 있다
        why = self.mon.update(now, self.pose[:2], self.heading(), v, w)
        if why is None:
            return False
        self.retry += 1
        if self.retry > self.get_parameter('max_retry').value:
            self.pub(0.0, 0.0)
            self.set_state('FAIL', f'막힘 {why} — 재시도 {self.retry - 1} 번 넘음, 멈춤')
            self.get_logger().error('[M4] 주차 실패 — 멈춤')
            return True
        if self.state in ('ENTER', 'CHECK', 'HOLD', 'FIX'):
            self.exit_to = 'ALIGN'
            self.set_state('EXIT', f'막힘: {why} → 빠져나와 정렬부터 ({self.retry} 번째)')
        elif self.bay is None:
            self.set_state('APPROACH', f'막힘: {why} → 칸 찾기 계속 ({self.retry} 번째)')
            self.phase_t = now - 2.0
        else:
            self.set_state('ALIGN', f'막힘: {why} → 정렬부터 ({self.retry} 번째)')
        self.pub(0.0, 0.0)
        return True

    # ---------------------------------------------------------------- 주기
    def tick(self):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.state_pub.publish(String(data=self.state))
        if self.pose is None:
            return
        now = self.now()
        if self.state == 'IDLE' and g('standalone') and self.imu is not None and self.scan is not None:
            self.reset_plan()
            self.set_state('APPROACH', '단독 모드 — 바로 시작')
        if self.state in ('IDLE', 'DONE', 'FAIL'):
            return
        if self.h0 is None:
            self.h0 = self.heading()
            self.start_xy = self.pose[:2]
            self.t0 = now
        s = self.state
        if s == 'WAIT':
            pt = self.pts()
            if pt is not None:
                x, y = pt
                sel = (x > 0.1) & (x < g('wait_detect_m')) & (self.sgn * y > 0.25) & (self.sgn * y < g('side_near_m'))
                n = int(sel.sum())
                ext = float(x[sel].max() - x[sel].min()) if n else 0.0
                # LIMO 옆면 (길이 0.32 m) 만큼 길게 이어진 점들만 — 트랙 옆 표지판 받침대 (점 3 개) 를 주차 LIMO 로 보고
                # 차선 추종을 놓아 코너에서 직진해 나간 일 (실차 11-05)
                ok = n >= g('wait_min_pts') and ext >= g('wait_min_len_m')
                self._wait_hits = self._wait_hits + 1 if ok else 0
                if self._wait_hits >= g('wait_confirm'):
                    self.h0, self.start_xy, self.t0 = self.heading(), self.pose[:2], now
                    self.set_state('APPROACH', f'앞옆에 주차 LIMO (점 {n} 개, 앞뒤 {ext:.2f} m, {self._wait_hits} 번 연속)')
            return
        if s == 'APPROACH':
            self.do_approach(now)
        elif s == 'ALIGN':
            self.do_align(now)
        elif s == 'ROTATE_IN':
            if self.rotate(self.rot_target, now, 'MEASURE', g('rot_tol_in_deg')):
                self.rot_err['in'] = math.degrees(wrap(self.heading() - self.rot_target))
                self.set_state('MEASURE', f'90° 회전 끝 (IMU 오차 {self.rot_err["in"]:+.1f}°)')
                self.meas = []
            elif self.rotate_stuck(now, False):
                return
        elif s == 'ARC_IN':
            self.do_arc_in(now)
        elif s == 'ARC_OUT':
            self.do_arc_out(now)
        elif s == 'MEASURE':
            self.do_measure(now)
        elif s == 'ENTER':
            self.do_enter(now)
        elif s == 'CHECK':
            self.do_check(now)
        elif s == 'FIX':
            self.do_fix(now)
        elif s == 'ROTATE_OUT_RETRY':
            if self.rotate(self.rot_target, now, self.after_rot, g('rot_tol_out_deg')):
                self.set_state(self.after_rot or 'ALIGN', '처음 방향으로 돌아섬 → 다시 정렬')
            elif self.rotate_stuck(now, True):
                self.set_state(self.after_rot or 'ALIGN', f'돌아서기도 안 끝남 (오차 {math.degrees(wrap(self.rot_target - self.heading())):+.1f}°) → 그대로 정렬')
        elif s == 'HOLD':
            self.pub(0.0, 0.0)
            if now - self.phase_t >= g('hold_s') and not g('park_exit'):
                self.result += f', 전체 {now - self.t0:.1f} s (빠져나오지 않음 — park_exit false)'
                self.set_state('DONE', self.result)
                self.result_pub.publish(String(data=self.result))
                self.done_pub.publish(Empty())
                self.pub(0.0, 0.0)
            elif now - self.phase_t >= g('hold_s'):
                self.exit_to = 'ROTATE_OUT'
                self.set_state('EXIT', f'{g("hold_s"):.1f} s 정지 끝 → 들어간 거리 {self.enter_len:.2f} m 후진')
        elif s == 'EXIT':
            self.do_exit(now)
        elif s == 'ROTATE_OUT':
            out_done = self.rotate(self.h0, now, 'DONE', g('rot_tol_out_deg'))
            if not out_done and self.rotate_stuck(now, True):
                self.get_logger().warn(f'[M4] 나올 때 회전이 {2 * g("rot_timeout_s"):.0f} s 넘게 안 끝남 — 그대로 끝 (차선 추종이 이어받음)')
                out_done = True
            if out_done:
                self.rot_err['out'] = math.degrees(wrap(self.heading() - self.h0))
                if abs(self.rot_err['out']) > 15.0:
                    self.get_logger().warn(f'[M4] 나올 때 방향 오차 {self.rot_err["out"]:+.1f}° 로 끝냄 — 차선 추종이 못 잡을 수 있음')
                self.result += f', 나올 때 90° 회전 오차 (IMU) {self.rot_err["out"]:+.1f}°, 전체 {now - self.t0:.1f} s'
                self.set_state('DONE', self.result)
                self.result_pub.publish(String(data=self.result))
                self.done_pub.publish(Empty())
                self.pub(0.0, 0.0)

    def do_approach(self, now):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        d = self.side_dist()
        pr = self.prof
        ps = self.pose_at(self.scan_t) if self.scan_t is not None else None
        a_now = self.along(ps) if ps is not None else self.along()   # 그 스캔을 찍은 때의 위치
        # 가까움↔멂 경계는 실제로 지난 뒤 첫 스캔에서 잡힘 (평균 반 주기 늦음 → 칸 가운데가 진행 방향으로 v × 반 주기 밀림,
        # 0.2 m/s 에서 ~1.3 cm). 경계 위치 = 직전 스캔과 이번 스캔의 가운데 (다른 세션 검토 11-06)
        new_scan = pr.get('scan_t') != self.scan_t
        a = 0.5 * (pr['a_prev'] + a_now) if pr.get('a_prev') is not None else a_now
        if not new_scan:
            a = pr.get('a_edge', a_now)
        near = d is not None and d < g('side_near_m')
        far = d is None or d > g('side_far_m')
        if pr['phase'] == 'near1' and near:
            pr['phase'], pr['near0'], pr['near_d'] = 'in_near1', a, [d]
        elif pr['phase'] == 'in_near1':
            if near:
                pr['near_d'].append(d)
            elif far:
                ln = a - pr.get('near0', a)
                if ln < g('limo_min_len_m'):
                    # 표지판 기둥·사람 다리처럼 짧은 물체 — LIMO 가 아님 (실차 11-06 full10: P 표지판 기둥과 첫 LIMO 사이에 주차)
                    pr['phase'], pr['near_d'] = 'near1', []
                    self.get_logger().info(f'[M4] 옆 물체 길이 {ln:.3f} m < {g("limo_min_len_m")} — LIMO 아님 (표지판 등), 다음 LIMO 를 찾음')
                else:
                    pr['phase'], pr['far0'] = 'far', a
                    self.get_logger().info(f'[M4] 옆: 가까움 → 멂 (LIMO 끝, 길이 {ln:.3f} m, 진행 {a:.3f} m, '
                                           f'LIMO 끝면까지 {np.median(pr["near_d"]):.3f} m)')
        elif pr['phase'] == 'far' and near:
            self.row_ref = d                   # 줄 맞추기 기준을 LIMO 마다 새로 (첫 LIMO 기준이면 다음 LIMO 끝에서 2° 꺾임)
            w = a - pr['far0']
            if g('gap_min_m') <= w <= g('gap_max_m'):
                pr['far1'] = a
                pr['near_d'].append(d)
                depth = float(np.median(pr['near_d'])) + g('limo_half_l')
                c_along = (pr['far0'] + pr['far1']) / 2
                axis = wrap(self.h0 + self.sgn * math.pi / 2)
                lat0 = self.lateral()
                cx = self.start_xy[0] + c_along * math.cos(self.h0) - lat0 * math.sin(self.h0) + depth * math.cos(axis)
                cy = self.start_xy[1] + c_along * math.sin(self.h0) + lat0 * math.cos(self.h0) + depth * math.sin(axis)
                self.bay = {'c': (cx, cy), 'axis': axis, 'along': c_along, 'gap': w, 'depth': depth}
                self.set_state('ALIGN', f'빈 칸: LIMO 옆면 사이 {w:.3f} m, 칸 가운데 진행 {c_along:.3f} m, 깊이 (LIMO 끝 + 반 길이) '
                                        f'{depth:.3f} m, odom ({cx:.2f}, {cy:.2f}) — 시작 뒤 {now - self.t0:.1f} s')
                self.pub(0.0, 0.0)
                return
            pr['phase'], pr['near0'], pr['near_d'] = 'in_near1', a, [d]
            self.get_logger().info(f'[M4] 틈 {w:.3f} m — 칸 폭이 아님, 다음 틈을 찾음')
        if new_scan:
            pr['a_prev'], pr['scan_t'], pr['a_edge'] = a_now, self.scan_t, a
        if near and self.row_ref is None:
            self.row_ref = d
        # 방향 유지 (IMU) + 주차 LIMO 줄과 옆 거리 유지 (가까움 구간에서만)
        h_cmd = self.h0
        if near and self.row_ref is not None:
            # 기준 = 지금 지나는 LIMO 의 최근 옆 거리 중앙값 (첫 LIMO 기준이면 LIMO 끝면 위치 차이 2.5 cm 를 줄 틀어짐으로 보고
            # 칸 경계 바로 앞에서 2° 꺾었다)
            nd = pr['near_d'][-15:]
            ref = float(np.median(nd)) if len(nd) >= 3 else self.row_ref
            corr = g('row_keep_k') * (d - ref) * self.sgn
            h_cmd = self.h0 + max(-math.radians(g('row_keep_max_deg')), min(math.radians(g('row_keep_max_deg')), corr))
        w = 1.5 * wrap(h_cmd - self.heading())
        v = g('approach_speed')
        if self.stuck(now, v, w):
            return
        self.pub(v, max(-0.4, min(0.4, w)))

    def do_align(self, now):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        herr = wrap(self.h0 - self.heading())
        if not self.align_rot and abs(herr) > math.radians(g('align_head_gate_deg')) and not self.ack():
            # 방향이 많이 틀어진 채 들어옴 (돌아서기가 접촉·시간 한도로 덜 끝남 등) — 앞뒤로 가면 칸 축 (옆 LIMO) 쪽으로 감.
            # 제자리에서 먼저 처음 방향으로, 다 돌 때까지 (래치) (다른 세션 검토 11-06)
            self.align_rot = True
            self.rot_reset(now)
            self.get_logger().info(f'[M4] 정렬 전 방향 맞추기: 처음 방향과 {math.degrees(herr):+.1f}° 틀어짐 → 제자리 회전')
        if self.align_rot:
            if self.rotate(self.h0, now, None, g('rot_tol_out_deg')):
                self.align_rot = False
                self.mon.reset()
                return
            why = ('최대 각속도로도 안 돎 (접촉)' if self.rot_contact
                   else '시간 한도' if now - self.rot_t0 >= 2 * g('rot_timeout_s') else None)
            if why is not None:
                self.retry += 1
                self.pub(0.0, 0.0)
                if self.retry > g('max_retry'):
                    self.set_state('FAIL', f'정렬 전 방향 맞추기 실패: {why} (오차 {math.degrees(herr):+.1f}°) — 재시도 넘음, 멈춤')
                    self.get_logger().error('[M4] 주차 실패 — 멈춤')
                    return
                self.get_logger().warn(f'[M4] 정렬 전 방향 맞추기 막힘: {why} (오차 {math.degrees(herr):+.1f}°) — 다시 ({self.retry} 번째)')
                self.rot_reset(now)
            return
        err = self.bay['along'] - self.along()
        if self.ack():
            err -= g('ack_park_radius_m')    # 아커만: 칸 가운데 − 반경 자리에서 원호 시작
        if abs(err) < g('align_tol_m') and abs(self.speed) < 0.01 and self.ack():
            self.pub(0.0, 0.0)
            self.lat_ref = self.lateral()    # 차로 가운데 (나올 때 여기로)
            self.set_state('ARC_IN', f'원호 시작 자리 (칸 가운데 {g("ack_park_radius_m"):.2f} m 앞, 오차 {err * 100:+.1f} cm) → '
                                     f'전진 원호 {"오른쪽" if self.sgn < 0 else "왼쪽"} 90°')
            return
        if abs(err) < g('align_tol_m') and abs(self.speed) < 0.01:
            # 제자리 회전 반경 + 여유 확인 (라이다 점 쓸기 검사)
            pt = self.pts()
            boxes = avoid.scan_boxes(pt, 0.0, 0.0, 0.0, reach=0.6) if pt is not None else []
            target = wrap(self.h0 + self.sgn * math.pi / 2)
            if not avoid.rot_ok(0.0, 0.0, 0.0, wrap(target - self.heading()), boxes, g('rot_margin_m')):
                self.get_logger().warn(f'[M4] 제자리 회전 여유 부족 (반경 {ROT_R:.3f} m + {g("rot_margin_m")} m) — 그래도 돈다')
            self.rot_target = target
            self.pub(0.0, 0.0)
            self.set_state('ROTATE_IN', f'칸 가운데선 (오차 {err * 100:+.1f} cm) → 제자리 {"오른쪽" if self.sgn < 0 else "왼쪽"} 90°')
            return
        v = max(-g('align_speed'), min(g('align_speed'), 1.2 * err))
        if 0 < abs(v) < g('align_min_speed'):
            v = math.copysign(g('align_min_speed'), v)
        if abs(err) < g('align_tol_m'):
            v = 0.0
        w = 1.5 * wrap(self.h0 - self.heading())
        if self.stuck(now, abs(v), w):
            return
        self.pub(v, max(-0.3, min(0.3, w)))

    def do_measure(self, now):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.pub(0.0, 0.0)
        yl, yr = self.faces(0.15, 0.9)
        if yl is not None and yr is not None:
            self.meas.append((yl, yr))
        if now - self.phase_t < g('measure_s'):
            return
        if len(self.meas) < 3:
            self.get_logger().warn('[M4] 다시 재기: 양옆 LIMO 면이 안 보임 — 처음 잰 칸 가운데로 들어감')
            e = 0.0
        else:
            yl, yr = np.median(np.array(self.meas), axis=0)
            e = float((yl + yr) / 2)       # 칸 가운데가 차 왼쪽으로 e
            # 칸 가운데선을 다시 잰 값으로 (odom)
            a = self.bay['axis']
            lx, ly = -math.sin(a), math.cos(a)        # 칸 축 왼쪽
            cx0, cy0 = self.bay['c']
            dd = (self.pose[0] - cx0) * math.cos(a) + (self.pose[1] - cy0) * math.sin(a)
            self.bay['c'] = (self.pose[0] - dd * math.cos(a) + e * lx, self.pose[1] - dd * math.sin(a) + e * ly)
            self.get_logger().info(f'[M4] 다시 잼: 양옆 LIMO 면 왼 {yl:+.3f} / 오른 {yr:+.3f} m → 칸 가운데 옆 오차 {e * 100:+.1f} cm')
        if abs(e) > g('measure_tol_m') and self.ack():
            self.get_logger().warn(f'[M4] 옆 오차 {e * 100:+.1f} cm > {g("measure_tol_m") * 100:.0f} cm — 아커만은 돌아 나오지 않고 '
                                   f'다시 잰 칸 가운데로 들어가며 조향으로 맞춤')
        elif abs(e) > g('measure_tol_m'):
            self.retry += 1
            if self.retry > g('max_retry'):
                self.set_state('FAIL', f'옆 오차 {e * 100:+.1f} cm — 재시도 넘음')
                return
            # 돌아 나와 진행 방향으로 e 만큼 옮긴 뒤 다시 (칸 축 왼쪽 = 진행 방향 앞뒤로 바뀜)
            self.bay['along'] += e * (1.0 if self.sgn < 0 else -1.0)
            self.after_rot = 'ALIGN'
            self.rot_target = self.h0
            self.set_state('ROTATE_OUT_RETRY', f'옆 오차 {e * 100:+.1f} cm > {g("measure_tol_m") * 100:.0f} cm → 돌아서 다시 정렬')
            return
        self.enter_from = self.pose[:2]
        self.line = StopLine(g('line_match_m'))
        self.set_state('ENTER', f'전진 진입 (칸 깊이 가운데까지 약 {self.bay_frame()[0] * -1:.2f} m)')

    def ack(self):
        return self.get_parameter('drive_mode').value == 'ackermann'

    def turned(self):
        """처음 방향에서 칸 쪽으로 돈 각 [rad] (0 = 진행 방향, 90° = 칸 축)."""
        return self.sgn * wrap(self.heading() - self.h0)

    def do_arc_in(self, now):
        """전진 원호로 90°. 남은 진행 거리 Δu = 칸 가운데 − 지금, 돈 각 θ 에서 칸 축에 닿는 반경 r = Δu / (1 − sin θ)
        (반경을 최소 반경으로 자르면 칸 가운데를 지나침 → ENTER 가 옆 오차를 조향으로 맞춤)."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        th = self.turned()
        if th >= math.radians(90.0 - g('ack_arc_end_deg')):
            self.pub(0.0, 0.0)
            self.rot_err['in'] = math.degrees(th) - 90.0
            du = self.bay['along'] - self.along()
            self.set_state('MEASURE', f'원호 끝 (90° 오차 {self.rot_err["in"]:+.1f}°, 칸 가운데선까지 진행 {du * 100:+.1f} cm)')
            self.meas = []
            return
        du = self.bay['along'] - self.along()
        den = 1.0 - math.sin(max(0.0, th))
        r = du / den if den > 1e-3 and du > 0 else g('ack_min_radius_m')
        r = max(g('ack_min_radius_m'), min(3.0, r))
        v = g('ack_arc_speed')
        w = self.sgn * v / r
        self.arc_log('in', f'전진 원호: 돈 각 {math.degrees(th):.0f}°, 칸 가운데선까지 {du:.2f} m → 반경 {r:.2f} m')
        if self.stuck(now, v, w):
            return
        self.pub(v, w)

    def do_arc_out(self, now):
        """후진 원호로 처음 방향까지. 차로 가운데까지 남은 깊이 d, 돈 각 θ 에서 r = d / (1 − cos θ)."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        th = self.turned()
        d = self.sgn * (self.lateral() - getattr(self, 'lat_ref', 0.0))   # 차로 가운데에서 칸 쪽 깊이 (+)
        if th <= math.radians(g('ack_arc_end_deg') + 1.0):
            self.pub(0.0, 0.0)
            then = getattr(self, 'arc_then', None)
            self.arc_then = None
            if then == 'ALIGN':
                self.set_state('ALIGN', f'차로로 나옴 (방향 {math.degrees(th):+.1f}°, 차로 가운데에서 {d * 100:+.1f} cm) → 정렬부터')
                return
            self.rot_err['out'] = math.degrees(th)
            self.result += f', 나올 때 방향 오차 (IMU) {self.rot_err["out"]:+.1f}°, 차로 가운데에서 {d * 100:+.1f} cm, 전체 {now - self.t0:.1f} s'
            self.set_state('DONE', self.result)
            self.result_pub.publish(String(data=self.result))
            self.done_pub.publish(Empty())
            return
        den = 1.0 - math.cos(th)
        r = d / den if den > 1e-3 and d > 0 else g('ack_min_radius_m')
        r = max(g('ack_min_radius_m'), min(3.0, r))
        v = -g('ack_arc_speed')
        w = -self.sgn * abs(v) / r            # 칸 쪽으로 돈 것을 되돌림
        self.arc_log('out', f'후진 원호: 돈 각 {math.degrees(th):.0f}°, 차로 가운데까지 깊이 {d:.2f} m → 반경 {r:.2f} m')
        if self.stuck(now, abs(v), w):
            return
        self.pub(v, w)

    def arc_log(self, key, msg):
        t = self.now()
        if t - getattr(self, '_arc_log_t', {}).get(key, 0.0) > 1.0:
            self._arc_log_t = {**getattr(self, '_arc_log_t', {}), key: t}
            self.get_logger().info(f'[M4] {msg}')

    def depth_remain(self):
        """칸 깊이 가운데까지 남은 거리 [m] — 끝선을 봤으면 끝선 − 칸 깊이 반, 아니면 LIMO 로 잰 칸 가운데."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        r = self.line.remaining(self.pose)
        if r is not None:
            return r - g('bay_d') / 2, '끝선'
        return -self.bay_frame()[0], 'LIMO'

    def do_enter(self, now):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        remain, src = self.depth_remain()
        if remain <= g('depth_tol_m'):
            self.pub(0.0, 0.0)
            self.enter_len = math.dist(self.enter_from, self.pose[:2])
            self.set_state('CHECK', f'칸 깊이 가운데 ({src} 기준, 남은 {remain * 100:+.1f} cm), 들어간 거리 {self.enter_len:.2f} m')
            return
        yl, yr = self.faces(-0.2, 0.35)
        e = (yl + yr) / 2 if (yl is not None and yr is not None) else -self.bay_frame()[1]
        herr = wrap(self.bay['axis'] - self.heading())
        w = g('enter_k_lat') * e + g('enter_k_head') * herr
        w = max(-g('enter_max_w'), min(g('enter_max_w'), w))
        v = max(g('enter_min_speed'), min(g('enter_speed'), g('enter_kp') * remain))
        # 쓸기 검사 (M7 과 같은 함수): 앞으로 remain 만큼 가며 네 모서리가 라이다 점에 닿지 않는지
        pt = self.pts()
        if pt is not None:
            boxes = avoid.scan_boxes(pt, 0.0, 0.0, 0.0, reach=0.8)
            if avoid.free(0.0, 0.0, 0.0, boxes, max(0.05, remain), 0.01) < min(remain, 0.05):
                self.pub(0.0, w * 0.5)
                if self.stuck(now, v, w):
                    return
                return
        if self.stuck(now, v, w):
            return
        self.pub(v, w)

    def do_check(self, now):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.pub(0.0, 0.0)
        if now - self.phase_t < 0.4:
            return
        m = self.corners_margin()
        d, l, h = self.bay_frame()
        txt = (f'네 모서리 여유 (앞왼 {m[0] * 100:+.1f}, 앞오 {m[1] * 100:+.1f}, 뒤오 {m[2] * 100:+.1f}, 뒤왼 {m[3] * 100:+.1f} cm), '
               f'칸 가운데에서 깊이 {d * 100:+.1f} 옆 {l * 100:+.1f} cm, 방향 {math.degrees(h):+.1f}°, '
               f'들어갈 때 90° 회전 오차 (IMU) {self.rot_err.get("in", float("nan")):+.1f}°')
        front, back = min(m[0], m[1]), min(m[2], m[3])
        if min(m) < g('corner_min_m') and self.fix_n < g('max_fix') and abs(front - back) > 0.01:
            delta = (front - back) / 2       # + = 앞이 넉넉 → 앞으로
            self.fix_n += 1
            self.fix_to = d + delta
            self.get_logger().warn(f'[M4] {txt} → 앞뒤 보정 {delta * 100:+.1f} cm ({self.fix_n}/{g("max_fix")})')
            self.set_state('FIX')
            return
        self.result = f'주차: {txt}, 시작 뒤 {now - self.t0:.1f} s'
        self.get_logger().info(f'[M4] {self.result}')
        self.set_state('HOLD', '모서리 확인 끝')

    def do_fix(self, now):
        d, _, _ = self.bay_frame()
        err = self.fix_to - d
        if abs(err) < 0.005:
            self.pub(0.0, 0.0)
            self.enter_len = math.dist(self.enter_from, self.pose[:2])
            self.set_state('CHECK', '앞뒤 보정 끝')
            return
        v = max(-0.05, min(0.05, 1.0 * err))
        v = math.copysign(max(0.02, abs(v)), v)
        if self.stuck(now, abs(v), 0.0):
            return
        self.pub(v, 0.0)

    def do_exit(self, now):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        back = math.dist(self.enter_from, self.pose[:2]) if self.enter_from else 0.0
        d = self.bay_frame()[0]
        # 들어간 거리만큼: 들어가기 시작한 자리 (odom) 까지 칸 축 깊이로
        start_d = ((self.enter_from[0] - self.bay['c'][0]) * math.cos(self.bay['axis'])
                   + (self.enter_from[1] - self.bay['c'][1]) * math.sin(self.bay['axis']))
        if d <= start_d + 0.005:
            self.pub(0.0, 0.0)
            nxt = self.exit_to or 'ROTATE_OUT'
            if nxt == 'ALIGN' and self.ack():
                self.arc_then = 'ALIGN'
                self.set_state('ARC_OUT', '빠져나옴 → 후진 원호로 차로에 나와 정렬부터')
            elif nxt == 'ALIGN':
                self.rot_target = self.h0
                self.after_rot = 'ALIGN'
                self.set_state('ROTATE_OUT_RETRY', '빠져나옴 → 돌아서 정렬부터')
            elif self.ack():
                self.set_state('ARC_OUT', f'후진 끝 (들어간 {self.enter_len:.2f} m 만큼, 들어가기 시작한 자리에서 {back * 100:.1f} cm) → 후진 원호로 차로')
            else:
                self.rot_target = self.h0
                self.set_state('ROTATE_OUT', f'후진 끝 (들어간 {self.enter_len:.2f} m 만큼, 들어가기 시작한 자리에서 {back * 100:.1f} cm) → 반대 90°')
            return
        yl, yr = self.faces(-0.35, 0.2)
        e = (yl + yr) / 2 if (yl is not None and yr is not None) else -self.bay_frame()[1]
        herr = wrap(self.bay['axis'] - self.heading())
        # 후진: 칸 축 방향은 그대로 두고 (방향 오차만 보정), 옆 오차는 반대 부호로 조금
        w = g('enter_k_head') * herr - 0.5 * g('enter_k_lat') * e
        w = max(-g('enter_max_w'), min(g('enter_max_w'), w))
        v = -max(g('enter_min_speed'), min(g('exit_speed'), 0.6 * (d - start_d) + 0.02))
        if self.stuck(now, abs(v), w):
            return
        self.pub(v, w)

    # ---------------------------------------------------------------- RViz
    def markers(self):
        if self.pose is None:
            return
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def mk(i, typ, color, scale):
            m = Marker()
            m.header.frame_id, m.header.stamp, m.ns, m.id, m.type = 'odom', stamp, 'parking', i, typ
            m.action = Marker.ADD
            m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            m.scale.x = m.scale.y = m.scale.z = scale
            m.pose.orientation.w = 1.0
            return m

        def rect(cx, cy, a, hl, hw):
            pts = []
            for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1)):
                pts.append(Point(x=cx + sx * hl * math.cos(a) - sy * hw * math.sin(a),
                                 y=cy + sx * hl * math.sin(a) + sy * hw * math.cos(a), z=0.01))
            return pts
        car = mk(1, Marker.LINE_STRIP, (0.2, 0.6, 1.0, 1.0), 0.01)
        car.points = rect(self.pose[0], self.pose[1], self.pose[2], HL, HW)
        ma.markers.append(car)
        if self.bay is not None:
            cx, cy = self.bay['c']
            a = self.bay['axis']
            bay = mk(2, Marker.LINE_STRIP, (0.1, 1.0, 0.3, 1.0), 0.012)
            bay.points = rect(cx, cy, a, g('bay_d') / 2, g('bay_w') / 2)
            ma.markers.append(bay)
            tgt = mk(3, Marker.ARROW, (1.0, 0.8, 0.1, 1.0), 0.03)
            tgt.scale.x, tgt.scale.y, tgt.scale.z = 0.2, 0.03, 0.03
            tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z = cx, cy, 0.02
            tgt.pose.orientation.z, tgt.pose.orientation.w = math.sin(a / 2), math.cos(a / 2)
            ma.markers.append(tgt)
            if self.state in ('ENTER', 'CHECK', 'FIX', 'HOLD', 'EXIT'):
                m = self.corners_margin()
                for k, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, -1), (-1, 1))):
                    t = mk(10 + k, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0) if m[k] >= 0 else (1.0, 0.2, 0.2, 1.0), 0.05)
                    x, y, yaw = self.pose
                    t.pose.position.x = x + sx * (HL + 0.04) * math.cos(yaw) - sy * (HW + 0.04) * math.sin(yaw)
                    t.pose.position.y = y + sx * (HL + 0.04) * math.sin(yaw) + sy * (HW + 0.04) * math.cos(yaw)
                    t.pose.position.z = 0.1
                    t.text = f'{m[k] * 100:.1f}'
                    ma.markers.append(t)
        st = mk(20, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0), 0.06)
        st.pose.position.x, st.pose.position.y, st.pose.position.z = self.pose[0], self.pose[1], 0.35
        st.text = f'M4 {self.state}'
        ma.markers.append(st)
        self.mk_pub.publish(ma)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ParkingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.pub(0.0, 0.0)
        except Exception:   # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()
