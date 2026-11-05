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
        p('gap_min_m', 0.45)               # 빈 칸 폭 (LIMO 옆면 사이) 최소 — 차 폭 0.22 + 여유
        p('gap_max_m', 1.30)
        p('limo_half_l', 0.161)            # 이웃 LIMO 반 길이 (끝면 → 칸 깊이 가운데)
        p('align_tol_m', 0.008)
        p('align_speed', 0.06)
        p('rot_margin_m', 0.03)            # 제자리 회전 쓸기 검사 여유
        p('rot_tol_deg', 1.5)
        p('rot_max_w', 0.8)
        p('rot_min_w', 0.2)
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
        self.mon = StuckMonitor(self.get_parameter('stuck_s').value, 0.01, 2.0, 0.02, 0.1)
        self.phase_t = None
        self.after_rot = None
        self.exit_to = None
        self.result = ''
        self.rot_w = 0.0
        self.rot_still = None

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

    def rotate(self, target, now, then):
        """IMU 로 target 까지 제자리 회전. 끝나면 then 상태로."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        err = wrap(target - self.heading())
        wz = self.imu[2] if self.imu is not None else 0.0
        tol = math.radians(g('rot_tol_deg'))
        if abs(err) < tol:
            # 오차 안: 멈추고 흔들림이 가라앉을 때까지 (0.3 s 동안 |wz| < 0.05) — 구동 지연 때문에 돌면서 끝내면 넘어간다
            self.pub(0.0, 0.0)
            self.rot_w = 0.0
            if abs(wz) >= 0.05:
                self.rot_still = None
                return False
            self.rot_still = self.rot_still or now
            return now - self.rot_still >= 0.3
        self.rot_still = None
        # 구동 지연 (rot_lag_s) 동안 지금 각속도로 더 돌 것을 빼고 본 오차 — 0 을 명령해도 그만큼 더 돈다
        pred = err - wz * g('rot_lag_s')
        if abs(pred) < tol or pred * err < 0:
            self.pub(0.0, 0.0)
            self.rot_w = 0.0
            return False
        # 오차 비례 (rot_slow_deg 에서 rot_max_w), 최소 rot_min_w (스키드는 작은 각속도로는 안 돈다)
        mag = max(g('rot_min_w'), min(g('rot_max_w'), g('rot_max_w') * abs(pred) / math.radians(g('rot_slow_deg'))))
        w = math.copysign(mag, pred)
        self.pub(0.0, w)
        self.rot_w = w
        return False

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
            if self.rotate(self.rot_target, now, 'MEASURE'):
                self.rot_err['in'] = math.degrees(wrap(self.heading() - self.rot_target))
                self.set_state('MEASURE', f'90° 회전 끝 (IMU 오차 {self.rot_err["in"]:+.1f}°)')
                self.meas = []
            elif self.stuck(now, 0.0, self.rot_w):
                return
        elif s == 'MEASURE':
            self.do_measure(now)
        elif s == 'ENTER':
            self.do_enter(now)
        elif s == 'CHECK':
            self.do_check(now)
        elif s == 'FIX':
            self.do_fix(now)
        elif s == 'ROTATE_OUT_RETRY':
            if self.rotate(self.rot_target, now, self.after_rot):
                self.set_state(self.after_rot or 'ALIGN', '처음 방향으로 돌아섬 → 다시 정렬')
            elif self.stuck(now, 0.0, self.rot_w):
                return
        elif s == 'HOLD':
            self.pub(0.0, 0.0)
            if now - self.phase_t >= g('hold_s'):
                self.exit_to = 'ROTATE_OUT'
                self.set_state('EXIT', f'{g("hold_s"):.1f} s 정지 끝 → 들어간 거리 {self.enter_len:.2f} m 후진')
        elif s == 'EXIT':
            self.do_exit(now)
        elif s == 'ROTATE_OUT':
            if self.rotate(self.h0, now, 'DONE'):
                self.rot_err['out'] = math.degrees(wrap(self.heading() - self.h0))
                self.result += f', 나올 때 90° 회전 오차 (IMU) {self.rot_err["out"]:+.1f}°, 전체 {now - self.t0:.1f} s'
                self.set_state('DONE', self.result)
                self.result_pub.publish(String(data=self.result))
                self.done_pub.publish(Empty())
                self.pub(0.0, 0.0)
            elif self.stuck(now, 0.0, self.rot_w):
                return

    def do_approach(self, now):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        d = self.side_dist()
        pr = self.prof
        a = self.along()
        near = d is not None and d < g('side_near_m')
        far = d is None or d > g('side_far_m')
        if pr['phase'] == 'near1' and near:
            pr['phase'] = 'in_near1'
        elif pr['phase'] == 'in_near1':
            if near:
                pr['near_d'].append(d)
            elif far:
                pr['phase'], pr['far0'] = 'far', a
                self.get_logger().info(f'[M4] 옆: 가까움 → 멂 (LIMO 끝, 진행 {a:.3f} m, LIMO 끝면까지 {np.median(pr["near_d"]):.3f} m)')
        elif pr['phase'] == 'far' and near:
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
            pr['phase'], pr['near_d'] = 'in_near1', [d]
            self.get_logger().info(f'[M4] 틈 {w:.3f} m — 칸 폭이 아님, 다음 틈을 찾음')
        if near and self.row_ref is None:
            self.row_ref = d
        # 방향 유지 (IMU) + 주차 LIMO 줄과 옆 거리 유지 (가까움 구간에서만)
        h_cmd = self.h0
        if near and self.row_ref is not None:
            corr = g('row_keep_k') * (d - self.row_ref) * self.sgn
            h_cmd = self.h0 + max(-math.radians(g('row_keep_max_deg')), min(math.radians(g('row_keep_max_deg')), corr))
        w = 1.5 * wrap(h_cmd - self.heading())
        v = g('approach_speed')
        if self.stuck(now, v, w):
            return
        self.pub(v, max(-0.4, min(0.4, w)))

    def do_align(self, now):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        err = self.bay['along'] - self.along()
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
        if 0 < abs(v) < 0.02:
            v = math.copysign(0.02, v)
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
        if abs(e) > g('measure_tol_m'):
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
            if nxt == 'ALIGN':
                self.rot_target = self.h0
                self.after_rot = 'ALIGN'
                self.set_state('ROTATE_OUT_RETRY', '빠져나옴 → 돌아서 정렬부터')
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
