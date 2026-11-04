"""속도 명령 선택기 (v3 4장 /cmd_vel_mux) — 여러 노드가 낸 속도 명령 중 지금 미션에 맞는 하나만 차에 보낸다.

  sub  lane_topic    (기본 /lane/cmd_vel)    — lane_follower (drive.launch 에서 /cmd_vel 대신 여기로)
  sub  mission_topic (기본 /mission/cmd_vel) — mission_manager 가 직접 운전하는 미션 (M7 회피 …)
  sub  parking_topic (기본 /parking/cmd_vel)  — parking_controller (M4 주차·탈출)
  sub  state_topic   (기본 /mission/state), parking_state_topic (기본 /parking/state)
  pub  cmd_vel_topic (기본 /cmd_vel)         — 차 (limo_base / autorace_sim)
  pub  selected_topic (기본 /cmd_vel_mux/selected, std_msgs/String) — /cmd_vel 을 낼 때마다 그 출처: "lane" | "mission" |
                                                 "zero:<lane|mission>" (고른 입력이 끊겨 낸 0) — rosbag 으로 메시지 단위 확인용

/parking/state 가 진행 중 (IDLE·WAIT·DONE 아님, 1 s 안에 받음) 이면 주차 명령 (WAIT = 차선 추종이 주차 구역까지 몰고 감), 아니면 /mission/state 가 mission_states 중 하나면 미션 명령,
아니면 차선 명령을 그대로 넘긴다 (주차는 미션 매니저 없이 단독으로도 돈다).
고른 쪽 명령이 stale_s 동안 안 오면 0 을 한 번 낸다 (그 노드가 죽어도 차가 마지막 명령으로 계속 가지 않게).
라이다 안전 정지 (guard_states 상태 또는 주차 중, 뎁스·미션 판단과 별개): /scan 에서 차 앞 (차 폭 + guard_side_m 통로) guard_front_m 안에
점이 있으면 v > 0 을 0 으로 (회전은 그대로), 뒤 guard_rear_m 안이면 v < 0 을 0 으로. 라이다 최소 거리 (0.12 m) 때문에 차에 바짝 붙은
물체는 라이다에서 사라지므로, 한 번 막으면 차가 guard_release_m 물러나거나 guard_release_deg 돌 때까지 막아 둔다.
로그: 고른 입력이 바뀔 때, log_period_s 마다 요약 (고른 입력, 넘긴·버린 수, 마지막 명령, 입력별 나이),
     내보내는 명령이 정지 (|v|, |w| < stop_eps) 로 바뀔 때와 다시 움직일 때 (출처·상태와 같이), 입력이 끊겨 0 을 낼 때.
"""
import math
import signal

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        p = self.declare_parameter
        p('lane_topic', '/lane/cmd_vel')
        p('mission_topic', '/mission/cmd_vel')
        p('state_topic', '/mission/state')
        p('parking_topic', '/parking/cmd_vel')
        p('parking_state_topic', '/parking/state')
        p('cmd_vel_topic', '/cmd_vel')
        p('mission_states', ['M7_AVOID'])
        p('stale_s', 0.3)
        p('selected_topic', '/cmd_vel_mux/selected')
        p('log_period_s', 2.0)             # 0 = 요약 로그 안 냄
        p('stop_eps', 0.01)                # 이보다 작은 |v|, |w| 는 정지 명령
        p('guard_states', ['M7_AVOID'])    # 라이다 안전 정지를 켜는 상태 ([] = 늘)
        p('scan_topic', '/scan')
        p('odom_topic', '/odom')
        p('laser_x', 0.103)                # base_link → 라이다 [m]
        p('car_half_l', 0.161)
        p('car_half_w', 0.109)
        p('guard_front_m', 0.10)           # 앞 범퍼에서 [m]. 라이다 최소 거리 0.12 m → 범퍼 앞 0.06 m 안은 안 보인다
        p('guard_rear_m', 0.08)
        p('guard_side_m', 0.02)            # 통로 = 차 폭 + 양쪽 이만큼
        p('guard_release_m', 0.03)         # 막은 뒤 이만큼 물러나거나
        p('guard_release_deg', 15.0)       #   이만큼 돌면 풀어 줌 (그때 다시 보고 판단)
        p('scan_stale_s', 0.5)             # 라이다가 이만큼 끊기면 안전 정지 안 함 (경고)
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.state = 'LANE'
        self._last = {'lane': None, 'mission': None, 'parking': None}    # 마지막으로 받은 시각
        self._park = (None, -math.inf)     # (/parking/state, 받은 시각)
        self._zeroed = False
        self._n = {'lane': [0, 0], 'mission': [0, 0], 'parking': [0, 0]}     # 요약 기간 동안 [넘김, 버림]
        self._cmd = {'lane': None, 'mission': None, 'parking': None}       # 마지막으로 받은 명령
        self._out = None                   # 마지막으로 낸 (출처, v, w)
        self._stop_since = None            # 정지 명령을 내기 시작한 (시각, 출처)
        self._last_log = None
        self._scan = None                  # (시각, 앞 빈 거리, 뒤 빈 거리) — 범퍼에서 [m]
        self._pose = None                  # (x, y, yaw)
        self._latch = {'front': None, 'rear': None}   # 막은 때의 자세
        self._stale_warned = False
        self.pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.sel_pub = self.create_publisher(String, g('selected_topic'), 10)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 20)
        self.create_subscription(Twist, g('lane_topic'), lambda m: self.on_cmd('lane', m), 10)
        self.create_subscription(Twist, g('mission_topic'), lambda m: self.on_cmd('mission', m), 10)
        self.create_subscription(String, g('state_topic'), self.on_state, 10)
        self.create_subscription(Twist, g('parking_topic'), lambda m: self.on_cmd('parking', m), 10)
        self.create_subscription(String, g('parking_state_topic'), self.on_park_state, 10)
        self.create_timer(0.05, self.watchdog)
        self.get_logger().info(f"cmd_vel_mux: {g('lane_topic')} | {g('mission_topic')} (상태 {g('mission_states')}) → {g('cmd_vel_topic')}")

    def parking(self):
        st, t = self._park
        return st not in (None, '', 'IDLE', 'WAIT', 'DONE') and self.get_clock().now().nanoseconds * 1e-9 - t < 1.0

    def source(self):
        if self.parking():
            return 'parking'
        return 'mission' if self.state in self.get_parameter('mission_states').value else 'lane'

    def on_park_state(self, msg):
        before = self.source()
        self._park = (msg.data, self.get_clock().now().nanoseconds * 1e-9)
        if self.source() != before:
            self.get_logger().info(f'[mux] {before} → {self.source()} (주차 {msg.data}, 상태 {self.state})')

    def on_state(self, msg):
        if msg.data != self.state:
            before = self.source()
            self.state = msg.data
            if self.source() != before:
                self.get_logger().info(f'[mux] {before} → {self.source()} (상태 {self.state})')

    def on_cmd(self, who, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._last[who] = now
        self._cmd[who] = msg
        if who == self.source():
            self.emit(msg, who, now)
            self._zeroed = False
            self._n[who][0] += 1
        else:
            self._n[who][1] += 1

    def on_scan(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        r = np.asarray(msg.ranges, np.float32)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        x = r[ok] * np.cos(a[ok]) + g('laser_x')
        y = r[ok] * np.sin(a[ok])
        L, Wd = g('car_half_l'), g('car_half_w') + g('guard_side_m')
        lane = (np.abs(y) < Wd) & ~((np.abs(x) < L + 0.005) & (np.abs(y) < g('car_half_w') + 0.005))   # 내 차 몸에 맞은 점은 뺌
        fx, rx = x[lane & (x > L)] - L, -x[lane & (x < -L)] - L
        now = self.get_clock().now().nanoseconds * 1e-9
        self._scan = (now, float(fx.min()) if fx.size else math.inf, float(rx.min()) if rx.size else math.inf)
        self.release()

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self.release()

    def release(self):
        """막아 둔 방향을 풀어 줄지: 물러났거나 돌았고, 지금 라이다로 그쪽이 비어 있으면."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        if self._pose is None or self._scan is None:
            return
        for side, i, lim in (('front', 1, g('guard_front_m')), ('rear', 2, g('guard_rear_m'))):
            lp = self._latch[side]
            if lp is None:
                continue
            moved = math.dist(lp[:2], self._pose[:2])
            turned = abs(math.degrees(math.atan2(math.sin(self._pose[2] - lp[2]), math.cos(self._pose[2] - lp[2]))))
            if (moved >= g('guard_release_m') or turned >= g('guard_release_deg')) and self._scan[i] > lim:
                self._latch[side] = None
                self.get_logger().info(f'[mux] 라이다 안전 정지 풀림 ({"앞" if side == "front" else "뒤"}): {moved * 100:.1f} cm, '
                                       f'{turned:.0f}° 움직인 뒤 {self._scan[i]:.2f} m 빔 (상태 {self.state})')

    def guard(self, msg, tag, now):
        """라이다 안전 정지 — 막으면 v 만 0 으로 바꾼 명령과 '+guard' 붙인 출처를 돌려준다."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        states = g('guard_states')
        if states and self.state not in states and not self.parking():
            return msg, tag
        if self._scan is None or now - self._scan[0] > g('scan_stale_s'):
            if not self._stale_warned:
                self._stale_warned = True
                self.get_logger().warn(f'[mux] 라이다 ({g("scan_topic")}) 가 안 들어옴 — 안전 정지 꺼짐 (상태 {self.state})')
            return msg, tag
        self._stale_warned = False
        v = msg.linear.x
        for side, i, lim, bad in (('front', 1, g('guard_front_m'), v > 0), ('rear', 2, g('guard_rear_m'), v < 0)):
            if not bad:
                continue
            if self._latch[side] is None and self._scan[i] <= lim:
                self._latch[side] = self._pose or (0.0, 0.0, 0.0)
                self.get_logger().warn(f'[mux] 라이다 안전 정지 ({"앞" if side == "front" else "뒤"}): 범퍼에서 {self._scan[i]:.2f} m '
                                       f'— {tag} 의 v {v:+.2f} 를 0 으로 (w {msg.angular.z:+.2f} 는 그대로, 상태 {self.state})')
            if self._latch[side] is not None:
                out = Twist()
                out.angular.z = msg.angular.z
                return out, f'{tag}+guard'
        return msg, tag

    def emit(self, msg, tag, now):
        """/cmd_vel 과 그 출처를 같이 내고, 정지 명령의 시작·끝을 로그로."""
        msg, tag = self.guard(msg, tag, now)
        self.pub.publish(msg)
        self.sel_pub.publish(String(data=tag))
        eps = self.get_parameter('stop_eps').value
        v, w = msg.linear.x, msg.angular.z
        stopped = abs(v) < eps and abs(w) < eps
        if stopped and self._stop_since is None:
            self._stop_since = (now, tag)
            self.get_logger().info(f'[mux] 정지 명령 시작 — 출처 {tag} (상태 {self.state})')
        elif not stopped and self._stop_since is not None:
            t0, tag0 = self._stop_since
            self._stop_since = None
            self.get_logger().info(f'[mux] 정지 명령 끝 ({now - t0:.1f} s, 출처 {tag0}) — 이제 {tag} v {v:+.2f} w {w:+.2f} (상태 {self.state})')
        self._out = (tag, v, w)

    def watchdog(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        src = self.source()
        t = self._last[src]
        if (t is None or now - t > self.get_parameter('stale_s').value) and not self._zeroed:
            if t is not None:
                self.get_logger().warn(f'[mux] {src} 입력이 {now - t:.2f} s 끊김 → 0 을 냄 (상태 {self.state})')
            self.emit(Twist(), f'zero:{src}', now)
            self._zeroed = True
        period = self.get_parameter('log_period_s').value
        if period > 0 and (self._last_log is None or now - self._last_log >= period):
            if self._last_log is not None:
                age = {k: ('없음' if v is None else f'{now - v:.2f} s') for k, v in self._last.items()}
                out = '없음' if self._out is None else f'{self._out[0]} v {self._out[1]:+.2f} w {self._out[2]:+.2f}'
                self.get_logger().info(
                    f'[mux] 고름 {src} (상태 {self.state}) — {now - self._last_log:.1f} s 동안 lane 넘김 {self._n["lane"][0]}/버림 '
                    f'{self._n["lane"][1]}, mission 넘김 {self._n["mission"][0]}/버림 {self._n["mission"][1]}, '
                    f'parking 넘김 {self._n["parking"][0]}/버림 {self._n["parking"][1]}, '
                    f'마지막 /cmd_vel {out}, 입력 나이 lane {age["lane"]} mission {age["mission"]} parking {age["parking"]}')
            self._n = {'lane': [0, 0], 'mission': [0, 0], 'parking': [0, 0]}
            self._last_log = now


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
