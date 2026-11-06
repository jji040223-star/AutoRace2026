"""휠 거리 + IMU 방향 오도메트리 (odom_imu) — 스키드 LIMO 의 휠 오도메트리는 도는 각도를 크게 틀린다.

  sub  /odom (nav_msgs/Odometry, limo_base)  — 앞으로 간 속도만 씀
  sub  /imu  (sensor_msgs/Imu)               — 자이로 z 를 적분한 방향
  pub  /odom_imu (nav_msgs/Odometry)         — 위치 = 휠 속도 × IMU 방향으로 적분, 방향 = IMU. TF 는 안 냄.
  pub  /imu_unbiased (sensor_msgs/Imu)       — /imu 에서 자이로 치우침을 뺀 것 (mission·parking 의 자이로 적분이 받음)

자이로 치우침 (멈춰 있을 때 갱신, ZUPT): 휠이 zupt_hold_s 넘게 멈춰 있으면 (|v|, |w| 작음) 그동안 자이로 z 값을 천천히 따라가
치우침으로 삼는다. 정지선·신호·후진 대기처럼 서는 때가 많아 치우침이 쌓여 흐르는 것을 막는다 (2D 시뮬 11-06: 0.11°/s 치우침이면
M7 60~100 s 동안 ~10° 흘러 멈춤·차로 착각).

실차 (11-05): 90° 코너를 휠 오도메트리는 약 69° 로 잼 (23 % 적게) — 코너 몇 개 뒤 방향이 40~60° 틀어져
M2·M7 차로 옮기기가 대각선으로 나갔다. 같은 주행에서 자이로 적분과 IMU 자세값은 1° 안으로 맞고 멈춰 있을 때 흐르지 않았다.
IMU 가 imu_timeout_s 동안 안 오면 휠 오도메트리 각속도로 대신 돈다. /odom 위치가 0.5 m 넘게 튀면 (시뮬 리셋) 그 자세로 다시 시작.
"""
import math
import signal

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class OdomImu(Node):
    def __init__(self):
        super().__init__('odom_imu')
        p = self.declare_parameter
        p('odom_in', '/odom')
        p('imu_topic', '/imu')
        p('odom_out', '/odom_imu')
        p('imu_timeout_s', 0.5)
        p('jump_reset_m', 0.5)
        p('zupt_hold_s', 0.5)              # 휠이 이만큼 멈춰 있어야 치우침을 잼 (멈추는 순간의 흔들림 제외)
        p('zupt_v', 0.003)                 # [m/s] 이보다 느리면 멈춤
        p('zupt_w', 0.01)                  # [rad/s] 휠 각속도 이보다 작으면 멈춤
        p('zupt_max', 0.05)                # [rad/s] 자이로가 이보다 크면 (멈춘 게 아님) 안 씀
        p('zupt_keep', 0.7)                # 멈춘 구간이 끝날 때 누적 평균에 합침: 그 구간 200 장마다 예전 무게 × 이것
        p('imu_out', '/imu_unbiased')
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.pub = self.create_publisher(Odometry, g('odom_out'), 50)
        self.imu_pub = self.create_publisher(Imu, g('imu_out'), qos_profile_sensor_data)
        self.bias = 0.0
        self.still_since = None            # 휠이 멈춘 시각 (odom stamp)
        self.n_zupt = 0
        self.seg_s = self.seg_n = 0.0      # 이번 정지 구간 합·장 수
        self.acc_s = self.acc_n = 0.0      # 지난 정지 구간들 누적 (zupt_keep 로 줄여 감)
        self.create_subscription(Odometry, g('odom_in'), self.on_odom, 50)
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, qos_profile_sensor_data)
        self.x = self.y = self.yaw = 0.0
        self.imu_yaw = None                # 자이로 적분 (임의 기준)
        self.imu_t = None
        self.imu_wz = 0.0
        self.off = None                    # yaw = imu_yaw + off
        self.last = None                   # (t, odom x, odom y, odom yaw)
        self.n_fallback = 0
        self.create_timer(10.0, self.report)
        self.get_logger().info(f"odom_imu: {g('odom_in')} (속도) + {g('imu_topic')} (방향) → {g('odom_out')}")

    def on_imu(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        t = stamp_s(msg.header)
        raw = msg.angular_velocity.z
        if (self.still_since is not None and t - self.still_since > g('zupt_hold_s')
                and abs(raw - self.bias) < g('zupt_max')):
            self.seg_s += raw
            self.seg_n += 1
            # 정지 중엔 누적 + 이번 구간 평균으로 바로 반영 (EMA 는 마지막 몇 장 잡음에 끌림 — 다른 세션 검토 11-06)
            self.bias = (self.acc_s + self.seg_s) / (self.acc_n + self.seg_n)
            self.n_zupt += 1
        wz = raw - self.bias
        msg.angular_velocity.z = wz
        self.imu_pub.publish(msg)
        if self.imu_t is None:
            self.imu_yaw = 0.0
        elif 0.0 < t - self.imu_t < 0.5:
            self.imu_yaw = wrap(self.imu_yaw + 0.5 * (wz + self.imu_wz) * (t - self.imu_t))
        self.imu_t, self.imu_wz = t, wz

    def on_odom(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        t = stamp_s(msg.header)
        ox, oy = msg.pose.pose.position.x, msg.pose.pose.position.y
        oyaw = yaw_of(msg.pose.pose.orientation)
        tw = msg.twist.twist
        if abs(tw.linear.x) < g('zupt_v') and abs(tw.angular.z) < g('zupt_w'):
            self.still_since = self.still_since if self.still_since is not None else t
        else:
            if self.seg_n > 0:             # 정지 구간 끝 → 누적에 합침
                k = g('zupt_keep') ** (self.seg_n / 200.0)    # 200 장 (IMU 100 Hz 면 2 s) 마다 zupt_keep — 짧은 정지가 여러 번 와도 긴 정지 정보 유지
                self.acc_s, self.acc_n = k * self.acc_s + self.seg_s, k * self.acc_n + self.seg_n
                self.seg_s = self.seg_n = 0.0
            self.still_since = None
        if self.last is None or math.hypot(ox - self.last[1], oy - self.last[2]) > g('jump_reset_m'):
            if self.last is not None:
                self.get_logger().info(f'[odom_imu] /odom 이 {math.hypot(ox - self.last[1], oy - self.last[2]):.2f} m 튐 — 그 자세로 다시 시작')
            self.x, self.y, self.yaw = ox, oy, oyaw
            self.off = None if self.imu_yaw is None else wrap(oyaw - self.imu_yaw)
            self.last = (t, ox, oy, oyaw)
            self.publish(msg, 0.0)
            return
        dt = t - self.last[0]
        imu_ok = self.imu_yaw is not None and self.imu_t is not None and abs(t - self.imu_t) < g('imu_timeout_s')
        if imu_ok:
            if self.off is None:
                self.off = wrap(self.yaw - self.imu_yaw)
            new_yaw = wrap(self.imu_yaw + self.off)
            wz = self.imu_wz
        else:
            new_yaw = wrap(self.yaw + wrap(oyaw - self.last[3]))
            wz = msg.twist.twist.angular.z
            self.off = None
            self.n_fallback += 1
        if 0.0 < dt < 0.5:
            v = msg.twist.twist.linear.x
            mid = self.yaw + 0.5 * wrap(new_yaw - self.yaw)
            self.x += v * dt * math.cos(mid)
            self.y += v * dt * math.sin(mid)
        self.yaw = new_yaw
        self.last = (t, ox, oy, oyaw)
        self.publish(msg, wz)

    def publish(self, src, wz):
        out = Odometry()
        out.header = src.header
        out.child_frame_id = src.child_frame_id
        out.pose.pose.position.x, out.pose.pose.position.y = self.x, self.y
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        out.twist.twist.linear.x = src.twist.twist.linear.x
        out.twist.twist.linear.y = src.twist.twist.linear.y
        out.twist.twist.angular.z = float(wz)
        self.pub.publish(out)

    def report(self):
        if self.last is not None:
            self.get_logger().info(f'[odom_imu] 방향 {math.degrees(self.yaw):.1f}° (휠만 {math.degrees(self.last[3]):.1f}°), '
                                   f'IMU 없어 휠로 돈 횟수 {self.n_fallback}, 자이로 치우침 {math.degrees(self.bias):+.3f}°/s '
                                   f'(멈춰서 잰 장 수 {self.n_zupt})')


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = OdomImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
