"""LIMO 4륜 스키드 디퍼렌셜 구동: /cmd_vel (v, w) → 바퀴 속도 → 접지력.

왜 타이어 모델을 따로 두나
  PhysX 접촉 마찰은 '굳은' 쿨롱 마찰이라, 축거/윤거 L/W > 1 인 4륜 스키드 차량은 옆 미끄러짐 저항
  모멘트가 구동 모멘트보다 커서 거의 돌지 못한다. LIMO 는 L/W = 0.20/0.175 = 1.14 이고, 실측으로
  PhysX 마찰 그대로면 제자리 회전이 명령의 ~20 %, 주행 중 회전은 ~5 % 였다. 실차는 타이어 변형
  (미끄럼에 비례하는 힘) 덕분에 돈다.
  model: tire  → 바퀴 충돌체는 마찰 0 (하중만 받음). 바퀴 접지점마다 미끄럼 속도에 비례하는 종/횡력을
                  계산해 μ·N 마찰원으로 자르고 차체에 건다. μ 는 바닥 마찰 파라미터 → 마찰을 낮추면
                  가속·제동·선회 한계가 실제로 낮아진다.
  model: physx → PhysX 마찰 그대로 (비교용).
  tracking     → 몸체 실제 속도를 되먹임(PI)해 명령 트위스트를 따라감 = 실차 하위 제어기가 명령 속도를
                  내 준다고 보는 것. false 면 기구학 그대로 바퀴 속도만 준다.
"""
import math

import numpy as np


class LimoDrive:
    """(v, w) 명령 → 좌/우 바퀴 각속도 [rad/s]."""

    def __init__(self, cfg, yaw_ratio=1.0):
        self.r = float(cfg["wheel_radius"])
        self.W = float(cfg["track_width"])
        self.v_max = float(cfg["max_linear_vel"])
        self.w_max = float(cfg["max_angular_vel"])
        self.wheel_max = float(cfg["max_wheel_speed"]) / self.r
        t = cfg["tracking"]
        self.tracking = bool(t["enabled"])
        self.kp_v, self.ki_v = float(t["kp_v"]), float(t["ki_v"])
        self.kp_w, self.ki_w = float(t["kp_w"]), float(t["ki_w"])
        self.ff_w = 1.0 / yaw_ratio            # 정상상태 회전비를 미리 보상
        self.i_max = float(t["integral_limit"])
        self.iv = self.iw = 0.0

    def reset(self):
        """적분항 비우기 (리셋 전 벽에 막혀 쌓인 값이 출발 때 튀지 않게)."""
        self.iv = self.iw = 0.0

    def wheel_speeds(self, v_ref, w_ref, v_meas, w_meas, dt):
        v_ref = max(-self.v_max, min(self.v_max, v_ref))
        w_ref = max(-self.w_max, min(self.w_max, w_ref))
        v_u, w_u = v_ref, w_ref
        if self.tracking:
            ev, ew = v_ref - v_meas, w_ref - w_meas
            self.iv = max(-self.i_max, min(self.i_max, self.iv + ev * dt))
            self.iw = max(-self.i_max, min(self.i_max, self.iw + ew * dt))
            v_u = v_ref + self.kp_v * ev + self.ki_v * self.iv
            w_u = self.ff_w * w_ref + self.kp_w * ew + self.ki_w * self.iw
        wl = (v_u - w_u * self.W / 2) / self.r
        wr = (v_u + w_u * self.W / 2) / self.r
        s = max(1.0, abs(wl) / self.wheel_max, abs(wr) / self.wheel_max)   # 비율 유지하며 포화
        return wl / s, wr / s


class TireModel:
    """바퀴 접지점 미끄럼 속도 → 종/횡력 (선형, μ·N 마찰원으로 포화).

    wheels: [(x, y)] base_link 기준 접지점 (FL, RL, FR, RR 순서), ground_z: base_link 아래 접지 높이(음수)
    """

    def __init__(self, wheels, ground_z, c_long, c_lat, mass, mu, g=9.81):
        self.wheels = [np.array([x, y, ground_z]) for x, y in wheels]
        self.c_long, self.c_lat = float(c_long), float(c_lat)
        self.normal = mass * g / len(wheels)
        self.mu = float(mu)

    def yaw_ratio(self):
        """제자리 회전 정상상태: 실제 w / 기구학 w = Cx·W² / (Cx·W² + Cy·L²) (W, L = 평균 윤거·축거)."""
        W = np.mean([abs(p[1]) for p in self.wheels]) * 2
        L = np.mean([abs(p[0]) for p in self.wheels]) * 2
        return self.c_long * W * W / (self.c_long * W * W + self.c_lat * L * L)

    def forces(self, roll_speeds, base_pos, base_rot, com_pos, lin_vel, ang_vel):
        """roll_speeds: 바퀴별 굴림 속도 [m/s] (= ω_wheel·r). 반환: [(힘, 작용점)] 월드 좌표."""
        x_b, y_b = base_rot[:, 0].copy(), base_rot[:, 1].copy()
        x_b[2] = y_b[2] = 0.0
        x_b /= np.linalg.norm(x_b) or 1.0
        y_b /= np.linalg.norm(y_b) or 1.0
        out = []
        f_max = self.mu * self.normal
        for p_local, u in zip(self.wheels, roll_speeds):
            p = base_pos + base_rot @ p_local
            v = lin_vel + np.cross(ang_vel, p - com_pos)
            fx = self.c_long * (u - v @ x_b)
            fy = -self.c_lat * (v @ y_b)
            mag = math.hypot(fx, fy)
            if mag > f_max:
                fx, fy = fx * f_max / mag, fy * f_max / mag
            out.append((fx * x_b + fy * y_b, p))
        return out


def body_twist(lin_vel_world, ang_vel_world, quat_wxyz):
    """월드 좌표 속도 → (전진 속도, 요 각속도). 평면 주행 가정."""
    w, x, y, z = quat_wxyz
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    v = lin_vel_world[0] * math.cos(yaw) + lin_vel_world[1] * math.sin(yaw)
    return v, ang_vel_world[2]


def quat_to_rot(q_wxyz):
    w, x, y, z = q_wxyz
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
