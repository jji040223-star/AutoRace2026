"""M7 (3차선 정차 LIMO) · M8 (라바콘) 공통 — 차 사각형 쓸기 검사, 라이다 점 상자, 진행 감시.

좌표는 쓰는 쪽이 정한 평면 (s 앞, l 왼쪽, psi 는 s 축에서 왼쪽 +). 상자는 축에 나란한 [s0, s1, l0, l1].
  hits / margins / escape / free / rot_ok : 차 사각형 (반 길이·반 폭 + 여유) 을 곧게 밀거나 제자리에서 돌려 보며 상자와 겹침 검사
  scan_boxes                              : 라이다 점 (base_link) → 그 평면의 작은 상자 — 뎁스 기억이 가까이서 틀려도 실제 거리로
  arc_clear                               : 곡률 후보마다 그 호를 따라 갈 때 상자까지 최소 여유 (여유 사다리로)
  cone_gates / gate_chain / gate_path / gate_aim : M8 라바콘 사이 '틈' (통로를 가로지르는 두 콘) 과 그 가운데점을 이은 길에서 차의 옆 오차·길 방향
  StuckMonitor                            : 명령은 움직이라는데 check_s 동안 move_m·rot_deg 도 못 움직임 (밀고 있음)
  RetryCounter                            : 같은 자리 (radius 안) 재시도 횟수
"""
import math

import numpy as np

CAR_HALF_L, CAR_HALF_W = 0.161, 0.109     # LIMO 반 길이·반 폭 [m] (base_link 중심)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def hits(poses, boxes, margins):
    """poses (N,3) [s, l, psi] 의 차 사각형 (반 길이·반 폭 + 상자별 margin) 과 boxes (M,4) 의 겹침 (N, M) — 분리축 검사."""
    P = np.asarray(poses, float)[:, None, :]
    B = np.asarray(boxes, float)[None, :, :]
    mg = np.asarray(margins, float)[None, :]
    a, b = CAR_HALF_L + mg, CAR_HALF_W + mg
    c, s = np.cos(P[..., 2]), np.sin(P[..., 2])
    ac, as_ = np.abs(c), np.abs(s)
    hs, hl = (B[..., 1] - B[..., 0]) / 2, (B[..., 3] - B[..., 2]) / 2
    dx, dy = (B[..., 0] + B[..., 1]) / 2 - P[..., 0], (B[..., 2] + B[..., 3]) / 2 - P[..., 1]
    return ((np.abs(dx) < hs + a * ac + b * as_) & (np.abs(dy) < hl + a * as_ + b * ac)
            & (np.abs(dx * c + dy * s) < a + hs * ac + hl * as_) & (np.abs(dy * c - dx * s) < b + hs * as_ + hl * ac))


def margins(pose, boxes, margin):
    """상자별 여유: 그 자세에서 이미 여유 안에 든 상자 (옆을 바짝 지나는 중) 는 0 — 안 그러면 어느 방향으로도 못 간다."""
    mg = np.full(len(boxes), float(margin))
    mg[hits([pose], boxes, mg)[0]] = 0.0
    return mg


def escape(pose, boxes, direction=None):
    """지금 자세에서 이미 겹친 (여유 0) 상자 — 기억 상자는 뎁스 오차·가정 길이로 실제보다 조금 클 수 있다. 제자리 회전
    (direction None) 이나 그 상자 중심에서 멀어지는 방향이면 빼고 검사한다. 다가가는 방향이면 None (= 못 감)."""
    if not boxes:
        return boxes
    over = hits([pose], boxes, np.zeros(len(boxes)))[0]
    if not over.any():
        return boxes
    keep = []
    for b, o in zip(boxes, over):
        if not o:
            keep.append(b)
        elif direction is not None:
            bc = ((b[0] + b[1]) / 2 - pose[0], (b[2] + b[3]) / 2 - pose[1])
            if bc[0] * direction[0] + bc[1] * direction[1] > 0.0:
                return None
    return keep


def free(sc, lc, h, boxes, dmax, margin, dirn=1.0, step=0.02):
    """(sc, lc) 에서 방향 h 로 곧게 (dirn −1 = 뒤로) 갈 때 부딪히기 전까지 거리. 그 방향으로 돌아선 자리부터 겹치면 −1
    (이미 겹친 상자에서 멀어지는 방향은 그 상자를 빼고 — escape)."""
    boxes = escape((sc, lc, h), list(boxes), (dirn * math.cos(h), dirn * math.sin(h)))
    if boxes is None:
        return -1.0
    if not boxes:
        return float(dmax)
    mg = margins((sc, lc, h), boxes, margin)
    d = np.arange(0.0, dmax + 1e-9, step)
    poses = np.stack([sc + dirn * d * math.cos(h), lc + dirn * d * math.sin(h), np.full_like(d, h)], 1)
    hit = hits(poses, boxes, mg).any(axis=1)
    if hit[0]:
        return -1.0
    return float(d[np.argmax(hit) - 1]) if hit.any() else float(dmax)


def rot_ok(sc, lc, psi, h, boxes, rot_margin):
    """제자리에서 psi → h 로 돌 때 부딪히지 않는가 (3° 간격). 지금 이미 겹친 상자는 뺀다 (escape)."""
    boxes = escape((sc, lc, psi), list(boxes))
    if not boxes:
        return True
    n = max(2, int(abs(wrap(h - psi)) / math.radians(3.0)) + 2)
    hs = psi + np.linspace(0.0, wrap(h - psi), n)
    mg = margins((sc, lc, psi), boxes, rot_margin)
    poses = np.stack([np.full(n, sc), np.full(n, lc), hs], 1)
    return not hits(poses, boxes, mg).any()


def scan_boxes(scan, sc, lc, psi, half=0.01, reach=1.3):
    """라이다 점 (base_link x, y 배열) → (sc, lc, psi) 에 있는 차 기준 평면의 반 변 half 상자들 (reach 안 것만)."""
    if scan is None:
        return []
    x, y = scan
    sel = np.hypot(x, y) < reach
    x, y = x[sel], y[sel]
    c, s = math.cos(psi), math.sin(psi)
    ps, pl = sc + c * x - s * y, lc + s * x + c * y
    return [(a - half, a + half, b - half, b + half) for a, b in zip(ps.tolist(), pl.tolist())]


def arc_clear(kappas, boxes, length, steps=(0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06), n=8):
    """차 (0, 0, 0) 에서 곡률 kappas 의 호를 length 만큼 갈 때 상자들과의 최소 여유 [m] (steps 사다리, 맨 아래도 닿으면 −1).
    지금 자리에서 이미 겹친 상자는 뺀다 (escape — 이미 닿은 콘에서 벗어나는 길도 고를 수 있게)."""
    boxes = escape((0.0, 0.0, 0.0), list(boxes))
    out = np.full(len(kappas), steps[-1])
    if not boxes:
        return out
    d = np.linspace(length / n, length, n)
    for j, k in enumerate(kappas):
        if abs(k) < 1e-6:
            poses = np.stack([d, np.zeros(n), np.zeros(n)], 1)
        else:
            th = k * d
            poses = np.stack([np.sin(th) / k, (1 - np.cos(th)) / k, th], 1)
        out[j] = -1.0
        for m in steps:
            if hits(poses, boxes, np.full(len(boxes), m)).any():
                break
            out[j] = m
    return out


def cone_gates(cones, gap_min, body_r, max_center=0.8, max_skew_deg=50.0):
    """라바콘 (차 좌표 x 앞, y 왼쪽) 두 개씩 묶은 틈 중 통로를 가로지르는 것: 실제 빈 거리 (중심 거리 − 몸통 지름) ≥ gap_min,
    중심 거리 ≤ max_center, 두 콘을 잇는 선이 진행 방향과 거의 수직 (max_skew_deg 안 — 같은 줄 콘끼리는 빠짐), 가운데가 차 앞.
    → [(mx, my, gap, 콘 a, 콘 b)] 가운데 x 순."""
    out = []
    skew = math.radians(max_skew_deg)
    for i, a in enumerate(cones):
        for b in cones[i + 1:]:
            d = math.dist(a, b)
            if d > max_center or d - 2 * body_r < gap_min:
                continue
            ang = math.atan2(b[1] - a[1], b[0] - a[0])
            if abs(abs(wrap(ang)) - math.pi / 2) > skew:
                continue
            mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
            if mx > -0.3:                  # 옆·조금 뒤 틈도 (지금 차가 있는 길 조각)
                out.append((mx, my, d - 2 * body_r, a, b))
    return sorted(out)


def gate_path(gates, preview=0.1):
    """틈 가운데점을 앞 순으로 이은 길 (차 좌표). 차 (0, 0) 에서 가장 가까운 점과 그 preview 앞의 길 방향, 옆 오차.
    → (길 방향 psi [rad, 왼쪽 +], 옆 오차 e [m, 길이 차 왼쪽에 있으면 +], 앞으로 남은 길 길이) 또는 None (가운데점 2 개 미만)."""
    pts = []
    for mx, my, *_ in gates:
        if not pts or mx > pts[-1][0] + 0.05:    # 거의 같은 깊이의 틈은 앞 것만
            pts.append((mx, my))
    if len(pts) < 2:
        return None
    best = None
    for i, (p, q) in enumerate(zip(pts, pts[1:])):
        dx, dy = q[0] - p[0], q[1] - p[1]
        seg = math.hypot(dx, dy)
        t = max(0.0, min(1.0, -(p[0] * dx + p[1] * dy) / (seg * seg))) if i > 0 else min(1.0, -(p[0] * dx + p[1] * dy) / (seg * seg))
        qx, qy = p[0] + t * dx, p[1] + t * dy
        d = math.hypot(qx, qy)
        if best is None or d < best[0]:
            best = (d, i, t, qx, qy)
    _, i, t, qx, qy = best
    # preview 만큼 길을 따라 앞으로 간 곳의 조각 방향
    left = preview + (1.0 - t) * 0.0
    k, tt = i, t
    while k < len(pts) - 1:
        seg = math.dist(pts[k], pts[k + 1])
        rest = (1.0 - tt) * seg
        if rest >= left:
            break
        left -= rest
        k, tt = k + 1, 0.0
    k = min(k, len(pts) - 2)
    psi = math.atan2(pts[k + 1][1] - pts[k][1], pts[k + 1][0] - pts[k][0])
    p0 = math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0])
    e = -math.sin(p0) * qx + math.cos(p0) * qy
    remain = (1.0 - t) * math.dist(pts[i], pts[i + 1]) + sum(math.dist(a, b) for a, b in zip(pts[i + 1:], pts[i + 2:]))
    return psi, e, remain


def gate_chain(gates, ref_y=0.0, step=(0.12, 0.6), jump=(0.25, 0.3), start=(0.35, 0.5), mu=2.5):
    """틈 중 가장 그럴듯하게 이어지는 줄 → [틈, …] (앞 순, cone_gates 의 원소). 줄이 없으면 [].
    A → B 로 이어짐: step[0] ≤ B.mx − A.mx ≤ step[1], |B.my − A.my| ≤ jump[0] + jump[1]·(B.mx − A.mx).
    줄의 첫 틈: 차 (0, 0) 에서 |my| ≤ start[0] + start[1]·mx.
    점수 = 틈 수 − mu·|첫 틈 my − ref_y| (ref_y = 직전에 따라가던 목표점의 옆 위치), 같으면 옆 변화 합이 작은 것.
    통로 밖 물체 (트랙 밖 사람 다리 등) 와 바깥 줄 라바콘이 만든 틈은 통로 줄에서 0.4 m 넘게 떨어져 짧은 줄밖에 못 만들고,
    통로 끝에서 진짜 줄이 짧아져도 직전 목표에서 먼 줄은 mu 로 진다. 차가 통로 가운데에 없어도 (코너에서 들어올 때) 되고,
    휘어진 통로도 된다. step[0] 은 대각 틈 (왼 i – 오 i+1, 가운데가 반 간격 0.15 m 뒤) 도 잇도록 0.15 보다 작게 (경계값은 피함)."""
    P = sorted((q for q in gates if q[0] > 0.05), key=lambda q: q[0])
    best = [None] * len(P)                 # (점수, −옆 변화 합, 이전 번호)
    for i, q in enumerate(P):
        x, y = q[0], q[1]
        if abs(y) <= start[0] + start[1] * x:
            best[i] = (1.0 - mu * abs(y - ref_y), 0.0, -1)
        for k in range(i):
            if best[k] is None:
                continue
            dx, dy = x - P[k][0], abs(y - P[k][1])
            if step[0] <= dx <= step[1] and dy <= jump[0] + jump[1] * dx:
                cand = (best[k][0] + 1.0, best[k][1] - dy, k)
                if best[i] is None or cand[:2] > best[i][:2]:
                    best[i] = cand
    if not any(best):
        return []
    i = max((k for k in range(len(P)) if best[k]), key=lambda k: best[k][:2])
    out = []
    while i >= 0:
        out.append(P[i])
        i = best[i][2]
    return out[::-1]


def gate_aim(gates, look, ref_y=0.0, **chain_kw):
    """차 (0, 0) → 이어진 틈 줄 (gate_chain) 가운데점들을 이은 길에서 길이 look 인 점 (모자라면 마지막 가운데점).
    줄이 없지만 차 앞 틈이 있으면 옆 위치가 ref_y 에 가장 가까운 틈 가운데점. 틈이 없으면 None."""
    chain = gate_chain(gates, ref_y, **chain_kw)
    if not chain:
        ahead = [q for q in gates if q[0] > 0.05]
        if not ahead:
            return None
        q = min(ahead, key=lambda q: abs(q[1] - ref_y))
        return q[0], q[1]
    pts = [(0.0, 0.0)] + [(q[0], q[1]) for q in chain]
    left = look
    for p, q in zip(pts, pts[1:]):
        seg = math.dist(p, q)
        if seg >= left:
            t = left / seg
            return p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])
        left -= seg
    return pts[-1]


class StuckMonitor:
    """명령은 움직이라는데 (v > v_min 또는 |w| > w_min) check_s 동안 move_m·rot_deg 도 못 움직이면 막힘."""

    def __init__(self, check_s=1.0, move_m=0.01, rot_deg=2.0, v_min=0.03, w_min=0.3):
        self.check_s, self.move_m, self.rot = check_s, move_m, math.radians(rot_deg)
        self.v_min, self.w_min = v_min, w_min
        self.mon = None

    def reset(self):
        self.mon = None

    def update(self, now, pos, yaw, v, w):
        """막혔으면 설명 문자열, 아니면 None."""
        if not (v > self.v_min or abs(w) > self.w_min):
            self.mon = None
            return None
        m = self.mon
        if m is None or math.dist(pos, m[1]) > self.move_m or abs(wrap(yaw - m[2])) > self.rot:
            self.mon = (now, pos, yaw, v, w)
            return None
        if now - m[0] > self.check_s:
            why = (f'명령 v {m[3]:+.2f} w {m[4]:+.2f} 인데 {now - m[0]:.1f} s 동안 {math.dist(pos, m[1]) * 100:.1f} cm, '
                   f'{math.degrees(abs(wrap(yaw - m[2]))):.1f}° 만 움직임')
            self.mon = None
            return why
        return None


class RetryCounter:
    """같은 자리 (첫 자리에서 radius 안) 재시도 횟수."""

    def __init__(self, radius=0.2):
        self.radius = radius
        self.at = None
        self.count = 0

    def reset(self):
        self.at, self.count = None, 0

    def add(self, pos):
        if self.at is not None and math.dist(pos, self.at) < self.radius:
            self.count += 1
        else:
            self.at, self.count = pos, 1
        return self.count
