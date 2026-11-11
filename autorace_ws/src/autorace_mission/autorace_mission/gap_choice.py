"""M7 틈 고르기 (m7_gap_mode): '몇 번 차로로 갈지' 대신 '다음 장애물 줄의 어느 빈 틈으로 갈지'.

차로 번호 판단은 격자 (차로 위치) 가 반 차로 틀리면 두 상자가 세 차로를 다 막은 것처럼 보여 '갈 곳 없음' 이 되고
(실차 11-10 ack39: 130 s 갇힘), 늦게 보인 상자로 목표 차로를 좌↔우 번갈아 바꿨다 (ack37·38). 여기서는
  - 다음 줄 (앞면이 비슷한 상자 묶음, 뎁스 기억 + 라이다 점) 의 빈 옆 구간을 세 차로 전 폭에서 구하고,
  - 그 안에서 지금 옆 위치로부터 가장 적게 옮기는 점을 목표로 하되,
  - 차가 이미 향한 쪽 틈을 먼저 (아커만은 반대로 틀기가 비쌈), 그다음 줄 틈과 이어지는 쪽, 갈림길 쪽 순으로,
  - 한 번 고른 틈은 그 틈이 막히지 않는 한 유지 (실패 횟수로 뒤집지 않음).
ROS 없이 부를 수 있는 순수 함수 — 실차 로그 재생 시험 (tools/simcmp/gap_replay.py) 에서도 같은 함수를 씀.

좌표: s = 차로 방향 앞, l = 왼쪽 + (mission_manager 의 M7 차로 좌표). 상자 = (s0, s1, l0, l1).
"""
import math


def s_curve_len(dl, radius):
    """옆으로 dl 옮기는 S 자 (반경 radius 원호 둘) 에 필요한 앞 길이."""
    dl = abs(dl)
    if dl <= 1e-6:
        return 0.0
    if dl >= 2 * radius:
        return 2 * radius + (dl - 2 * radius)          # 반원 넘는 옮김은 없다고 봄 (차로 둘 = 1.04 m > 2R) — 대략
    c = 1.0 - dl / (2 * radius)
    return 2 * radius * math.sqrt(max(0.0, 1.0 - c * c))


def rows_ahead(boxes, s_from, horizon, row_tol):
    """s_from 보다 앞면이 앞인 상자를 앞면 순으로 줄로 묶음 (앞면 차이 row_tol 안 = 같은 줄)."""
    ahead = sorted((b for b in boxes if b[0] > s_from and b[0] < s_from + horizon), key=lambda b: b[0])
    rows = []
    for b in ahead:
        if rows and b[0] - rows[-1][0][0] <= row_tol:
            rows[-1].append(b)
        else:
            rows.append([b])
    return rows


def free_intervals(row, lo, hi, clear):
    """줄의 상자들 사이 차 가운데가 갈 수 있는 옆 구간들 [(a, b)] — 상자 옆면에서 clear 안쪽, [lo, hi] 안."""
    occ = sorted((b[2] - clear, b[3] + clear) for b in row)
    out, cur = [], lo
    for a, b in occ:
        if a > cur:
            out.append((cur, min(a, hi)))
        cur = max(cur, b)
        if cur >= hi:
            break
    if cur < hi:
        out.append((cur, hi))
    return [(a, b) for a, b in out if b - a >= 0.0]


def choose_gap(boxes, sc, sf, lc, psi, lo, hi, p, prev=None, goal_l=None):
    """다음 줄 틈 목표.  p: dict (clear, radius, horizon, row_tol, buf, w_dir, w_next, w_goal, hold_bonus, dir_deg).
    prev: 지난번 결과 dict 또는 None.  goal_l: 갈림길 쪽 끝 차로 가운데 l (없으면 None).
    반환: dict(lt, gap, row_s, cands, why) 또는 None (앞에 줄이 없음)."""
    rows = rows_ahead(boxes, sf - 0.05, p['horizon'], p['row_tol'])
    if not rows:
        return None
    row = rows[0]
    row_s = min(b[0] for b in row)
    d = max(0.0, row_s - sf)                       # 차 앞 → 줄 앞면
    gaps = free_intervals(row, lo, hi, p['clear'])
    nxt = free_intervals(rows[1], lo, hi, p['clear']) if len(rows) > 1 else None
    cands = []
    w_all = p.get('w_goal_all', 0.0)
    pts = []
    for a, b in gaps:
        buf = min(p['buf'], (b - a) / 2)
        pts.append((a, b, buf, min(max(lc, a + buf), b - buf)))       # 그 틈에서 지금 옆과 가장 가까운 점
        if w_all > 0 and goal_l is not None:
            # 같은 틈 안 갈림길 쪽 점도 후보 (옛 차로 방식의 '갈림길 쪽 옆 차로가 비면 미리 옮김' — 시뮬 g1_h1_gap:
            # 틈만 따라 왼쪽 끝으로 가서 B 로 두 칸 옮길 거리가 없었음)
            tg = min(max(goal_l, a + buf), b - buf)
            if abs(tg - pts[-1][3]) > 0.05:
                pts.append((a, b, buf, tg))
    for a, b, buf, t in pts:
        dl = t - lc
        need = s_curve_len(dl, p['radius'])
        # 차가 이미 그쪽으로 향해 있으면 필요한 길이가 줆 (향한 만큼 옆으로 감), 반대면 늚
        toward = math.copysign(1.0, dl) * math.sin(psi) if abs(dl) > 1e-3 else 0.0
        need = max(0.0, need - toward * d)
        cost = abs(dl)
        if abs(dl) > 0.05 and abs(psi) > math.radians(p['dir_deg']) and toward < 0:
            cost += p['w_dir'] * abs(psi)          # 아커만: 향한 쪽 반대로 가는 틈은 비쌈 (ack37·38 좌↔우)
        if nxt is not None:
            # 다음 줄 틈과 이어지는가: t 에서 다음 줄 가장 가까운 틈까지 옆 거리
            dn = min((0.0 if a2 <= t <= b2 else min(abs(t - a2), abs(t - b2))) for a2, b2 in nxt) if nxt else 1.0
            cost += p['w_next'] * dn
        elif goal_l is not None:
            cost += p['w_goal'] * abs(t - goal_l)
        if w_all > 0 and goal_l is not None:
            cost += w_all * abs(t - goal_l)        # 다음 줄이 있어도 갈림길 쪽 (m7_gap_w_goal_all, 기본 0 = 안 씀)
        feas = need <= d + p['slack']
        if not feas:
            cost += 10.0 + (need - d)              # 못 들어가는 틈은 맨 뒤 (그래도 다 못 들어가면 덜 못 들어가는 것)
        if prev is not None and abs(prev['row_s'] - row_s) < p['row_tol'] and a - 1e-3 <= prev['lt'] <= b + 1e-3:
            if w_all > 0 and goal_l is not None:
                if abs(t - prev['lt']) < 0.08:
                    cost -= p['hold_bonus']        # 고른 점 유지 (갈림길 쪽 점 후보가 있을 땐 점에 보너스, 점을 옮기지 않음)
            else:
                cost -= p['hold_bonus']            # 고른 틈 유지
                t = min(max(prev['lt'], a + buf), b - buf)
        cands.append((cost, t, (a, b), feas, need))
    if not cands:
        return {'lt': None, 'gap': None, 'row_s': row_s, 'cands': [], 'why': '빈 틈 없음', 'd': d}
    cands.sort(key=lambda c: c[0])
    best = cands[0]
    return {'lt': best[1], 'gap': best[2], 'row_s': row_s, 'cands': cands, 'd': d,
            'why': ('들어감' if best[3] else f'S 자 {best[4]:.2f} m > 앞 {d:.2f} m (후진 필요)')}
