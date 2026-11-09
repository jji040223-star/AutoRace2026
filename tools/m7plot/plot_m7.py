# 실차 M7: 경로 (bag 복구 odom 거리 + IMU 방향, 로그의 차 s/l 로 맞춤) + 기억한 장애물 + 라이다 점 + 막힘 위치
import sys, re, pickle, math
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'DejaVu Sans']
log, bagpk, scanpk, out = sys.argv[1:5]; snaps = [float(x) for x in sys.argv[5:]]
B = 1793960000.0
rx = re.compile(r'\[(17939\d+\.\d+)\]')
L = [l for l in open(log, errors='ignore').read().splitlines() if '[mission_manager]' in l]
t_entry = None; dumps = []; stuck = []; lanes_at = []
for l in L:
    m = rx.search(l)
    if not m: continue
    t = float(m.group(1))
    if 'LANE → M7_AVOID' in l and t_entry is None: t_entry = t
    d = re.search(r'차 s ([-\d.]+) l ([-+\d.]+), 차로 가운데 l \[([^\]]*)\].*기억 \d+ 개: (.*)$', l)
    if d:
        lc = [float(v.split(':')[1]) for v in d.group(3).split(', ')]
        bx = [tuple(map(float, b)) for b in re.findall(r's ([-\d.]+)~([-\d.]+) l ([-+\d.]+)~([-+\d.]+)', d.group(4))]
        dumps.append((t, float(d.group(1)), float(d.group(2)), lc, bx))
    if '[M7] 막힘' in l: stuck.append(t)
bag = pickle.load(open(bagpk, 'rb')); od = np.array(bag['odom']); im = np.array(bag['imu'])
# 경로: 거리 = 휠 odom 속도 (부호 포함) 적분, 방향 = IMU 적분 (멈춘 동안 평균으로 치우침 뺌) 또는 휠 방향 ÷ 0.77 중 로그에 더 잘 맞는 것
t0, t1 = t_entry - 3, max(snaps) + 1
od = od[(od[:, 0] > t0 - 5) & (od[:, 0] < t1)]; im = im[(im[:, 0] > t0 - 5) & (im[:, 0] < t1)]
still = np.abs(np.interp(im[:, 0], od[:, 0], od[:, 4])) < 1e-3
bias = im[still, 1].mean() if still.sum() > 50 else 0.0
th_imu = np.interp(od[:, 0], im[:, 0], np.r_[0, np.cumsum((im[1:, 1] - bias) * np.diff(im[:, 0]))])
th_whl = np.unwrap(od[:, 3]); th_whl = (th_whl - th_whl[0]) / 0.77
dt = np.r_[0, np.diff(od[:, 0])]
def build(hd):
    P = np.zeros((len(od), 2))
    for k in range(1, len(od)):
        P[k] = P[k - 1] + od[k, 4] * dt[k] * np.array([math.cos(hd[k]), math.sin(hd[k])])
    return P
def fitR(P, hd):
    pt_ = lambda t: np.array([np.interp(t, od[:, 0], P[:, 0]), np.interp(t, od[:, 0], P[:, 1])])
    A = np.array([pt_(t_entry)] + [pt_(d[0]) for d in dumps]); Bq = np.array([[0, 0]] + [[d[1], d[2]] for d in dumps])
    ca, cb = A.mean(0), Bq.mean(0); H = (A - ca).T @ (Bq - cb); U_, S_, Vt = np.linalg.svd(H); R = (U_ @ Vt).T
    if np.linalg.det(R) < 0: Vt[1] *= -1; R = (U_ @ Vt).T
    T = cb - R @ ca
    return R, T, np.abs((A @ R.T + T) - Bq).max()
best = None
for name, hd_ in (('IMU', th_imu), ('휠÷0.77', th_whl)):
    P_ = build(hd_); R_, T_, f_ = fitR(P_, hd_)
    print(name, round(f_, 3))
    if best is None or f_ < best[0]: best = (f_, name, P_, hd_, R_, T_)
fit, hname, P, hd, R, T = best
pt = lambda t: np.array([np.interp(t, od[:, 0], P[:, 0]), np.interp(t, od[:, 0], P[:, 1])])
hh = lambda t: np.interp(t, od[:, 0], hd)
rot = math.atan2(R[1, 0], R[0, 0])
Pm = P @ R.T + T
scans = pickle.load(open(scanpk, 'rb'))
fig, axs = plt.subplots(len(snaps), 1, figsize=(11, 4.3 * len(snaps)), facecolor='#2b2d31')
axs = np.atleast_1d(axs)
for ax, ts in zip(axs, snaps):
    ax.set_facecolor('#3a3c40')
    d = min(dumps, key=lambda q: abs(q[0] - ts)); lc = d[3]; W = 0.52
    top, bot = lc[0] + W / 2, lc[-1] - W / 2
    smin, smax = -0.8, 4.3
    ax.add_patch(Rectangle((smin, bot), smax - smin, top - bot, color='#222326', zorder=0))
    for y in (top, bot): ax.plot([smin, smax], [y, y], color='#e8a92b', lw=2.5, zorder=1)
    for k in range(len(lc) - 1):
        y = (lc[k] + lc[k + 1]) / 2
        ax.plot([smin, smax], [y, y], color='#dddddd', lw=1.5, ls=(0, (6, 6)), zorder=1)
    for k, y in enumerate(lc): ax.text(smax + 0.05, y, f'차로 {k}', color='#a8e6a3', va='center', fontsize=10)
    for b in d[4]:
        c = (b[2] + b[3]) / 2; inside = bot <= c <= top
        ax.add_patch(Rectangle((b[0], b[2]), b[1] - b[0], b[3] - b[2], fill=False, ec='#ff3df5' if inside else '#888888',
                               lw=2, ls='--' if inside else ':', zorder=4))
    m = (od[:, 0] >= t_entry) & (od[:, 0] <= ts)
    ax.plot(Pm[m, 0], Pm[m, 1], color='#ff9a1f', lw=2.2, zorder=5)
    for st in stuck:
        if st <= ts:
            q = R @ pt(st) + T; ax.plot(q[0], q[1], 'x', color='#ff3030', ms=11, mew=2.5, zorder=6)
    q = R @ pt(ts) + T; h = hh(ts) + rot
    corners = [(0.161, 0.109), (0.161, -0.109), (-0.161, -0.109), (-0.161, 0.109)]
    ax.add_patch(Polygon([(q[0] + x * math.cos(h) - y * math.sin(h), q[1] + x * math.sin(h) + y * math.cos(h)) for x, y in corners],
                         fill=False, ec='white', lw=2, zorder=7))
    sk = min(scans, key=lambda s: abs(s - ts))
    if abs(sk - ts) < 0.3:
        a0, da, rr = scans[sk]; rr = np.array(rr); aa = a0 + da * np.arange(len(rr)); okr = np.isfinite(rr) & (rr > 0.05) & (rr < 2.5)
        hs = hh(sk) + rot; qs = R @ pt(sk) + T
        x = rr[okr] * np.cos(aa[okr]) + 0.103; y = rr[okr] * np.sin(aa[okr])
        ax.scatter(qs[0] + x * math.cos(hs) - y * math.sin(hs), qs[1] + x * math.sin(hs) + y * math.cos(hs), s=6, c='#3fd0ff', zorder=6)
    ax.set_xlim(smin, smax + 0.5); ax.set_ylim(bot - 0.25, top + 0.25); ax.set_aspect('equal')
    ax.set_title(f'M7 진입 뒤 {ts - t_entry:.1f} s — 차 s {d[1]:.2f} l {d[2]:+.2f} (기억 {len(d[4])} 개)', color='white', fontsize=12, loc='left')
    ax.tick_params(colors='#aaaaaa'); [sp_.set_color('#555') for sp_ in ax.spines.values()]
    ax.set_xlabel('s 진행 방향 [m] →', color='#aaaaaa'); ax.set_ylabel('l 왼쪽 [m]', color='#aaaaaa')
fig.text(0.01, 0.005, '주황 경로   분홍 점선 = 기억한 장애물   회색 점선 = 세 차로 밖이라 무시   하늘색 = 그 순간 라이다 점   빨강 X = 막힘·후진   흰 사각형 = 차',
         color='#dddddd', fontsize=10)
fig.text(0.99, 0.005, f'경로 (방향 {hname}) ↔ 로그 차 위치 오차 최대 {fit * 100:.1f} cm', color='#888888', fontsize=9, ha='right')
plt.tight_layout(rect=(0, 0.03, 1, 1)); plt.savefig(out, dpi=110, facecolor=fig.get_facecolor())
print('fit', round(fit, 3), 'bias', round(bias, 4))
