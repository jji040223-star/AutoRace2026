"""화면용 시뮬 설정을 무작위로: 3차선 정차 LIMO 7 대 (줄마다 1–2 대) + 갈림길 표지판 A/B.
지나갈 길 보장: 들어가는 가운데 차로 (1) 에서 시작해, 앞 줄에서 어느 빈 차로를 골랐어도 다음 줄 빈 차로까지 한 칸 안.
차로를 옮겨야 하는 줄 사이는 0.9 m 이상 (기본 배치와 같음), 같은 차로로 곧게 가면 0.75 m 이상.
마지막 줄은 x 5.15 보다 앞 (갈림길 A·B 입구를 안 막음), 마지막 줄 가운데 차로는 비움 (갈림길 표지판 앞).
사용: python3 gen_layout.py <출력 yaml>   → 고른 배치를 한 줄로 찍음"""
import random
import sys

LANES = [0.42, 0.94, 1.46]                # 차로 0 (왼쪽) · 1 · 2 의 y (진행 방향 −x 기준 왼쪽이 작은 y)
r = random.Random()


LAST_X = 5.15        # 마지막 줄은 이보다 앞 (기본 배치와 같음) — 분리대 (x 4.28) 앞 갈림길 A·B 입구를 막지 않게


def make():
    counts = [2, 2, 1, 2]
    r.shuffle(counts)
    rows = [r.sample(range(3), c) for c in counts]
    if 1 in rows[-1]:
        return None          # 마지막 줄 가운데 차로 = 갈림길 표지판 (x 4.2, 가운데) 바로 앞 → 막지 않음
    # 갈 수 있는 차로 집합을 앞 줄부터 따라감 (한 줄 지날 때 한 칸까지 옮김)
    reach, path_sets, need_change = {1}, [], []
    for occ in rows:
        free = {i for i in range(3) if i not in occ}
        # 앞 줄에서 어느 빈 차로를 골랐어도 다음 줄 빈 차로까지 한 칸 안이어야 함
        if any(not any(abs(f - p) <= 1 for f in free) for p in reach):
            return None
        nxt = {f for f in free if any(abs(f - p) <= 1 for p in reach)}
        need_change.append(not reach <= free)
        reach = nxt
        path_sets.append(sorted(nxt))
    gaps = [r.uniform(0.90, 1.00) if need_change[k] else r.uniform(0.75, 1.00) for k in range(1, len(rows))]
    if 7.85 - sum(gaps) < LAST_X:
        return None          # 줄 사이를 다 넣으면 마지막 줄이 갈림길 입구까지 감
    x, out = 7.85, []
    for k, occ in enumerate(rows):
        if k > 0:
            x -= gaps[k - 1]
        out += [(round(x + r.uniform(-0.03, 0.03), 2), LANES[i], i) for i in occ]
    return out, rows, path_sets


while True:
    res = make()
    if res is not None:
        break
out, rows_occ, path_sets = res
sign = r.choice('AB')
cone = r.choice(['curvy', 'curvy_m'])
with open(sys.argv[1], 'w') as f:
    f.write('sim2d:\n  ros__parameters:\n    cone_layout: ' + cone + '\n')
    f.write(f'    branch_sign: {sign}\n')
    f.write('    static_layout_xy: [' + ', '.join(f'{a}, {b}' for a, b, _ in out) + ']\n')
xs = sorted({round(a, 1) for a, _, _ in out}, reverse=True)
print(f'표지판 {sign} | 라바콘 {"왼쪽 먼저" if cone == "curvy" else "오른쪽 먼저"} | 줄마다 막힌 차로 (0 왼쪽·1 가운데·2 오른쪽): ' + ' / '.join(f'{sorted(o)}' for o in rows_occ)
      + ' | 지나갈 수 있는 차로: ' + ' → '.join(str(p) for p in path_sets))
