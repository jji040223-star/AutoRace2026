"""M8 라바콘 배치 — /sim/reset 때 고른 배치로 라바콘 14 개 (7 쌍) 를 옮긴다 (run_sim.py).

미션 USD (LimoIsaacSIM/autorace_2026 config/missions.yaml cones) 의 라바콘은 Cone_00, 01 = 첫 쌍 (왼쪽, 오른쪽) … 순서.
배치는 통로 가운데선 (진행 방향 heading 으로 pitch 마다) 과 그 옆 half_gap 에 콘을 두는 같은 방식에 모양만 바꾼다.
  default  : USD 그대로 (가운데선이 sway 0.12 m 로 한 번 휘는 S)
  straight : 직선 두 줄
  curve    : 한쪽으로 휘는 곡선 통로 (끝에서 0.15 m 옆으로)
  s        : 더 큰 S (sway 0.16 m)
  narrow   : 곧은 통로가 점점 좁아짐 (half_gap 0.27 → 0.21 m — 콘 바닥판 사이 0.31 m, 차 폭 0.22 m). 흔들림까지 겹치면 가운데선을
             그대로 따라가도 여유 1.3 cm 라 S 는 s 배치에서 따로 본다
  wide     : 라바콘 간격이 조금 넓음 (pitch 0.33 m). 끝 쌍은 다른 배치와 같은 자리 (왼쪽 직선 안 — 그대로 늘리면 끝 콘이 커브 입구까지 감)
  jitter   : default 에 콘마다 ±3 cm 흔들기 (리셋마다 새로)
  random   : 리셋마다 위 중 하나
  curvy_m  : (random 에 안 들어감) curvy 를 좌우로 뒤집은 것 (처음에 오른쪽으로 휨)
  curvy    : (random 에 안 들어감) 통로 폭은 default 와 같고 (half_gap 0.25) 더 크게 휨 (sway 0.20, 주기 1.5 m) — 곧게 가면 콘에 닿음
  tight    : (random 에 안 들어감) 쌍 간격 좁고 (half_gap 0.20) 크게 휨 (sway 0.18) — 곧게 가면 안쪽 콘에 닿음.
             가운데선을 따라가면 차 옆 (0.11) + 바닥판 (0.055) 뒤 여유 3.5 cm
"""
import math
import random

NAMES = ("default", "straight", "curve", "s", "narrow", "wide", "jitter")


def positions(name, base, rng=None):
    """base: dict(start=[x, y], heading=deg, pairs, pitch, half_gap, sway, sway_period). [(x, y)] × 2·pairs (왼쪽, 오른쪽 순)."""
    rng = rng or random.Random()
    b = dict(base)
    hg0 = hg1 = b["half_gap"]
    bend = 0.0
    if name == "straight":
        b["sway"] = 0.0
    elif name == "curve":
        b["sway"], bend = 0.0, 0.15
    elif name == "s":
        b["sway"] = 0.16
    elif name == "curvy":
        # 11-09: 0.20 / 1.5 → 0.18 / 1.8 — 통로 가운데선 최소 반경 0.28 m 는 아커만 (최소 반경 0.47 m) 으로 못 지남 (여유 −0.5 cm).
        # 0.18 / 1.8 = 가운데선 반경 0.46 m, 여유 약 6 cm. 공식 (default 0.12 / 1.8) 보다 1.5 배 휨 (악조건 시험)
        b["sway"], b["sway_period"] = 0.18, 1.8
    elif name == "curvy_m":
        b["sway"], b["sway_period"] = -0.18, 1.8
    elif name == "tight":
        b["sway"], hg0, hg1 = 0.18, 0.20, 0.20
    elif name == "narrow":
        b["sway"], hg0, hg1 = 0.0, 0.27, 0.21
    elif name == "wide":
        n_ = int(b["pairs"]) - 1
        h_ = math.radians(b["heading"])
        back = n_ * (0.33 - b["pitch"])
        b["start"] = [b["start"][0] - back * math.cos(h_), b["start"][1] - back * math.sin(h_)]
        b["pitch"] = 0.33
    hx, hy = math.cos(math.radians(b["heading"])), math.sin(math.radians(b["heading"]))
    nx, ny = -hy, hx                                                     # 왼쪽
    n = int(b["pairs"])
    span = max(1e-6, (n - 1) * b["pitch"])
    out = []
    for i in range(n):
        s = i * b["pitch"]
        off = b["sway"] * math.sin(2 * math.pi * s / b["sway_period"]) + bend * (s / span) ** 2
        hg = hg0 + (hg1 - hg0) * s / span
        cx, cy = b["start"][0] + hx * s + nx * off, b["start"][1] + hy * s + ny * off
        out += [(cx + nx * hg, cy + ny * hg), (cx - nx * hg, cy - ny * hg)]
    if name == "jitter":
        out = [(x + rng.uniform(-0.03, 0.03), y + rng.uniform(-0.03, 0.03)) for x, y in out]
    return out
