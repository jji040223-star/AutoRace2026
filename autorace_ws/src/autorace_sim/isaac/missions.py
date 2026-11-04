# 원본: ~/LimoIsaacSIM/autorace_2026/autorace_2026_python/missions.py — 두 곳이 같아야 한다 (고치면 같이).
"""미션 시설물 중 시간에 따라 바뀌는 것: 신호등 색 (M1), 갈림길 A/B 표지판 (M2), 방해 광고판 화면 (M1).

미션 USD(tools/build_mission_usd.py 생성)의 customData autorace:switch 에 바꿀 prim 이 적혀 있고, 여기서는 보임(visibility)만 바꾼다.
시간은 시뮬레이션 시간(World.current_time = /clock)이다 — 시뮬이 실시간보다 느려도 로봇이 보는 주기는 설정값 그대로.
갈림길 표지판: branch_period 0 이고 branch_random 이면 리셋 (= 한 판 시작) 할 때 A/B 를 무작위로 정해 그 판 동안 고정한다
(실제 대회: 주행마다 무작위, 한 주행 안에서는 안 바뀜). branch_period > 0 이면 예전처럼 주기마다 바꾼다 (시험용).
"""
import random

from pxr import UsdGeom

LIGHT_ORDER = ("red", "green", "yellow")    # 적 → 녹 → 황 → 적 (도로교통 3색 신호)
TAG = "[autorace_2026]"


class MissionSwitcher:
    def __init__(self, stage, root_path, switch, cfg):
        """switch: 미션 USD customData autorace:switch, cfg: extension settings 에서 읽은 주기 (AutoRace2026.__init__)."""
        def prim(rel):
            return UsdGeom.Imageable(stage.GetPrimAtPath(f"{root_path}/{rel}"))

        self._lamps = {c: prim(p) for c, p in switch.get("traffic_light", {}).items()}
        self._faces = {k: prim(p) for k, p in switch.get("branch_sign", {}).items()}
        self._slides = [prim(p) for p in switch.get("ad_slides", [])]
        self._cfg = dict(cfg)
        if self._faces and self._cfg["branch_start"] not in self._faces:
            self._cfg["branch_start"] = sorted(self._faces)[0]
        start = cfg["light_start"] if cfg["light_start"] in LIGHT_ORDER else "red"
        i = LIGHT_ORDER.index(start)
        self._phases = [(c, cfg["light_s"][c]) for c in LIGHT_ORDER[i:] + LIGHT_ORDER[:i] if cfg["light_s"][c] > 0]
        self._rng = random.Random()
        self.light_override = None  # "red" | "yellow" | "green" 이면 주기 대신 그 색 고정 (시험용, 리셋해도 유지)
        self.reset()

    def reset(self):
        self._shown = {}            # 무엇이 보이는지 (바뀔 때만 USD 를 쓴다)
        self._branch_k = None
        if self._cfg["branch_random"] and self._cfg["branch_period"] <= 0 and self._faces:
            self._branch = self._rng.choice(sorted(self._faces))     # 이 판의 A/B — 다음 리셋까지 고정
        else:
            self._branch = self._cfg["branch_start"]
        self.update(0.0)

    def update(self, t):
        self._show("light", self._lamps, self.light_override or self._light_at(t), t)
        self._show("branch", self._faces, self._branch_at(t), t)
        self._show("slide", dict(enumerate(self._slides)), self._slide_at(t), t, log=False)

    def light_now(self, t):
        return self.light_override or self._light_at(t)

    def _light_at(self, t):
        if not self._phases:
            return None             # 모든 색 0 초 → 다 꺼짐
        tt = t % sum(d for _, d in self._phases)
        for color, d in self._phases:
            if tt < d:
                return color
            tt -= d
        return self._phases[-1][0]

    def _branch_at(self, t):
        period = self._cfg["branch_period"]
        if period <= 0:
            return self._branch
        k = int(t // period)
        if k != self._branch_k:
            if k == 0:
                self._branch = self._cfg["branch_start"]
            elif self._cfg["branch_random"]:
                self._branch = self._rng.choice(sorted(self._faces))
            else:
                self._branch = "B" if self._branch == "A" else "A"
            self._branch_k = k
        return self._branch

    def _slide_at(self, t):
        period = self._cfg["slide_period"]
        if not self._slides:
            return None
        return int(t // period) % len(self._slides) if period > 0 else 0

    def _show(self, name, prims, key, t, log=True):
        if self._shown.get(name, object()) == key:
            return
        for k, p in prims.items():
            if p:
                p.MakeVisible() if k == key else p.MakeInvisible()
        self._shown[name] = key
        if log:
            print(f"{TAG} t={t:7.2f}s {'신호등' if name == 'light' else '갈림길 표지판'} → {key}")
