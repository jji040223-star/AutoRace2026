"""정지선·끝선 추적 (M3·M1 의 mission_manager line_update / line_remaining 과 같은 방식) — parking_controller 가 쓴다.

측정 (차 중심 → 선 가까운 끝 거리) 을 영상 찍은 순간의 odom 자세와 짝지어 선 위치 (odom) 로 바꿔 두고, 화면 아래로 사라진 뒤는
오도메트리로 남은 거리를 잰다. 처음 잡은 선만 따라간다 — 그 선이 있어야 할 자리와 match_m 넘게 다른 측정은 다른 선이라 버린다.
"""
import math


class StopLine:
    def __init__(self, match_m=0.10):
        self.match_m = match_m
        self.pt = None                     # (x, y, 잰 때의 진행 방향)

    def update(self, pose, dist):
        """pose = 측정 순간 odom (x, y, yaw), dist = 차 중심 → 선 [m]. 받아들였으면 True."""
        x, y, yaw = pose
        pt = (x + dist * math.cos(yaw), y + dist * math.sin(yaw), yaw)
        if self.pt is not None and math.dist(pt[:2], self.pt[:2]) > self.match_m:
            return False
        self.pt = pt
        return True

    def remaining(self, pose):
        """차 중심 → 선, 잰 방향으로 [m] (지나면 음수). 아직 못 봤으면 None."""
        if self.pt is None:
            return None
        lx, ly, yaw = self.pt
        return (lx - pose[0]) * math.cos(yaw) + (ly - pose[1]) * math.sin(yaw)
