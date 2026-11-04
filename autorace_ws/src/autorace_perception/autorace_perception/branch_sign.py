"""M2 갈림길 표지판 A/B 보조 판별 (yolo_detector 가 YOLO 결과와 합친다).

YOLO (turn_a / turn_b) 는 실물 사진으로 학습해서 시뮬 렌더 표지판은 1 m 앞 (지름 53 px) 에서도 못 잡았다 (2026-11-01 측정).
그래서 표지판 모양을 직접 본다: 파란 원 (HSV) → 원 안의 흰 무늬 (화살표 + 글자) 를 32×32 로 줄여 A·B 그림과 정규화 상관.
  시뮬: 4 m 앞 지름 14 px 부터 맞는 쪽 0.6–0.95, 반대쪽 ≤ 0.47. 주차 표지판 (파란 원 P) 은 A 0.46 / B 0.25.
  화살표 방향이 반대인 두 그림이라 좌우가 뒤집히면 결과도 뒤집힌다 (좌우 반전 영상 금지).
"""
import math

import cv2
import numpy as np

N = 32


def white_pattern(bgr):
    """원 안의 흰 정도 (채도 낮고 밝을수록 1) 를 N×N 으로 줄여 평균 0, 표준편차 1 로 맞춘 벡터."""
    b = cv2.resize(bgr, (N, N), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(b, cv2.COLOR_BGR2HSV).astype(np.float32)
    w = np.clip((150 - hsv[..., 1]) / 100, 0, 1) * np.clip((hsv[..., 2] - 100) / 80, 0, 1)
    yy, xx = np.mgrid[0:N, 0:N]
    circ = ((xx - N / 2 + 0.5) / (N / 2)) ** 2 + ((yy - N / 2 + 0.5) / (N / 2)) ** 2 < 0.8 ** 2
    v = w[circ]
    return (v - v.mean()) / (v.std() + 1e-6)


class BranchSign:
    def __init__(self, templates, blue_lo=(100, 110, 50), blue_hi=(130, 255, 255), min_px=12, min_fill=0.35,
                 border_px=2):
        """templates: {라벨: 그림 경로} (예: {'turn_a': '.../sign_branch_A.png', 'turn_b': ...})."""
        self.tpl = {}
        for label, path in templates.items():
            img = cv2.imread(path)
            if img is None:
                raise FileNotFoundError(path)
            self.tpl[label] = white_pattern(img)
        self.blue_lo, self.blue_hi = tuple(blue_lo), tuple(blue_hi)
        self.min_px, self.min_fill, self.border_px = min_px, min_fill, border_px

    def circles(self, bgr):
        """파란 원 후보 [(x1, y1, x2, y2)] — 가로세로 비 0.75–1.33, 원 넓이 대비 파란 비율 ≥ min_fill, 화면 끝에 안 걸린 것."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        blue = cv2.inRange(hsv, self.blue_lo, self.blue_hi)
        n, _, st, _ = cv2.connectedComponentsWithStats(blue)
        H, W = blue.shape
        out = []
        for x, y, w, h, a in st[1:]:
            if min(w, h) < self.min_px or not 0.75 < w / h < 1.33:
                continue
            if a / (math.pi * w * h / 4) < self.min_fill:
                continue
            if x < self.border_px or y < self.border_px or x + w > W - self.border_px or y + h > H - self.border_px:
                continue                    # 잘린 원은 무늬가 어긋나 A/B 가 애매해진다
            out.append((int(x), int(y), int(x + w), int(y + h)))
        return out

    def classify(self, bgr, box):
        """원 상자 → {라벨: 상관} (−1 ~ 1)."""
        x1, y1, x2, y2 = box
        p = 2 if x2 - x1 < 30 else 1        # 작은 원은 테두리 흰 띠가 잘리지 않게 조금 넓게
        crop = bgr[max(0, y1 - p):y2 + p, max(0, x1 - p):x2 + p]
        v = white_pattern(crop)
        return {k: float((v * t).mean()) for k, t in self.tpl.items()}
