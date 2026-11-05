"""슬라이딩 윈도우(SW) 차선 인식 — SEAME 2026 Hackathon (team chita) D-Racer-Kit 의 lane_detector.py 를 옮긴 것.

원본: https://github.com/mark2455/2026_SEAME_Hackathon (커밋 eef5ca8)
      D-Racer-Kit/src/perception/perception/lane_detector.py, Apache License 2.0 (../LICENSE.D-Racer-Kit)
원본 저장소 README 의 Out 코스 "상시 SW 코리도"(sw_out_always — 양쪽 실선, 중앙선 없음, DRIVE 전 구간 SW) 경로를
그대로 옮겼다. 알고리즘·파라미터 이름·기본값은 원본(lane_node.py declare_parameter) 과 같다.

  process()            ROI(하단) → BEV 워프 → HSV 마스크 → MORPH_OPEN
                       → 기본 밴드(guided band) → SW 코리도 (락 성공이면 밴드를 대체, 실패면 기본 밴드 = 원본의 안전 계약)
                       → 밴드 가중 합성 + look-ahead 블렌딩 + 중심 점프 가드 → offset [-1, 1] + EMA, 곡률
  _sw_corridor_bands   ① 정지선(가로줄) 행 제거 ② 시드 = 직전 피팅 하단 + 하단 히스토그램 피크
                       ③ 상자 체인 (상자당 이동 상한, 빈 상자 = 직전 추세 유지) ④ 2차 피팅 + 게이트 (상자 수·픽셀·스팬·잔차)
                       ⑤ 차폭 간격 페어 → 중점 / 단선 → 직전 중심 연속성으로 좌·우 경계 판정 ⑥ 관측 범위 밖 외삽 금지

바꾼 점:
  - D-Racer 트랙 전용 상태는 뺐다: 회전교차로 진입·탈출 창, 개구부(mouth)·raw 폴백, 방향·곡률 게이트, 노랑 래치,
    junction·fork·가로선·빨간 노면 검출, 갈림길 시야 마스킹. (Out 코스 경로에서는 dir=0 이라 원래 쓰이지 않거나 해당 미션 전용)
  - SW 입력: 원본 Out 코스는 흰 마스크만. AutoRace 트랙 경계선은 노랑이라 색은 use_white / use_yellow 로 고른다.
  - 색(HSV) 문턱과 BEV 점은 트랙·카메라마다 다르다 → LIMO Pro 와 AutoRace 트랙 값은 config/lane_follower.yaml.
  - 결과를 튜플 대신 LaneResult 로 돌려주고, 디버그 그림은 BEV 위에 상자·피팅·중심만 그린다.
"""
from dataclasses import dataclass, field

import math

import cv2
import numpy as np


@dataclass
class LaneResult:
    found: bool
    offset: float            # [-1, 1], + = 차선 중심이 화면 오른쪽 (원본과 같은 부호)
    num_lanes: int           # 가장 가까운 밴드에서 본 경계 수 (2 = 양쪽, 1 = 한쪽)
    curvature: float         # [-1, 1], 먼 밴드가 가까운 밴드보다 오른쪽이면 +
    sw_locked: bool          # 이번 프레임 SW 코리도 락 성공 여부
    sw_fail: str = None      # 락 실패 사유 (noseed / nofit / noband), 피팅 기각 사유는 sw_fitrej
    sw_fitrej: str = None
    lane_width_px: float = 0.0
    width_near_px: float = 0.0   # 이번 프레임 두 경계(SW 페어) 사이 폭 — BEV 맨 아래 / 맨 위. 페어가 없으면 0 (M6 지그재그 판단용)
    width_far_px: float = 0.0
    line_side: int = 0           # 이번 프레임 SW 로 잡은 선: 2 = 양쪽, -1 = 왼쪽만, +1 = 오른쪽만, 0 = 없음 (3차선 차로 번호용)
    line_slope: float = 0.0      # 잡은 선의 BEV 기울기 dx/dy (px/px, 맨 아래 행). 차로 방향 계산용 (선을 못 잡으면 0)
    line_resid_px: float = 0.0   # 이번 프레임 SW 피팅 잔차 (채택한 선 중 큰 쪽, 픽셀 평균 |x − 곡선|). 톱니 지그재그 선이면 커진다 (M6)
    stop_row: int = -1           # 정지선(흰 가로줄) 가까운 쪽 끝의 BEV 행. 없거나 화면 아래에 걸쳐 잘리면 -1
    stop_cov: float = 0.0        # 그 줄이 차로 폭을 덮은 비율
    bev_h: int = 0               # BEV 높이 [px] (행 → 거리 환산용)
    debug: np.ndarray = field(default=None, repr=False)


class SlidingWindowLaneDetector:
    def __init__(self, roi_top_ratio=0.35, min_pixels=40, single_line_offset=0.25,
                 mask_mode='hsv', bright_thresh=160, use_white=True, use_yellow=True,
                 white_hsv_lo=(0, 0, 180), white_hsv_hi=(179, 60, 255),
                 yellow_hsv_lo=(22, 45, 140), yellow_hsv_hi=(40, 255, 255),
                 num_bands=4, morph_kernel=3, width_ema=0.1, smooth_alpha=0.5,
                 use_birdeye=True,
                 birdeye_src_ratio=(0.2228, 0.342, 0.7425, 0.342, 0.9247, 0.965, 0.046, 0.965),
                 birdeye_dst_ratio=(0.20, 0.00, 0.80, 0.00, 0.80, 1.00, 0.20, 1.00),
                 use_guided_band=True, guide_margin_px=60, guide_margin_growth_px=10,
                 guide_min_pixels=20, guide_use_previous_frame=True, guide_max_jump_px=80,
                 use_lookahead_control=True, near_weight=0.7, lookahead_weight=0.3,
                 lookahead_band_index=-1, adaptive_lookahead=False,
                 curve_lookahead_weight=0.4, curve_lookahead_thresh=0.25,
                 center_jump_max_ratio=0.10, lane_width_init=192.0, lane_width_tol=0.0,
                 sw_enabled=True, sw_num_boxes=9, sw_box_margin=55, sw_max_shift=35,
                 sw_min_box_px=8, sw_min_boxes=3, sw_min_pixels=50, sw_max_resid_px=14.0,
                 sw_peak_min_px=40, sw_max_peaks=3, sw_cross_row_frac=0.45, sw_side_default=-1,
                 stop_line_enabled=False, stop_line_min_cov=0.9, stop_line_min_rows=10, stop_line_max_deg=15.0,
                 stop_line_bev_depth_m=0.0, stop_line_lane_m=0.52, sw_single_normal=False):
        self.roi_top_ratio = roi_top_ratio        # 프레임 하단 (1 - ratio) 부분만 사용
        self.min_pixels = min_pixels              # 한쪽을 신뢰하기 위한 최소 차선 픽셀 수
        self.single_line_offset = single_line_offset  # 차선 폭을 아직 모를 때의 폴백 offset
        self.mask_mode = mask_mode                # 'hsv' (흰|노랑) 또는 'gray' (밝기 문턱)
        self.bright_thresh = bright_thresh
        self.use_white, self.use_yellow = use_white, use_yellow
        self.white_hsv_lo, self.white_hsv_hi = np.array(white_hsv_lo), np.array(white_hsv_hi)
        self.yellow_hsv_lo, self.yellow_hsv_hi = np.array(yellow_hsv_lo), np.array(yellow_hsv_hi)
        self.num_bands = max(1, num_bands)        # 가로 look-ahead band 개수
        self.morph_kernel = morph_kernel          # MORPH_OPEN 크기 (0 = 끔)
        self.width_ema = width_ema                # 기억된 차선 폭이 적응하는 속도
        self.smooth_alpha = smooth_alpha          # offset EMA (1 = 스무딩 없음)
        self.use_birdeye = use_birdeye
        self.birdeye_src_ratio = list(birdeye_src_ratio)   # ROI 기준 [x1,y1, …, x4,y4] (좌상, 우상, 우하, 좌하)
        self.birdeye_dst_ratio = list(birdeye_dst_ratio)
        self.use_guided_band = use_guided_band
        self.guide_margin_px = guide_margin_px
        self.guide_margin_growth_px = guide_margin_growth_px
        self.guide_min_pixels = guide_min_pixels
        self.guide_use_previous_frame = guide_use_previous_frame
        self.guide_max_jump_px = guide_max_jump_px
        self.use_lookahead_control = use_lookahead_control
        self.near_weight, self.lookahead_weight = near_weight, lookahead_weight
        self.lookahead_band_index = lookahead_band_index
        self.adaptive_lookahead = adaptive_lookahead
        self.curve_lookahead_weight = curve_lookahead_weight
        self.curve_lookahead_thresh = curve_lookahead_thresh
        self.center_jump_max_ratio = center_jump_max_ratio   # 프레임 간 중심 이동 상한 (반 차폭 × 비율, 단선·소실만)
        self._lane_width = float(lane_width_init)            # 기억된 차선 폭 (BEV px)
        self._lane_width_init = float(lane_width_init)       # 폭 학습 하한 가드 기준
        self.lane_width_tol = float(lane_width_tol)          # >0 이면 폭 학습을 init × (1 ± tol) 안에서만 (0 = 원본: 하한 0.5 배만)
        self.sw_enabled = sw_enabled
        self.sw_num_boxes = sw_num_boxes          # 상자 개수
        self.sw_box_margin = sw_box_margin        # 상자 반폭 (px)
        self.sw_max_shift = sw_max_shift          # 상자당 이동 상한 (px) — 누운 실선·반대 가지 차단
        self.sw_min_box_px = sw_min_box_px        # 상자 적중 최소 픽셀
        self.sw_min_boxes = sw_min_boxes          # 유효 피팅 최소 적중 상자
        self.sw_min_pixels = sw_min_pixels        # 유효 피팅 최소 총 픽셀
        self.sw_max_resid_px = sw_max_resid_px    # 피팅 평균 잔차 상한 (px)
        self.sw_peak_min_px = sw_peak_min_px      # 시드 피크 최소 질량
        self.sw_max_peaks = sw_max_peaks          # 프레임당 피크 시드 수
        self.sw_cross_row_frac = sw_cross_row_frac  # 정지선 행 제거 문턱 (가로 점유율)
        self.sw_side_default = sw_side_default    # 단선 분류 폴백 (-1 = 우측 경계)
        # LIMO 추가: 단선 중심을 선의 법선 방향 (바닥 m 기준) 으로 반 차폭 떨어진 곳으로. False = 원본 (BEV 가로로 반 차폭)
        self.sw_single_normal = sw_single_normal
        self._prev_pair_mid = None                # 직전 프레임에서 고른 경계 쌍의 아래 가운데 x (쌍 고르기 연속성)
        # LIMO 추가 — 정지선 검출 (M3·M1). 차선용 use_white 와 따로, 흰색 마스크(노랑 뺌)에서 차로 폭을 가로지르는 줄을 찾는다
        self.stop_line_enabled = stop_line_enabled
        self.stop_line_min_cov = stop_line_min_cov    # 차로 가운데 0.9 폭 중 흰 픽셀 비율 (횡단보도 막대 ~0.56, 마름모 ~0.55 보다 높게)
        self.stop_line_min_rows = stop_line_min_rows  # 줄 두께 최소 행 수 (정지선 40 mm ≈ 25 행, 횡단보도 앞 가로선 30 mm ≈ 18 행)
        self.stop_line_max_deg = stop_line_max_deg    # 정지선 기울기 한도 [deg] (바닥 기준) — 이 안에서 기울기를 바꿔 가며 찾음
        self.stop_line_bev_depth_m = stop_line_bev_depth_m  # BEV 세로 = 바닥 far − near [m] (0 이면 축척 같다고 봄)
        self.stop_line_lane_m = stop_line_lane_m            # BEV 차로 폭 lane_width_init 이 바닥에서 몇 m
        self.reset()

    def reset(self, center_hint=None):
        """center_hint: 차가 차로 가운데쯤에 있다고 알 때 (미션 매니저의 윈도우 리셋) 화면 폭 대비 차로 중심 (0.5 = 가운데).
        첫 프레임의 단선 좌·우 판정에 직전 중심 대신 쓴다 — 없으면 단선을 늘 우측 경계로 봐서, 커브 입구에서 앞을 가로지르는
        왼쪽 경계 하나만 보이면 반대로 꺾는다 (M8 끝 시험: 왼쪽 직선 끝 (0.72, 4.25) 에서 새로 시작하면 트랙 밖)."""
        self._offset_ema = 0.0
        self._center_hint = center_hint
        self._prev_center = None   # 직전 프레임의 lane_center (guide 시드, 단선 좌·우 판정)
        self._sw_prev_fit = None   # 직전 유효 피팅 (a, b, c) — 다음 프레임 시드 연속성
        self._prev_pair_mid = None
        self._bev_key = None
        self._bev_matrix = None
        self._bev_warned = False

    # ---------------------------------------------------------------- 메인
    def process(self, bgr, draw_debug=True):
        h, w = bgr.shape[:2]
        if self._prev_center is None and self._center_hint is not None:
            self._prev_center = self._center_hint * w
            self._center_hint = None
        roi = bgr[int(h * self.roi_top_ratio):h, 0:w]
        if self.use_birdeye:
            roi = self._apply_birdeye(roi)   # 실패 시 원본 ROI 로 폴백
        mask = self._build_mask(roi)
        # 노이즈 정리: MORPH_OPEN 이 반사광 점과 점선의 작은 틈을 제거해 떠도는 픽셀이 중심을 잡아끌지 않게 한다.
        use_morph = bool(self.morph_kernel and self.morph_kernel > 1)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.morph_kernel,) * 2) if use_morph else None
        clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k) if use_morph else mask

        roi_h = clean.shape[0]
        band_h = max(1, roi_h // self.num_bands)
        band_windows = []
        if self.use_guided_band:
            bands, near_lanes, band_windows = self._guided_bands(clean, w, roi_h, band_h)
        else:
            bands, near_lanes = [], 0   # (center_x, weight, band_index), band 0 = 가장 가까움 (ROI 하단)
            for i in range(self.num_bands):
                y1 = roi_h - i * band_h
                y0 = max(0, roi_h - (i + 1) * band_h)
                if y1 - y0 < 2:
                    continue
                cx, nlanes = self._band_center(clean[y0:y1, :], w)
                if cx is not None:
                    bands.append((cx, float(self.num_bands - i), i))
                    if i == 0:
                        near_lanes = nlanes

        # 상시 SW 코리도 (원본 sw_out_always): 락 실패 프레임은 위 밴드 그대로 (안전 계약)
        sw_locked = False
        self._pair_w = None
        self._line_resid = 0.0
        self._line_side = 0
        self._line_slope = 0.0
        self._sw_fit_dbg, self._sw_fail, self._sw_fitrej = [], None, None
        if self.sw_enabled:
            sw = self._sw_corridor_bands(clean, w, roi_h, band_h)
            if sw is not None:
                bands, near_lanes, sw_boxes = sw
                band_windows = list(band_windows) + sw_boxes
                sw_locked = True

        mid = w / 2.0
        near_cx = la_cx = None
        if not bands:
            lane_found, num_lanes = False, 0
            lane_center, curvature = mid, 0.0
        else:
            lane_found = True
            num_lanes = near_lanes if near_lanes else 1
            tw = sum(wt for _, wt, _ in bands)
            lane_center = sum(cx * wt for cx, wt, _ in bands) / tw
            # curvature = 가까운 band → 먼 band 로 차선이 흘러간 정도 (정규화). 부호 = 커브 방향
            curvature = self._estimate_curvature(bands, w)
            # 부분 pure-pursuit: 가장 가까운 band 와 look-ahead band 중심을 블렌딩
            near_cx = bands[0][0]
            la_cx = self._lookahead_center(bands)
            if self.use_lookahead_control and la_cx is not None:
                nw, lw = self.near_weight, self.lookahead_weight
                if self.adaptive_lookahead and abs(curvature) >= self.curve_lookahead_thresh:
                    lw = self.curve_lookahead_weight
                    nw = max(0.0, 1.0 - lw)
                if nw + lw > 0.0:
                    lane_center = (nw * near_cx + lw * la_cx) / (nw + lw)
            # 최종 중심 연속성 가드: 프레임당 이동을 반 차폭 × 비율로 제한. 두 경계가 다 보이면(nl=2) 중점을 믿는다.
            if (self.center_jump_max_ratio > 0.0 and self._prev_center is not None
                    and self._lane_width > 0 and near_lanes != 2):
                j = self.center_jump_max_ratio * (self._lane_width / 2.0)
                d = lane_center - self._prev_center
                if abs(d) > j:
                    lane_center = self._prev_center + (j if d > 0 else -j)

        raw_offset = float(max(-1.0, min(1.0, (lane_center - mid) / (w / 2.0))))
        # 시간축 스무딩(EMA)으로 프레임 간 조향 떨림을 줄인다. 소실 프레임은 직전 값 유지.
        if lane_found:
            self._offset_ema = self.smooth_alpha * raw_offset + (1.0 - self.smooth_alpha) * self._offset_ema
            self._prev_center = lane_center
        offset = float(self._offset_ema)
        stop_row, stop_cov = self._detect_stop_line(roi, lane_center) if self.stop_line_enabled else (-1, 0.0)
        debug = (self._draw_debug(roi, clean, lane_center, offset, curvature, band_windows, near_cx, la_cx, sw_locked,
                                  stop_row)
                 if draw_debug else None)
        wn, wf = self._pair_w if self._pair_w else (0.0, 0.0)
        return LaneResult(lane_found, offset, num_lanes, curvature, sw_locked, self._sw_fail, self._sw_fitrej,
                          self._lane_width, wn, wf, self._line_side, self._line_slope, self._line_resid, stop_row, stop_cov, roi_h, debug)

    # ---------------------------------------------------------------- 정지선 (LIMO 추가)
    def _detect_stop_line(self, bev, center):
        """BEV 컬러에서 차로를 가로지르는 흰 줄 중 가장 가까운 것의 가까운 쪽 끝 행 (차로 가운데 열에서). (행, 덮은 비율) 또는 (-1, 0).
        흰색 = white_hsv 범위에서 노랑 범위를 뺀 것 (노란 선의 옅은 픽셀 제외). 차로 가운데 ±0.45 폭 (차로 폭은 학습값 대신
        lane_width_init — 지그재그에서 학습값이 흔들려도 같은 구간을 본다). 줄이 BEV 맨 아래에 걸치면 가까운 끝이 화면 밖이라 -1.
        방향: 줄 기울기를 ±stop_line_max_deg (바닥 기준) 안에서 2.5° 간격으로 바꿔 가며 그 기울기의 줄을 따라 덮은 비율을 잰다 —
        커브를 막 빠져나와 차가 틀어진 채 정지선을 보면 (M1, 7–18°) 4 cm 선이 가로 행 하나를 다 덮지 못한다. 한도보다 비스듬한
        흰 줄 (마름모·글자 획) 은 안 잡힌다. 횡단보도 막대는 진행 방향으로 길어 덮는 비율에서 걸러진다 (~0.56)."""
        h, w = bev.shape[:2]
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, self.white_hsv_lo, self.white_hsv_hi)
        white[cv2.inRange(hsv, self.yellow_hsv_lo, self.yellow_hsv_hi) > 0] = 0
        half = 0.45 * self._lane_width_init
        x0, x1 = int(max(0, center - half)), int(min(w, center + half))
        if x1 - x0 < 10:
            return -1, 0.0
        sub = white[:, x0:x1:2] > 0        # 열은 2 칸 간격 (기울기 13 가지를 다 재도 프레임당 수 ms)
        cols = np.arange(sub.shape[1])
        xc = (sub.shape[1] - 1) / 2.0
        # 바닥 기울기 → BEV 픽셀 기울기 (세로·가로 축척이 다르다: 행/m ÷ 열/m)
        aspect = (h / self.stop_line_bev_depth_m) / (self._lane_width_init / self.stop_line_lane_m) if self.stop_line_bev_depth_m > 0 else 1.0
        n_ang = int(self.stop_line_max_deg // 2.5)
        best = None
        for k in range(-n_ang, n_ang + 1):
            dy = np.round((cols - xc) * 2.0 * math.tan(math.radians(2.5 * k)) * aspect).astype(int)
            rr = np.arange(h)[:, None] + dy[None, :]          # 가운데 열 행 r 의 줄이 열 j 에서 지나는 행
            ok = (rr >= 0) & (rr < h)
            on_px = np.zeros(rr.shape, bool)
            on_px[ok] = sub[rr[ok], np.broadcast_to(cols, rr.shape)[ok]]
            cov = on_px.mean(axis=1)
            on = cov >= self.stop_line_min_cov
            r = h - 1
            while r >= 0:                  # 아래(가까운 쪽)부터 위로 첫 묶음
                if not on[r]:
                    r -= 1
                    continue
                bottom = r
                while r >= 0 and on[r]:
                    r -= 1
                if bottom - r >= self.stop_line_min_rows:
                    c = float(cov[r + 1:bottom + 1].mean())
                    if best is None or bottom > best[0] + 2 or (abs(bottom - best[0]) <= 2 and c > best[1]):
                        best = (bottom, c)
                    break
        if best is None or best[0] >= h - 2:   # 화면 아래에 걸침 → 가까운 끝을 모름
            return -1, 0.0
        return best

    # ---------------------------------------------------------------- 마스크
    def _build_mask(self, roi):
        if self.mask_mode == 'gray':
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            return cv2.threshold(gray, self.bright_thresh, 255, cv2.THRESH_BINARY)[1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        if self.use_white:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, self.white_hsv_lo, self.white_hsv_hi))
        if self.use_yellow:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, self.yellow_hsv_lo, self.yellow_hsv_hi))
        return mask

    # ---------------------------------------------------------------- 기본 밴드
    def _band_center(self, band, w):
        """가로 band 하나의 차선 중심. 전체 폭을 탐색하고 고정된 이미지 중앙에서 좌/우를 나눈다."""
        mid = w // 2
        lx = self._mean_x(band[:, 0:mid], 0)
        rx = self._mean_x(band[:, mid:w], mid)
        return self._combine_lr(lx, rx, w)

    def _combine_lr(self, lx, rx, w):
        """좌/우 라인 x → 차선 중심. 둘 다 보이면 차선 폭을 기억하고, 하나만 보이면 그 라인에서 반 폭 떨어진 곳."""
        if lx is not None and rx is not None:
            width = rx - lx
            # 폭 학습 하한 가드: 단선 하나가 탐색창 중앙에 걸쳐 갈라지면 폭이 선 두께로 붕괴 → 학습 스킵
            # LIMO 추가: 방지턱 흔들림 등으로 엉뚱한 선 쌍(폭 ~100 px)을 배우면 SW 페어 창이 그 폭으로 옮겨가 계속 물고 감
            tol = self.lane_width_tol
            floor = (1.0 - tol) * self._lane_width_init if tol > 0 else 0.5 * self._lane_width_init
            ceil = (1.0 + tol) * self._lane_width_init if tol > 0 else float('inf')
            if width > 0 and floor <= width <= ceil:
                self._lane_width = (width if self._lane_width <= 0
                                    else (1 - self.width_ema) * self._lane_width + self.width_ema * width)
            return (lx + rx) / 2.0, 2
        half = (self._lane_width / 2.0) if self._lane_width > 0 else (w * self.single_line_offset)
        line = lx if lx is not None else rx
        if line is None:
            return None, 0
        # 시간 연속성 분류: 직전 중심에 더 가까운 해석 — 화면 반쪽 기준은 커브에서 좌/우가 매 프레임 뒤집힌다.
        if self._prev_center is not None:
            cand_left, cand_right = line + half, line - half
            if abs(cand_left - self._prev_center) <= abs(cand_right - self._prev_center):
                return cand_left, 1
            return cand_right, 1
        if lx is not None:
            return lx + half, 1
        return rx - half, 1

    def _guided_bands(self, clean, w, roi_h, band_h):
        """각 band 가 이전 band 중심 주변(± margin)만 탐색하는 multi-band 중심. 멀리 있는 다른 선이 중심을 끌지 못한다.
        (bands, near_lanes, band_windows) — band_windows = [(x0, y0, x1, y1, cx_or_None), …] 디버그용."""
        bands, band_windows = [], []
        near_lanes = 0
        guide = None
        for i in range(self.num_bands):
            y1 = roi_h - i * band_h
            y0 = max(0, roi_h - (i + 1) * band_h)
            if y1 - y0 < 2:
                continue
            band = clean[y0:y1, :]
            if guide is None:
                cx, nlanes = self._band_center(band, w)
                x0, x1 = 0, w
                if cx is None and i == 0 and self.guide_use_previous_frame and self._prev_center is not None:
                    x0, x1, cx, nlanes = self._windowed_center(band, w, self._prev_center, i)
            else:
                x0, x1, cx, nlanes = self._windowed_center(band, w, guide, i)
            if cx is not None:
                if guide is not None:
                    jump = cx - guide
                    if abs(jump) > self.guide_max_jump_px:   # 갑작스러운 중심 점프를 클램프
                        cx = guide + (self.guide_max_jump_px if jump > 0 else -self.guide_max_jump_px)
                bands.append((cx, float(self.num_bands - i), i))
                if i == 0:
                    near_lanes = nlanes
                guide = cx
            band_windows.append((x0, y0, x1, y1, cx))
        return bands, near_lanes, band_windows

    def _windowed_center(self, band, w, guide, band_index):
        """[guide-margin, guide+margin] 으로 제한된 _band_center. 좌/우 분할 지점이 guide."""
        margin = self.guide_margin_px + band_index * self.guide_margin_growth_px
        x0 = int(max(0, min(w - 2, guide - margin)))
        x1 = int(max(x0 + 2, min(w, guide + margin)))
        mid = int(max(x0 + 1, min(x1 - 1, round(guide))))
        lx = self._mean_x(band[:, x0:mid], x0, self.guide_min_pixels)
        rx = self._mean_x(band[:, mid:x1], mid, self.guide_min_pixels)
        cx, nlanes = self._combine_lr(lx, rx, w)
        return x0, x1, cx, nlanes

    def _mean_x(self, half_mask, x_offset, min_pixels=None):
        xs = np.nonzero(half_mask)[1]
        if xs.size < (self.min_pixels if min_pixels is None else min_pixels):
            return None
        return float(np.mean(xs)) + x_offset

    # ---------------------------------------------------------------- 슬라이딩 윈도우 코리도
    def _sw_corridor_bands(self, src, w, roi_h, band_h):
        """상자 체인 + 2차 피팅으로 코리도 경계를 추적해 밴드 리스트를 만든다.
        (bands, near_lanes, debug_boxes) 또는 None (락 실패 → 호출측 폴백)."""
        m = src.copy()
        # (1) 정지선(가로줄) 행 제거
        row_on = np.count_nonzero(m, axis=1)
        m[row_on > int(w * float(self.sw_cross_row_frac)), :] = 0
        nb = max(3, int(self.sw_num_boxes))
        bh = max(2, roi_h // nb)
        margin = max(4, int(self.sw_box_margin))
        half = (self._lane_width / 2.0) if self._lane_width > 0 else (w * self.single_line_offset)

        # (2) 시드: 직전 피팅 우선, 다음 하단 절반 히스토그램 피크 (질량 순)
        seeds = []
        if self._sw_prev_fit is not None:
            pa, pb, pc = self._sw_prev_fit
            yb = float(roi_h - 1)
            seeds.append(pa * yb * yb + pb * yb + pc)
        hist = np.count_nonzero(m[roi_h // 2:, :], axis=0)
        col_on = np.nonzero(hist >= 2)[0]
        if col_on.size:
            splits = np.nonzero(np.diff(col_on) > 8)[0]
            peaks = []
            for g in np.split(col_on, splits + 1):
                mass = int(hist[g].sum())
                if mass >= int(self.sw_peak_min_px):
                    peaks.append((mass, float((g * hist[g]).sum()) / mass))
            peaks.sort(reverse=True)
            seeds += [cx for _, cx in peaks[:max(1, int(self.sw_max_peaks))]]
        if not seeds:
            self._sw_fail = 'noseed'
            return None

        # (3) 상자 체인
        def chain(seed_x):
            x = float(seed_x)
            trend = 0.0
            pxs, pys, hit = [], [], 0
            boxes = []
            for i in range(nb):
                y1 = roi_h - i * bh
                y0 = max(0, y1 - bh)
                if y1 - y0 < 2:
                    break
                x0 = int(max(0, x - margin))
                x1 = int(min(w, x + margin))
                if x1 - x0 < 4:
                    break
                ys_w, xs_w = np.nonzero(m[y0:y1, x0:x1])
                cx = None
                if xs_w.size >= max(1, int(self.sw_min_box_px)):   # 0 이면 빈 상자 mean() 이 NaN
                    raw_cx = float(xs_w.mean()) + x0
                    trend = float(np.clip(raw_cx - x, -float(self.sw_max_shift), float(self.sw_max_shift)))
                    x += trend
                    cx = x
                    pxs.append(xs_w.astype(np.float64) + x0)
                    pys.append(ys_w.astype(np.float64) + y0)
                    hit += 1
                else:
                    x += trend   # 점선 틈/짧은 소실: 직전 추세로 계속 위로
                boxes.append((x0, y0, x1, y1, cx))
                if x < -margin or x > w + margin:
                    break
            return pxs, pys, hit, boxes

        # (4) 피팅 + 게이트
        def fit_chain(pxs, pys, hit):
            if hit < max(1, int(self.sw_min_boxes)) or not pxs:
                self._sw_fitrej = 'boxes'
                return None
            xs = np.concatenate(pxs)
            ys = np.concatenate(pys)
            if xs.size < int(self.sw_min_pixels):
                self._sw_fitrej = 'px'
                return None
            y_lo, y_hi = float(ys.min()), float(ys.max())
            if (y_hi - y_lo) < 2.0 * bh:
                self._sw_fitrej = 'span'
                return None
            try:
                a, b, c = np.polyfit(ys, xs, 2)
            except Exception:
                self._sw_fitrej = 'fiterr'
                return None
            resid = float(np.mean(np.abs(a * ys * ys + b * ys + c - xs)))
            if resid > float(self.sw_max_resid_px):
                self._sw_fitrej = 'resid'
                return None
            x_bot = a * y_hi * y_hi + b * y_hi + c
            return {'abc': (float(a), float(b), float(c)), 'y_lo': y_lo, 'y_hi': y_hi,
                    'resid': resid, 'x_bot': float(x_bot)}

        fits = []
        for s in seeds:
            pxs, pys, hit, boxes = chain(s)
            f = fit_chain(pxs, pys, hit)
            if f is None:
                continue
            # 중복 제거: 하단 교점이 기존 채택과 사실상 같은 선이면 잔차 좋은 쪽만
            dup = False
            for prev in fits:
                if abs(prev['x_bot'] - f['x_bot']) < 0.3 * half:
                    dup = True
                    if f['resid'] < prev['resid']:
                        prev.update(f)
                        prev['boxes'] = boxes
                    break
            if not dup:
                f['boxes'] = boxes
                fits.append(f)
        if not fits:
            self._sw_fail = 'nofit'
            return None

        def ev(f, y):
            a, b, c = f['abc']
            yc = min(max(y, f['y_lo']), f['y_hi'])   # (6) 외삽 금지: 스팬 클램프
            return a * yc * yc + b * yc + c

        # (5) 코리도 해석: 차폭 간격 두 경계(pair) 우선. 유효 페어 (간격 0.7–1.4 × 차폭) 중 간격이 차폭에 가장 가깝고,
        # 직전 중심에 가까운 것. (원본은 '왼쪽부터 첫 유효 페어' — 옆에 나란한 다른 도로 선 (안쪽 위 도로 북쪽 24 cm 의
        # 출발 도로 선) 이 보이면 두 도로에 걸친 74 cm (차폭 1.4 배 안) 쌍을 먼저 골라 차로 가운데에서 11 cm 벗어나 달렸다)
        fits.sort(key=lambda f: f['x_bot'])
        pair, best_cost = None, None
        lw = 2.0 * half
        for i in range(len(fits) - 1):
            for j in range(i + 1, len(fits)):
                sep = fits[j]['x_bot'] - fits[i]['x_bot']
                if 0.7 * lw <= sep <= 1.4 * lw:
                    cost = abs(sep - lw) / lw
                    ref = self._prev_pair_mid if self._prev_pair_mid is not None else self._prev_center
                    if ref is not None:     # 직전 쌍의 아래 가운데 (같은 단위) — 없으면 직전 최종 중심
                        cost += 0.5 * abs((fits[i]['x_bot'] + fits[j]['x_bot']) / 2.0 - ref) / lw
                    if best_cost is None or cost < best_cost:
                        pair, best_cost = (fits[i], fits[j]), cost

        bands = []
        self._prev_pair_mid = (pair[0]['x_bot'] + pair[1]['x_bot']) / 2.0 if pair else None
        if pair:
            fl, fr = pair
            chosen = [fl, fr]
            y_min = min(fl['y_lo'], fr['y_lo'])
            y_max = max(fl['y_hi'], fr['y_hi'])
            for i in range(self.num_bands):
                yc = roi_h - (i + 0.5) * band_h
                if yc < y_min - band_h or yc > y_max + band_h:
                    continue
                bands.append(((ev(fl, yc) + ev(fr, yc)) / 2.0, float(self.num_bands - i), i))
            near_lanes = 2
            best = fl if fl['resid'] <= fr['resid'] else fr
            self._pair_w = (ev(fr, roi_h - 1) - ev(fl, roi_h - 1), ev(fr, 0) - ev(fl, 0))
            self._line_resid = max(fl['resid'], fr['resid'])
            self._line_side = 2
            yb = roi_h - 1
            self._line_slope = ((2 * fl['abc'][0] * yb + fl['abc'][1]) + (2 * fr['abc'][0] * yb + fr['abc'][1])) / 2.0
        else:
            best = min(fits, key=lambda f: f['resid'])
            # 단선: 직전 중심에 가까운 해석 (좌측 경계면 중심 = 선 + 반차폭), 없으면 기본값
            if self._prev_center is not None:
                cand_l, cand_r = best['x_bot'] + half, best['x_bot'] - half
                side = 1.0 if abs(cand_l - self._prev_center) <= abs(cand_r - self._prev_center) else -1.0
            else:
                side = float(self.sw_side_default)   # -1 = 추적선이 우측 경계
            chosen = [best]
            self._line_resid = best['resid']
            self._line_side = -1 if best['x_bot'] < w / 2.0 else 1
            self._line_slope = 2 * best['abc'][0] * (roi_h - 1) + best['abc'][1]
            off = self._normal_offset(best, side, half, roi_h) if self.sw_single_normal else None
            for i in range(self.num_bands):
                yc = roi_h - (i + 0.5) * band_h
                if yc < best['y_lo'] - band_h or yc > best['y_hi'] + band_h:
                    continue
                cx = float(np.interp(yc, off[0], off[1])) if off is not None else ev(best, yc) + side * half
                bands.append((cx, float(self.num_bands - i), i))
            near_lanes = 1
        if not bands:
            self._sw_fail = 'noband'
            return None

        self._sw_prev_fit = best['abc']   # 락 성공시에만 — 실패 프레임은 시드 연속성 유지
        boxes_dbg = []
        for f in chosen:
            boxes_dbg += f.get('boxes', [])
            self._sw_fit_dbg.append([(int(round(ev(f, y))), int(y))
                                     for y in range(int(f['y_lo']), int(f['y_hi']) + 1, 4)])
        return bands, near_lanes, boxes_dbg

    def _normal_offset(self, f, side, half, roi_h):
        """단선 f 에서 바닥 기준 법선 방향으로 반 차폭 (half px = 가로 축척) 떨어진 중심 곡선 → (행 오름차순, x).
        원본처럼 BEV 가로로 half 를 더하면 선이 차 축과 θ 기울 때 실제 거리는 half·cosθ (45° 면 0.26 → 0.18 m) 라
        비스듬한 선 하나만 보이는 곳 (3 차선 진입 코너 등) 에서 차가 그 선에 붙고, 곡률도 선의 벌어짐을 따라 반대 부호가 났다.
        BEV 축척은 가로·세로가 다르다: kx = lane_width_init / 차폭 m, ky = BEV 높이 / 깊이 m. 깊이를 모르면 None (원본 방식)."""
        if self.stop_line_bev_depth_m <= 0 or self.stop_line_lane_m <= 0:
            return None
        kx = self._lane_width_init / self.stop_line_lane_m
        ky = roi_h / self.stop_line_bev_depth_m
        half_m = half / kx
        a, b, c = f['abc']
        y_lo, y_hi = f['y_lo'], f['y_hi']
        ext = half_m * ky + 2.0                  # 법선으로 옮기면 행도 이만큼까지 밀림 → 선을 양 끝 접선으로 곧게 연장 (2 차 외삽은 안 함)
        ys = np.arange(y_lo - ext, y_hi + ext + 1.0, 2.0)
        yc_ = np.clip(ys, y_lo, y_hi)
        sl = 2 * a * yc_ + b                     # dx/dy [px/px] (끝 밖은 끝 기울기)
        xs = a * yc_ * yc_ + b * yc_ + c + sl * (ys - yc_)
        tx, ty = sl / kx, 1.0 / ky               # 바닥 접선 (m / 행)
        nn = np.hypot(tx, ty)
        nx, ny = ty / nn, -tx / nn               # 법선, nx > 0 = 화면 오른쪽
        cx = xs + side * half_m * nx * kx
        cy = ys + side * half_m * ny * ky
        o = np.argsort(cy)
        return cy[o], cx[o]

    # ---------------------------------------------------------------- BEV, look-ahead, 곡률
    def _apply_birdeye(self, roi):
        """ROI 를 탑다운 뷰로 워프 (같은 크기). 실패하면 입력 ROI 를 그대로 — 잘못된 점 때문에 추종이 죽지 않게."""
        h, w = roi.shape[:2]
        key = (w, h, tuple(self.birdeye_src_ratio), tuple(self.birdeye_dst_ratio))
        if key != self._bev_key:
            self._bev_key = key
            self._bev_matrix = None
            try:
                if len(self.birdeye_src_ratio) != 8 or len(self.birdeye_dst_ratio) != 8:
                    raise ValueError('birdeye_*_ratio must be 8 floats [x1,y1,...,x4,y4]')
                src = np.float32([(self.birdeye_src_ratio[i] * w, self.birdeye_src_ratio[i + 1] * h) for i in range(0, 8, 2)])
                dst = np.float32([(self.birdeye_dst_ratio[i] * w, self.birdeye_dst_ratio[i + 1] * h) for i in range(0, 8, 2)])
                self._bev_matrix = cv2.getPerspectiveTransform(src, dst)
            except Exception as exc:
                if not self._bev_warned:
                    print(f'[sliding_window] bird-eye disabled, using raw ROI: {exc}')
                    self._bev_warned = True
        if self._bev_matrix is None:
            return roi
        try:
            return cv2.warpPerspective(roi, self._bev_matrix, (w, h))
        except cv2.error as exc:
            if not self._bev_warned:
                print(f'[sliding_window] bird-eye warp failed, using raw ROI: {exc}')
                self._bev_warned = True
            return roi

    def _lookahead_center(self, bands):
        """look-ahead band 중심 x: lookahead_band_index band 가 있으면 그것, 아니면 가장 먼 band (-1 = 항상 가장 먼 것)."""
        if not bands:
            return None
        if self.lookahead_band_index >= 0:
            for cx, _, i in bands:
                if i == self.lookahead_band_index:
                    return cx
        return bands[-1][0]

    @staticmethod
    def _estimate_curvature(bands, w):
        """감지된 가장 먼 band 와 가장 가까운 band 사이의 정규화된 횡방향 드리프트."""
        if len(bands) < 2:
            return 0.0
        near = min(bands, key=lambda b: b[2])
        far = max(bands, key=lambda b: b[2])
        return float(max(-1.0, min(1.0, (far[0] - near[0]) / (w / 2.0))))

    # ---------------------------------------------------------------- 디버그
    def _draw_debug(self, roi, mask, lane_center, offset, curvature, band_windows, near_cx, la_cx, sw_locked, stop_row=-1):
        """왼쪽 = BEV 컬러 + 상자(보라)·피팅(주황)·중심(빨강)·화면 중심(초록), 오른쪽 = 마스크."""
        dbg = roi.copy()
        for x0, y0, x1, y1, bcx in band_windows:
            cv2.rectangle(dbg, (int(x0), int(y0)), (int(x1) - 1, int(y1) - 1), (255, 0, 255), 1)
            if bcx is not None:
                cv2.circle(dbg, (int(bcx), int((y0 + y1) / 2)), 2, (0, 165, 255), -1)
        for poly in self._sw_fit_dbg:
            if len(poly) >= 2:
                cv2.polylines(dbg, [np.int32(poly)], False, (0, 165, 255), 2)
        h, w = dbg.shape[:2]
        cv2.line(dbg, (int(lane_center), 0), (int(lane_center), h), (0, 0, 255), 2)
        cv2.line(dbg, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)
        if near_cx is not None:
            cv2.line(dbg, (int(near_cx), h * 2 // 3), (int(near_cx), h), (255, 0, 0), 1)
        if la_cx is not None:
            cv2.line(dbg, (int(la_cx), 0), (int(la_cx), h // 3), (0, 165, 255), 1)
        if stop_row >= 0:                  # 정지선 가까운 쪽 끝 (하늘색)
            cv2.line(dbg, (0, stop_row), (w, stop_row), (255, 255, 0), 2)
        txt = f"off {offset:+.2f} curv {curvature:+.2f} {'SW' if sw_locked else 'band'} W{self._lane_width:.0f}"
        if self._line_resid > 0:
            txt += f" r{self._line_resid:.1f}"
        cv2.rectangle(dbg, (0, 0), (min(w, 8 + 7 * len(txt)), 16), (0, 0, 0), -1)
        cv2.putText(dbg, txt, (3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        return np.hstack([dbg, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
