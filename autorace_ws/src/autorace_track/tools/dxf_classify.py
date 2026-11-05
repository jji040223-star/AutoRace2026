#!/usr/bin/env python3
"""2026Track.dxf → 노면 요소 분류 → track_elements.json + 확인용 PNG.

도면은 모든 선이 'Visible' 레이어 하나에 '윤곽선'으로만 그려져 있다(채움 없음).
그래서 선 자체를 분류하지 않고, 선들을 노딩해 만든 닫힌 영역(face) 하나하나를
크기·모양·위상(구멍, 이웃)으로 분류한다.

좌표계: 트랙 매트(10000×6000) 좌하단 = (0, 0), x = 도면 오른쪽, y = 도면 위쪽, 단위 mm(실치수).
도면은 1:30으로 그려져 있어 DIMENSION의 dimlfac(=30)을 곱해 실치수로 바꾼다.
"""
import argparse
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import ezdxf
import matplotlib
import numpy as np
import shapely
from shapely.geometry import LineString, MultiLineString, Polygon
from shapely.ops import polygonize, unary_union

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch, PathPatch  # noqa: E402
from matplotlib.path import Path as MplPath  # noqa: E402

LAYER = "Visible"
SNAP_MM = 0.05     # 끝점이 1e-5 mm 수준으로 어긋나 있어 격자 스냅 후 노딩
SIZE_TOL_MM = 3.0

# 직사각형 요소: (짧은 변, 긴 변) mm → 분류. 치수는 PDF 치수선 기준.
RECT_RULES = [
    ((20, 200), "lane_dash"),     # A/B 구간 점선
    ((40, 500), "stop_line"),     # 차로 폭(500)을 가로지르는 40 mm 선
    ((30, 500), "crosswalk"),     # 횡단보도 앞뒤 가로선
    ((70, 200), "crosswalk"),     # 횡단보도 막대
    ((500, 600), "parking_bay"),  # 주차 구획 안쪽(비도색)
    ((10, 500), "start_black"),   # 출발선 체커 양옆 검정 띠
]
CHECKER_SIZES = (30, 40)          # 출발선 체커 칸
GLYPH_MAX_MM = 400                # 글자(천천히, A, B) 한 획 덩어리의 최대 크기
STRIP_MAX_THICK_MM = 30           # 경계선(20 mm) 판정용 두께 상한
STRIP_MIN_LEN_MM = 1000
TEXT_NEAR_DIAMOND_MM = 700        # 마름모에서 이 거리 안의 글자 = '천천히'
ZIGZAG_MIN_SEGS = 6
ZIGZAG_MIN_TURN_DEG = 60
ZIGZAG_BUFFER_MM = 25

PAINTED = {"road_edge", "zigzag", "lane_dash", "stop_line", "crosswalk", "slow_diamond",
           "slow_text", "branch_letter", "parking_line", "start_white", "start_black"}
SURFACE = {"road", "parking_bay", "off_road", "island"}


def dimension_scale(doc):
    for d in doc.modelspace().query("DIMENSION"):
        lf = d.override().get("dimlfac", None)
        if lf:
            return float(lf)
    raise SystemExit("DIMENSION의 dimlfac를 찾지 못함 → --scale로 지정")


def load_lines(doc, scale):
    raw = []
    for e in doc.modelspace():
        if e.dxf.layer != LAYER:
            continue
        t = e.dxftype()
        if t == "LINE":
            pts = [e.dxf.start, e.dxf.end]
        elif t in ("ARC", "SPLINE", "CIRCLE", "ELLIPSE", "LWPOLYLINE", "POLYLINE"):
            pts = list(e.flattening(0.002))   # 도면 단위 0.002 = 실치수 0.06 mm
        else:
            continue
        raw.append((t, [(p[0], p[1]) for p in pts]))
    allp = np.array([p for _, pts in raw for p in pts])
    ox, oy = allp.min(0)
    lines = [(t, LineString([((x - ox) * scale, (y - oy) * scale) for x, y in pts])) for t, pts in raw]
    size = ((allp[:, 0].max() - ox) * scale, (allp[:, 1].max() - oy) * scale)
    return lines, size


def build_faces(lines):
    ml = shapely.set_precision(MultiLineString([g for _, g in lines]), SNAP_MM)
    return list(polygonize(unary_union(ml)))


def rect_dims(f):
    c = list(f.minimum_rotated_rectangle.exterior.coords)
    a, b = math.dist(c[0], c[1]), math.dist(c[1], c[2])
    return min(a, b), max(a, b), f.area / max(a * b, 1e-9)


def thickness(f):
    return 2 * f.area / (f.exterior.length + sum(h.length for h in f.interiors))


def is_rhombus(ring):
    """꼭짓점 4개, 네 변 길이가 같고 직각이 아닌 닫힌 고리."""
    pts = list(Polygon(ring).simplify(1.0).exterior.coords)[:-1]
    if len(pts) != 4:
        return False
    sides = [math.dist(pts[i], pts[(i + 1) % 4]) for i in range(4)]
    if max(sides) - min(sides) > 0.05 * max(sides):
        return False
    v1 = np.subtract(pts[1], pts[0]); v2 = np.subtract(pts[3], pts[0])
    cosang = abs(np.dot(v1, v2)) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    return cosang > 0.2


def fills_hole_of(f, other):
    return any(abs(Polygon(h).area - f.area) < 1.0 and Polygon(h).buffer(0.5).contains(f)
               for h in other.interiors)


def zigzag_runs(lines):
    """LINE 끝점 그래프에서, 방향이 번갈아 크게 꺾이는 연속 선분 묶음을 찾는다."""
    key = lambda p: (round(p[0], 1), round(p[1], 1))
    adj = defaultdict(list)
    segs = [g for t, g in lines if t == "LINE"]
    for i, g in enumerate(segs):
        a, b = key(g.coords[0]), key(g.coords[-1])
        adj[a].append((i, b)); adj[b].append((i, a))
    seen, runs = set(), []
    for start in list(adj):
        if len(adj[start]) == 2:
            continue                          # 체인의 끝(차수 ≠ 2)에서만 출발
        for i0, nxt in adj[start]:
            if i0 in seen:
                continue
            chain, cur, prev_i = [start, nxt], nxt, i0
            seen.add(i0)
            while len(adj[cur]) == 2:
                (i1, n1), (i2, n2) = adj[cur]
                i, n = (i2, n2) if i1 == prev_i else (i1, n1)
                if i in seen:
                    break
                seen.add(i); chain.append(n); prev_i, cur = i, n
            runs.extend(_zig_subruns(chain))
    return runs


def _zig_subruns(chain):
    """연속된 꼭짓점이 매번 반대 방향으로 ≥60° 꺾이는 구간 = 지그재그."""
    pts = np.asarray(chain, float)
    u, v = pts[1:-1] - pts[:-2], pts[2:] - pts[1:-1]
    turn = np.degrees(np.arctan2(u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0], (u * v).sum(1)))
    sharp = np.abs(turn) >= ZIGZAG_MIN_TURN_DEG
    out, k = [], 0
    while k < len(turn):
        if not sharp[k]:
            k += 1
            continue
        e = k
        while e + 1 < len(turn) and sharp[e + 1] and np.sign(turn[e + 1]) == -np.sign(turn[e]):
            e += 1
        if e - k + 2 >= ZIGZAG_MIN_SEGS:      # 꼭짓점 k..e → 선분 e-k+2개
            out.append(LineString(pts[k:e + 3]))
        k = e + 1
    return out


def classify(faces, lines, size):
    n = len(faces)
    cls = [None] * n
    W, H = size
    bg = max(range(n), key=lambda i: faces[i].area)
    assert abs(faces[bg].bounds[2] - W) < 1 and abs(faces[bg].bounds[3] - H) < 1
    cls[bg] = "off_road"
    dims = [rect_dims(f) for f in faces]

    # 1) 규격 직사각형
    for i, f in enumerate(faces):
        s, l, rectness = dims[i]
        if cls[i] or f.interiors or rectness < 0.97:
            continue
        for (rs, rl), c in RECT_RULES:
            if abs(s - rs) <= SIZE_TOL_MM and abs(l - rl) <= SIZE_TOL_MM:
                cls[i] = c
                break

    # 2) 출발선 체커: 30/40 mm 칸들을 행·열로 정렬해 번갈아 흰색
    cells = [i for i, f in enumerate(faces) if not cls[i] and dims[i][2] > 0.97
             and all(min(abs(d - c) for c in CHECKER_SIZES) <= SIZE_TOL_MM for d in dims[i][:2])]
    if cells:
        xs = sorted({round(faces[i].centroid.x / 5) * 5 for i in cells})
        ys = sorted({round(faces[i].centroid.y / 5) * 5 for i in cells}, reverse=True)
        for i in cells:
            col = xs.index(round(faces[i].centroid.x / 5) * 5)
            row = ys.index(round(faces[i].centroid.y / 5) * 5)
            # 참고 PNG: 맨 윗줄은 오른쪽 칸이 흰색
            cls[i] = "start_white" if (row + col) % 2 == 1 else "road"

    # 3) 천천히 마름모: 마름모 고리(구멍 1개). 구멍을 채우는 face는 도로면
    for i, f in enumerate(faces):
        if not cls[i] and len(f.interiors) == 1 and is_rhombus(f.exterior) and is_rhombus(f.interiors[0]):
            cls[i] = "slow_diamond"
    for i, f in enumerate(faces):
        if not cls[i] and any(cls[j] == "slow_diamond" and fills_hole_of(f, faces[j]) for j in range(n)):
            cls[i] = "road"

    # 4) 길고 가는 띠(두께 ≈ 20 mm): 주차 구획 묶음 안에 들어 있으면 주차선(구획 사이·윗선),
    #    아니면 도로 경계선. 주차장 좌/하/우 테두리는 경계선 띠와 한 덩어리라 경계선으로 남는다.
    bays = [faces[j] for j in range(n) if cls[j] == "parking_bay"]
    bay_zone = unary_union(bays).envelope.buffer(30, join_style="mitre") if bays else None
    for i, f in enumerate(faces):
        if cls[i] or thickness(f) > STRIP_MAX_THICK_MM or dims[i][1] < STRIP_MIN_LEN_MM:
            continue
        cls[i] = "parking_line" if bay_zone is not None and bay_zone.contains(f) else "road_edge"

    # 5) 글자: 작은 비정형 face. 다른 글자의 구멍을 채우는 face(ㅇ, A, B 속)는 도로면
    glyphs = [i for i, f in enumerate(faces) if not cls[i] and dims[i][1] <= GLYPH_MAX_MM]
    for i in glyphs:
        if any(j != i and fills_hole_of(faces[i], faces[j]) for j in glyphs):
            cls[i] = "road"
    diamonds = [faces[j].centroid for j in range(n) if cls[j] == "slow_diamond"]
    for i in glyphs:
        if cls[i]:
            continue
        near = any(faces[i].centroid.distance(d) < TEXT_NEAR_DIAMOND_MM for d in diamonds)
        cls[i] = "slow_text" if near else "branch_letter"

    # 6) 나머지 넓은 면: 도로 경계선으로만 둘러싸였으면 섬(분리대 안쪽), 아니면 도로
    for i, f in enumerate(faces):
        if cls[i]:
            continue
        nbrs = {cls[j] for j in range(n) if j != i and faces[j].intersects(f)
                and f.boundary.intersection(faces[j].boundary).length > 1}
        if thickness(Polygon(f.exterior)) < 100:   # 구멍(막대·글자) 둘레는 빼고 판단
            cls[i] = "unknown"
        elif nbrs == {"road_edge"}:
            cls[i] = "island"
        else:
            cls[i] = "road"

    # 7) 경계선 중 지그재그 구간을 떼어 낸다
    runs = zigzag_runs(lines)
    zone = unary_union([r.buffer(ZIGZAG_BUFFER_MM, cap_style="flat") for r in runs]) if runs else None
    out = []
    for i, f in enumerate(faces):
        if cls[i] == "road_edge" and zone is not None and f.intersects(zone):
            for c, g in (("zigzag", f.intersection(zone)), ("road_edge", f.difference(zone))):
                out += [(c, p) for p in getattr(g, "geoms", [g]) if p.area > 1]
        else:
            out.append((cls[i], f))
    return out, len(runs)


def start_pose(elements):
    """시작 자세 = 출발선(체크무늬) 가운데, 진행 방향 = 같은 차로 안에서 가장 가까운 정지선·마름모·횡단보도 쪽.

    반환: (x, y [mm], yaw [deg]) — yaw 0 = +x. 출발선이 없으면 None.
    """
    starts = [p for c, p in elements if c in ("start_white", "start_black")]
    if not starts:
        return None
    x0, y0, x1, y1 = unary_union(starts).bounds
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    along_x = (x1 - x0) < (y1 - y0)            # 선이 y 로 길면 차로는 x 방향
    half_lane = ((y1 - y0) if along_x else (x1 - x0)) / 2
    nearest = None
    for c, p in elements:
        if c not in ("stop_line", "slow_diamond", "crosswalk"):
            continue
        across, ahead = ((p.centroid.y - cy, p.centroid.x - cx) if along_x
                         else (p.centroid.x - cx, p.centroid.y - cy))
        if abs(across) < half_lane and (nearest is None or abs(ahead) < abs(nearest)):
            nearest = ahead
    forward = nearest is None or nearest > 0
    yaw = (0.0 if forward else 180.0) if along_x else (90.0 if forward else -90.0)
    return cx, cy, yaw


def to_json(elements, size, scale, src):
    def ring(r):
        return [[round(x, 2), round(y, 2)] for x, y in r.coords]
    items = []
    for k, (c, p) in enumerate(elements):
        items.append({
            "id": k, "class": c, "painted": c in PAINTED,
            "area_mm2": round(p.area, 1),
            "centroid": [round(p.centroid.x, 2), round(p.centroid.y, 2)],
            "bbox": [round(v, 2) for v in p.bounds],
            "exterior": ring(p.exterior), "holes": [ring(h) for h in p.interiors],
        })
    pose = start_pose(elements)
    return {"source": src, "units": "mm", "drawing_scale": scale,
            "frame": "origin = mat bottom-left, +x = drawing right, +y = drawing up",
            "size_mm": [round(size[0], 3), round(size[1], 3)],
            "start_pose": None if pose is None else {"x": round(pose[0], 2), "y": round(pose[1], 2), "yaw_deg": pose[2],
                                                     "rule": "start line center, heading to nearest stop/diamond/crosswalk in lane"},
            "elements": items}


# ---------------------------------------------------------------- 시각화
FONT = "NanumGothic"
SURFACE_STYLE = {"off_road": "#fcfcfb", "road": "#d9d8d3", "parking_bay": "#d9d8d3", "island": "#b9b8b2"}
# 분류(도색) 색: 기준 팔레트 슬롯 순서 그대로. 지그재그는 경계선과 같은 색 + 빗금.
PAINT_STYLE = {
    "crosswalk":     ("#2a78d6", "횡단보도", None),
    "road_edge":     ("#eb6834", "도로 경계선", None),
    "zigzag":        ("#eb6834", "지그재그 (경계선의 일부)", "////"),
    "parking_line":  ("#1baf7a", "주차칸", None),
    "lane_dash":     ("#eda100", "차선 점선", None),
    "slow_diamond":  ("#e87ba4", "'천천히' 마름모", None),
    "slow_text":     ("#008300", "노면글자 '천천히'", None),
    "branch_letter": ("#4a3aa7", "A/B 분기 글자", None),
    "stop_line":     ("#e34948", "정지선", None),
    "start_white":   ("#ffffff", "출발선 (흰 체커)", None),
    "start_black":   ("#111111", "출발선 (검정 띠)", None),
}


def _patch(poly, **kw):
    verts, codes = [], []
    for r in [poly.exterior, *poly.interiors]:
        c = np.asarray(r.coords)
        verts += c.tolist(); codes += [MplPath.MOVETO] + [MplPath.LINETO] * (len(c) - 2) + [MplPath.CLOSEPOLY]
    return PathPatch(MplPath(verts, codes), **kw)


def _draw(ax, elements):
    for c, p in elements:
        if c in SURFACE_STYLE:
            ax.add_patch(_patch(p, fc=SURFACE_STYLE[c], ec="none", hatch="xx" if c == "island" else None, lw=0))
        elif c == "unknown":
            ax.add_patch(_patch(p, fc="#000000", ec="#000000", lw=1))
    for c, p in elements:
        if c in PAINT_STYLE:
            col, _, hatch = PAINT_STYLE[c]
            edge = "#52514e" if c == "start_white" else ("#ffffff" if hatch else "none")
            ax.add_patch(_patch(p, fc=col, ec=edge, lw=0.4 if c == "start_white" else 0, hatch=hatch))
    ax.set_aspect("equal")


LIMO_FOOTPRINT_MM = (322, 246)   # WeGo limo_diff_thin 바퀴 바깥까지


def _draw_start(ax, pose, label=True):
    x, y, yaw = pose
    t = np.radians(yaw)
    L, Wd = LIMO_FOOTPRINT_MM
    corners = np.array([[-L / 2, -Wd / 2], [L / 2, -Wd / 2], [L / 2, Wd / 2], [-L / 2, Wd / 2]])
    rot = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    ax.add_patch(plt.Polygon(corners @ rot.T + [x, y], closed=True, fill=False, ec="#0b0b0b", lw=1.6, ls="--"))
    ax.annotate("", xy=(x + 300 * np.cos(t), y + 300 * np.sin(t)), xytext=(x, y),
                arrowprops=dict(arrowstyle="-|>", color="#0b0b0b", lw=2.2, mutation_scale=18))
    if label:
        ax.text(x, y - Wd / 2 - 60, f"LIMO 시작 ({x / 1000:.3f}, {y / 1000:.3f}) m, yaw {yaw:.0f}°",
                fontsize=11, ha="left", va="top", color="#0b0b0b",
                bbox=dict(fc="white", ec="#52514e", lw=0.5, alpha=0.9))


def render_png(elements, size, counts, path, n_zig):
    plt.rcParams["font.family"] = FONT
    plt.rcParams["hatch.color"] = "#ffffff"
    W, H = size
    fig = plt.figure(figsize=(26, 26), dpi=110)
    gs = fig.add_gridspec(3, 4, height_ratios=[6, 2.9, 2.9], hspace=0.18, wspace=0.12)
    ax = fig.add_subplot(gs[0, :])
    _draw(ax, elements)
    ax.add_patch(plt.Rectangle((0, 0), W, H, fill=False, ec="#52514e", lw=1))
    ax.set_xlim(-100, W + 100); ax.set_ylim(-100, H + 100)
    ax.set_xticks(range(0, int(W) + 1, 1000)); ax.set_yticks(range(0, int(H) + 1, 1000))
    ax.tick_params(labelsize=11, colors="#52514e")
    ax.grid(color="#52514e", alpha=0.12, lw=0.6)
    ax.set_xlabel("x [mm]", fontsize=12, color="#52514e"); ax.set_ylabel("y [mm]", fontsize=12, color="#52514e")
    ax.set_title(f"2026Track.dxf 분류 결과  —  {W:.0f}×{H:.0f} mm (도면 1:30 → ×30), 원점 = 매트 좌하단",
                 fontsize=17, loc="left", color="#0b0b0b")
    pose = start_pose(elements)
    if pose:
        _draw_start(ax, pose)

    handles = []
    for c, (col, label, hatch) in PAINT_STYLE.items():
        if counts.get(c):
            handles.append(Patch(fc=col, ec="#52514e", hatch=hatch, label=f"{label}  ×{counts[c]}"))
    for c, label in (("road", "도로면 (비도색)"), ("island", "분리대 안쪽 (비도로)"), ("off_road", "트랙 밖 / 안쪽 빈 공간")):
        handles.append(Patch(fc=SURFACE_STYLE[c], ec="#52514e", hatch="xx" if c == "island" else None, label=label))
    if counts.get("unknown"):
        handles.append(Patch(fc="#000000", label=f"미분류 ×{counts['unknown']}"))
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.005, 1.0), fontsize=12,
              frameon=False, title=f"요소 (face 개수)\n지그재그 체인 {n_zig}개 검출", title_fontsize=12)

    zooms = [
        ("출발선 (체커) + LIMO 시작 자세", (700, 1320, 5280, 5900)),
        ("'천천히' + 횡단보도 (2차로)", (2200, 3850, 4540, 5140)),
        ("지그재그 + 마름모 + 횡단보도", (9180, 9980, 1880, 4560)),
        ("주차칸", (5950, 7800, 3250, 4700)),
        ("A/B 분기 + 점선", (2350, 5400, 100, 1780)),
        ("'천천히' (1차로) + 정지선", (2420, 3450, 5280, 5900)),
        ("왼쪽 직선 정지선", (300, 1150, 1650, 4100)),
        ("오른쪽 아래 정지선", (8200, 9000, 550, 1350)),
    ]
    for k, (title, (xa, xb, ya, yb)) in enumerate(zooms):
        z = fig.add_subplot(gs[1 + k // 4, k % 4])
        _draw(z, elements)
        if k == 0 and pose:
            _draw_start(z, pose, label=False)
        z.set_xlim(xa, xb); z.set_ylim(ya, yb)
        z.set_title(title, fontsize=13, loc="left", color="#0b0b0b")
        z.tick_params(labelsize=10, colors="#52514e")
        for s in z.spines.values():
            s.set_color("#52514e"); s.set_linewidth(0.6)
        ax.add_patch(plt.Rectangle((xa, ya), xb - xa, yb - ya, fill=False, ec="#0b0b0b", lw=0.8, ls="--"))
        ax.text(xa, yb + 25, str(k + 1), fontsize=11, color="#0b0b0b")
        z.text(0.01, 0.99, str(k + 1), transform=z.transAxes, va="top", fontsize=12,
               bbox=dict(fc="white", ec="#52514e", lw=0.5))
    fig.savefig(path, bbox_inches="tight", facecolor="white")


def main():
    here = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dxf", default=here / "data/source/2026Track.dxf", type=Path)
    ap.add_argument("--out-json", default=here / "data/generated/track_elements.json", type=Path)
    ap.add_argument("--out-png", default=here / "data/generated/track_classification.png", type=Path)
    ap.add_argument("--scale", type=float, default=None, help="도면→실치수 배율 (기본: DIMENSION dimlfac)")
    a = ap.parse_args()

    doc = ezdxf.readfile(a.dxf)
    scale = a.scale or dimension_scale(doc)
    lines, size = load_lines(doc, scale)
    faces = build_faces(lines)
    elements, n_zig = classify(faces, lines, size)
    counts = Counter(c for c, _ in elements)

    print(f"scale ×{scale:g}, size {size[0]:.1f} × {size[1]:.1f} mm, entities {len(lines)}, faces {len(faces)}")
    for c, k in sorted(counts.items(), key=lambda t: (t[0] not in PAINTED, t[0])):
        print(f"  {'paint' if c in PAINTED else 'surf ':5s} {c:14s} {k:4d}")
    a.out_json.parent.mkdir(parents=True, exist_ok=True)
    a.out_json.write_text(json.dumps(to_json(elements, size, scale, a.dxf.name), ensure_ascii=False))
    render_png(elements, size, counts, a.out_png, n_zig)
    pose = start_pose(elements)
    if pose:
        print(f"start pose: ({pose[0]:.1f}, {pose[1]:.1f}) mm, yaw {pose[2]:.0f} deg")
    print("wrote", a.out_json, "and", a.out_png)
    if counts.get("unknown"):
        raise SystemExit(f"미분류 face {counts['unknown']}개 — 규칙 확인 필요")


if __name__ == "__main__":
    main()
