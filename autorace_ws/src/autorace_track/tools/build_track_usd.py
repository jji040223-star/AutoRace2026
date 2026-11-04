#!/usr/bin/env python3
"""track_elements.json(mm) → Isaac Sim용 트랙 USD (단위 m, Z-up).

  /AutoRaceTrack                    defaultPrim
    Looks/                          UsdPreviewSurface 재질 (매트, 행사장 바닥, 페인트 색별)
    Physics/FloorMaterial           물리 재질: 마찰·반발 (track_build.yaml physics)
    Physics/FloorCollider           보이지 않는 충돌 평면 (z = 0, 무한)
    Ground/Mat, Ground/Venue        보이는 바닥 메쉬 (충돌 없음)
    Markings/<class>/<class>_NN     노면표시 메쉬 (충돌 없음), 시맨틱 라벨 class=<class>

좌표: DXF 분류 결과와 같다 — 원점 = 매트 좌하단, +x = 도면 오른쪽, +y = 도면 위쪽.
pxr는 Isaac Sim 5.1 과 같은 USD 24.05 (pip usd-core==24.5) 를 쓴다.
"""
import argparse
import json
from pathlib import Path

import shapely
import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
from shapely.geometry import Polygon

MM = 1e-3
PKG = Path(__file__).resolve().parent.parent


def srgb_to_linear(hex_color):
    rgb = [int(hex_color.lstrip("#")[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return Gf.Vec3f(*[c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in rgb])


def make_look(stage, path, hex_color, roughness):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(srgb_to_linear(hex_color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def triangulate(polys, z):
    """다각형(구멍 포함) 목록 → (points, faceVertexIndices). 모든 삼각형을 +z 방향(CCW)으로."""
    points, indices, vmap = [], [], {}
    for p in polys:
        for t in shapely.constrained_delaunay_triangles(p).geoms:
            a, b, c = list(t.exterior.coords)[:3]
            cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            if abs(cross) < 1e-9:
                continue
            for x, y in ((a, b, c) if cross > 0 else (a, c, b)):
                key = (round(x, 4), round(y, 4))
                if key not in vmap:
                    vmap[key] = len(points)
                    points.append(Gf.Vec3f(key[0] * MM, key[1] * MM, z))
                indices.append(vmap[key])
    return points, indices


def make_mesh(stage, path, polys, z, look, display_hex):
    points, indices = triangulate(polys, z)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * (len(indices) // 3)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)]))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.constant)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(False)
    mesh.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(Vt.Vec3fArray(points)))
    mesh.CreateDisplayColorAttr(Vt.Vec3fArray([srgb_to_linear(display_hex)]))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(look)
    return mesh


def add_class_label(prim, label):
    """Isaac Sim 5.x 시맨틱 라벨 (UsdSemantics.LabelsAPI, instance 'class')."""
    prim.AddAppliedSchema("SemanticsLabelsAPI:class")
    prim.CreateAttribute("semantics:labels:class", Sdf.ValueTypeNames.TokenArray).Set(Vt.TokenArray([label]))


def element_polygon(e):
    return Polygon(e["exterior"], e["holes"])


def build(cfg, elements_json, out_path):
    data = json.loads(elements_json.read_text())
    W, H = data["size_mm"]
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/AutoRaceTrack").GetPrim()
    stage.SetDefaultPrim(root)
    Usd.ModelAPI(root).SetKind("assembly")
    root.SetCustomDataByKey("autorace:source", data["source"])
    root.SetCustomDataByKey("autorace:frame", "origin = mat bottom-left corner, +x = drawing right, +y = drawing up, z = 0 is mat top")
    root.SetCustomDataByKey("autorace:size_m", Gf.Vec2d(W * MM, H * MM))
    sp = data.get("start_pose")
    if sp:   # 출발선(체크무늬) 가운데, 진행 방향 (dxf_classify.start_pose)
        root.SetCustomDataByKey("autorace:start_pose", Gf.Vec3d(sp["x"] * MM, sp["y"] * MM, sp["yaw_deg"]))

    # 재질
    UsdGeom.Scope.Define(stage, "/AutoRaceTrack/Looks")
    looks = {"mat": make_look(stage, "/AutoRaceTrack/Looks/Mat", cfg["mat"]["color"], cfg["mat"]["roughness"])}
    for name, hex_color in cfg["paint"]["colors"].items():
        looks[name] = make_look(stage, f"/AutoRaceTrack/Looks/Paint_{name}", hex_color, cfg["paint"]["roughness"])

    # 물리: 재질 + 충돌 평면 (z = 0)
    ph = cfg["physics"]
    UsdGeom.Scope.Define(stage, "/AutoRaceTrack/Physics")
    pmat = UsdShade.Material.Define(stage, "/AutoRaceTrack/Physics/FloorMaterial")
    papi = UsdPhysics.MaterialAPI.Apply(pmat.GetPrim())
    papi.CreateStaticFrictionAttr(float(ph["static_friction"]))
    papi.CreateDynamicFrictionAttr(float(ph["dynamic_friction"]))
    papi.CreateRestitutionAttr(float(ph["restitution"]))
    pmat.GetPrim().AddAppliedSchema("PhysxMaterialAPI")
    pmat.GetPrim().CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token,
                                   custom=False, variability=Sdf.VariabilityUniform).Set(ph["friction_combine_mode"])

    # 충돌은 무한 평면(PhysX plane). 얇고 넓은 박스는 바퀴(원기둥)와 접촉이 깨져 바퀴가 바닥에 파묻힌다.
    margin = float(cfg["venue"]["margin_m"])
    col = UsdGeom.Plane.Define(stage, "/AutoRaceTrack/Physics/FloorCollider")
    col.CreateAxisAttr(UsdGeom.Tokens.z)
    col.CreateWidthAttr(W * MM + 2 * margin)              # 표시용 크기 (물리는 무한 평면)
    col.CreateLengthAttr(H * MM + 2 * margin)
    col.AddTranslateOp().Set(Gf.Vec3d(W * MM / 2, H * MM / 2, 0.0))
    col.CreatePurposeAttr(UsdGeom.Tokens.guide)          # 렌더·카메라에 안 보이게
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(col.GetPrim()).Bind(
        pmat, UsdShade.Tokens.weakerThanDescendants, "physics")

    # 보이는 바닥
    UsdGeom.Xform.Define(stage, "/AutoRaceTrack/Ground")
    mat_mesh = make_mesh(stage, "/AutoRaceTrack/Ground/Mat", [Polygon([(0, 0), (W, 0), (W, H), (0, H)])],
                         0.0, looks["mat"], cfg["mat"]["color"])
    add_class_label(mat_mesh.GetPrim(), "track_floor")
    if margin > 0:
        m = margin / MM
        venue_look = make_look(stage, "/AutoRaceTrack/Looks/Venue", cfg["venue"]["color"], cfg["venue"]["roughness"])
        venue = make_mesh(stage, "/AutoRaceTrack/Ground/Venue",
                          [Polygon([(-m, -m), (W + m, -m), (W + m, H + m), (-m, H + m)])],
                          float(cfg["venue"]["z_m"]), venue_look, cfg["venue"]["color"])
        add_class_label(venue.GetPrim(), "venue_floor")

    # 노면표시
    z = float(cfg["paint"]["height_m"])
    class_color = cfg["paint"]["class_color"]
    UsdGeom.Xform.Define(stage, "/AutoRaceTrack/Markings")
    counts = {}
    for e in data["elements"]:
        if not e["painted"]:
            continue
        c = e["class"]
        if c not in class_color:
            raise SystemExit(f"track_build.yaml paint.class_color 에 '{c}' 색이 없음")
        if c not in counts:
            UsdGeom.Xform.Define(stage, f"/AutoRaceTrack/Markings/{c}")
            counts[c] = 0
        color = class_color[c]
        mesh = make_mesh(stage, f"/AutoRaceTrack/Markings/{c}/{c}_{counts[c]:02d}", [element_polygon(e)],
                         z, looks[color], cfg["paint"]["colors"][color])
        prim = mesh.GetPrim()
        add_class_label(prim, c)
        prim.SetCustomDataByKey("autorace:element_id", e["id"])
        prim.SetCustomDataByKey("autorace:centroid_m", Gf.Vec2d(e["centroid"][0] * MM, e["centroid"][1] * MM))
        counts[c] += 1
    stage.GetRootLayer().Save()
    return counts


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", type=Path, default=PKG / "config/track_build.yaml")
    ap.add_argument("--static-friction", type=float, help="config 값 덮어쓰기")
    ap.add_argument("--dynamic-friction", type=float, help="config 값 덮어쓰기")
    ap.add_argument("--json", type=Path, help="분류 JSON (기본: config input_json)")
    ap.add_argument("--out", type=Path, help="출력 USD (기본: config output_usd)")
    a = ap.parse_args()
    cfg = yaml.safe_load(a.config.read_text())
    if a.static_friction is not None:
        cfg["physics"]["static_friction"] = a.static_friction
    if a.dynamic_friction is not None:
        cfg["physics"]["dynamic_friction"] = a.dynamic_friction
    out = a.out or PKG / cfg["output_usd"]
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        out.unlink()
    counts = build(cfg, a.json or PKG / cfg["input_json"], out)
    ph = cfg["physics"]
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")
    print(f"  floor friction static {ph['static_friction']} / dynamic {ph['dynamic_friction']}, "
          f"restitution {ph['restitution']}, combine {ph['friction_combine_mode']}")
    print("  markings:", ", ".join(f"{k} {v}" for k, v in counts.items()))


if __name__ == "__main__":
    main()
