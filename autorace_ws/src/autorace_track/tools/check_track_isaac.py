"""트랙 USD를 Isaac Sim에서 열어 (1) 미리보기 렌더, (2) 바닥 마찰 적용 여부를 확인한다.

실행:  ~/isaacsim/python.sh tools/check_track_isaac.py [--usd usd/autorace_track.usda] [--no-render]

마찰 확인: 0.2×0.2×0.05 m 박스를 v0로 밀어 미끄러진 거리 d를 재고 μ_eff = v0² / (2 g d) 를 계산.
박스 재질은 마찰 1.0 + combine 'min' 이라 μ_eff = 바닥 dynamic friction 이 되어야 한다.
런타임 덮어쓰기(params_sim.yaml 용 경로)도 같이 확인: 바닥 재질 값을 바꾼 뒤 다시 잰다.
"""
import argparse
import sys
from pathlib import Path

sys.stdout.reconfigure(line_buffering=True)   # app.close()가 버퍼를 비우지 않고 종료하므로

PKG = Path(__file__).resolve().parent.parent
ap = argparse.ArgumentParser()
ap.add_argument("--usd", type=Path, default=PKG / "usd/autorace_track.usda")
ap.add_argument("--out-dir", type=Path, default=PKG / "data/generated")
ap.add_argument("--no-render", action="store_true")
ap.add_argument("--override-friction", type=float, default=0.35)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 1600, "height": 960})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleRigidPrim  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

TRACK = "/World/Track"
FLOOR_PMAT = f"{TRACK}/Physics/FloorMaterial"

omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
stage.DefinePrim(TRACK, "Xform").GetReferences().AddReference(str(args.usd))

dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
dome.CreateIntensityAttr(600)
sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
sun.CreateIntensityAttr(2500)
sun.CreateAngleAttr(1.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35, 0, 25))


def look_at_camera(path, eye, target, focal_mm=18.0, up=(0, 0, 1)):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal_mm)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955 * 960 / 1600)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1e4))   # 기본 near=1 m 이면 가까운 바닥이 잘린다
    f = (Gf.Vec3d(*target) - Gf.Vec3d(*eye)).GetNormalized()
    r = Gf.Cross(f, Gf.Vec3d(*up))
    if r.GetLength() < 1e-6:
        r = Gf.Cross(f, Gf.Vec3d(0, 1, 0))
    r = r.GetNormalized()
    u = Gf.Cross(r, f)
    m = Gf.Matrix4d(r[0], r[1], r[2], 0, u[0], u[1], u[2], 0, -f[0], -f[1], -f[2], 0, eye[0], eye[1], eye[2], 1)
    UsdGeom.Xformable(cam).AddTransformOp().Set(m)
    return path


def render_views():
    views = {
        # 위에서 수직으로: 이미지 위쪽 = +y (도면과 같은 방향)
        "top": look_at_camera("/World/Cams/Top", (5.0, 3.0, 12.5), (5.0, 3.0, 0.0), focal_mm=22.0, up=(0, 1, 0)),
        "oblique": look_at_camera("/World/Cams/Oblique", (-1.2, -2.6, 4.2), (4.6, 2.6, 0.0), focal_mm=16.0),
        # LIMO 카메라 높이(≈0.2 m)에서 본 출발선 → 1차로
        "limo_start": look_at_camera("/World/Cams/LimoStart", (0.55, 5.58, 0.20), (2.2, 5.58, 0.0), focal_mm=12.0),
        # 오른쪽 내리막 직선: 지그재그 + 마름모 + 횡단보도
        "limo_zigzag": look_at_camera("/World/Cams/LimoZig", (9.58, 4.75, 0.20), (9.58, 3.2, 0.0), focal_mm=12.0),
    }
    rps, anns = {}, {}
    for name, cam in views.items():
        rps[name] = rep.create.render_product(cam, (1600, 960))
        anns[name] = rep.AnnotatorRegistry.get_annotator("rgb")
        anns[name].attach(rps[name])
    for _ in range(30):                     # 머티리얼·조명 수렴
        rep.orchestrator.step(rt_subframes=4)
    out = []
    for name, ann in anns.items():
        img = np.asarray(ann.get_data())[..., :3]
        p = args.out_dir / f"preview_{name}.png"
        Image.fromarray(img).save(p)
        out.append(p)
    return out


def slide_test(world, v0=1.5, seconds=2.5):
    world.reset()                          # 물리 재초기화 → 바닥 재질 변경도 반영
    box = world.scene.get_object("probe")
    box.set_world_pose(position=np.array([3.0, 2.8, 0.026]))
    box.set_linear_velocity(np.array([v0, 0.0, 0.0]))
    box.set_angular_velocity(np.zeros(3))
    x0 = box.get_world_pose()[0][0]
    for _ in range(int(seconds * 240)):
        world.step(render=False)
    pos = box.get_world_pose()[0]
    v = np.linalg.norm(box.get_linear_velocity())
    d = pos[0] - x0
    return d, v0 ** 2 / (2 * 9.81 * d), v, pos[2]


rendered = [] if args.no_render else render_views()

# ---- 마찰 확인
UsdPhysics.Scene.Define(stage, "/World/PhysicsScene").CreateGravityMagnitudeAttr(9.81)
box_path = "/World/FrictionProbe"
cube = UsdGeom.Cube.Define(stage, box_path)
cube.CreateSizeAttr(1.0)
cube.AddTranslateOp().Set(Gf.Vec3d(3.0, 2.8, 0.026))
cube.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.05))
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(1.0)
probe_mat = UsdShade.Material.Define(stage, "/World/ProbeMaterial")
pm = UsdPhysics.MaterialAPI.Apply(probe_mat.GetPrim())
pm.CreateStaticFrictionAttr(1.0)
pm.CreateDynamicFrictionAttr(1.0)
PhysxSchema.PhysxMaterialAPI.Apply(probe_mat.GetPrim()).CreateFrictionCombineModeAttr("min")
UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(probe_mat, UsdShade.Tokens.weakerThanDescendants, "physics")

world = World(physics_dt=1 / 240, rendering_dt=1 / 60, stage_units_in_meters=1.0)
world.scene.add(SingleRigidPrim(box_path, name="probe"))

floor = UsdPhysics.MaterialAPI(stage.GetPrimAtPath(FLOOR_PMAT))
mu_authored = floor.GetDynamicFrictionAttr().Get()
d1, mu1, v1, z1 = slide_test(world)

world.stop()
floor.GetDynamicFrictionAttr().Set(args.override_friction)
floor.GetStaticFrictionAttr().Set(args.override_friction)
d2, mu2, v2, z2 = slide_test(world)
world.stop()

print("=" * 70)
for p in rendered:
    print("rendered", p)
print(f"[friction] authored dynamic μ = {mu_authored:.3f}: slid {d1:.3f} m → μ_eff = {mu1:.3f} "
      f"(final |v| {v1:.3f} m/s, z {z1:.3f})")
print(f"[friction] runtime override μ = {args.override_friction:.3f}: slid {d2:.3f} m → μ_eff = {mu2:.3f} "
      f"(final |v| {v2:.3f} m/s, z {z2:.3f})")
ok = abs(mu1 - mu_authored) / mu_authored < 0.1 and abs(mu2 - args.override_friction) / args.override_friction < 0.1
print("[friction]", "OK" if ok else "MISMATCH")
print("=" * 70)
app.close()
