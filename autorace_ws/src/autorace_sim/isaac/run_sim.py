"""AutoRace 트랙 + LIMO 를 Isaac Sim 5.1 에 띄우고 ROS2 로 실차와 같은 인터페이스를 낸다.

보통은 `ros2 launch autorace_bringup bringup.launch.py mode:=sim` 이 띄운다. 단독 실행:
  ~/isaacsim/python.sh run_sim.py --params <params_sim.yaml> --track-usd <autorace_track.usda> [--headless]

ROS2 인터페이스 (이름은 params_sim.yaml, 기본값 = limo_ros2 humble 실차)
  sub  /cmd_vel                    geometry_msgs/Twist  → 4륜 스키드 디퍼렌셜 구동 (rclpy)
  pub  /odom, TF odom→base_link    시뮬레이터 기준 오도메트리 (출발 자세 = 원점)
  pub  /imu                        imu_link
  pub  /scan                       laser_link, PhysX 라이다 (EAI X2L 사양)
  pub  /camera/color/image_raw, /camera/color/camera_info
  pub  /isaac/camera/depth/image_raw (32FC1 m) → depth_converter 가 실차와 같은 16UC1 mm 로 바꿔 /camera/depth/image_raw
  pub  /camera/depth/camera_info, /camera/depth/points
  pub  /clock
  srv  /sim/reset                  std_srvs/Trigger: LIMO 출발 자세, 라바콘 원위치, 미션 주기 0 초부터 (Isaac 은 그대로)
  sub  /sim/cone_layout            std_msgs/String: 다음 리셋 때 쓸 라바콘 배치 (cone_layouts.NAMES 또는 random)
  sub  /sim/traffic_light          std_msgs/String: red | yellow | green = 신호등 색 고정 (시험용), cycle = 설정 주기
  sub  /sim/start_pose             geometry_msgs/Pose2D: 다음 리셋 한 번만 쓸 출발 자세 (world, 시험·데이터 수집·주차 단독 실행)
  sub  /sim/parking_layout         std_msgs/String: 다음 리셋 M4 주차 칸 정차 LIMO 배치 — default | seed=N (칸 안에서 옆 ±5 cm,
                                   깊이 ±6 cm, 방향 ±10°, 앞뒤 뒤집기 무작위) | "x,y,yaw_deg;x,y,yaw_deg"
  pub  /sim/parking_status         std_msgs/String (1 Hz): "limos=x,y,yaw_deg;…" 지금 주차 LIMO 자리 (시험 판정용)
  pub  /sim/traffic_light_state    std_msgs/String (5 Hz): 지금 켜진 색 (시험 판정용)
  pub  /sim/cone_status            std_msgs/String (1 Hz): "max_cm=… moved3=… tipped=… layout=… pos=x,y;…" — 이번 배치 자리에서
                                   라바콘이 밀린 거리, 3 cm 넘게 밀린 수, 넘어진 수, 지금 위치 (시험 판정용)
센서 정적 TF(base_link→laser_link 등)는 실차와 같은 방식으로 launch 의 static_transform_publisher 가 낸다.
"""
import argparse
import math
import random
import signal
import sys
import time
from pathlib import Path

import yaml

sys.stdout.reconfigure(line_buffering=True)   # SimulationApp.close()가 버퍼를 비우지 않고 끝나므로

ap = argparse.ArgumentParser()
ap.add_argument("--params", required=True, type=Path)
ap.add_argument("--track-usd", required=True)
ap.add_argument("--headless", action="store_true")
ap.add_argument("--max-sim-seconds", type=float, default=0.0, help="테스트용: 시뮬레이션 시간이 지나면 종료")
args = ap.parse_args()

doc = yaml.safe_load(args.params.read_text())
P = doc["isaac_sim"]["ros__parameters"]
MOUNTS = doc["sensor_mounts"]["ros__parameters"]
T, F = P["topics"], P["frames"]

from isaacsim import SimulationApp  # noqa: E402

# anti_aliasing: 0 끔, 1 TAA, 2 FXAA, 3 DLSS (Isaac 기본), 4 RTXAA. 차선·표지판 인식은 DLSS 로 맞췄다 (FXAA 면 M8 뒤 커브에서
# 차선 이탈). 참고: 카메라가 멈춰 있을 때 신호등 램프를 바꾸면 꺼진 램프 자리에 빛이 몇~십몇 초 남는 일이 있다 (렌더러 누적)
app = SimulationApp({"headless": args.headless or bool(P["headless"]), "width": 1280, "height": 720,
                     "anti_aliasing": int(P.get("anti_aliasing", 3))})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
app.update()

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.physx  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
import rclpy  # noqa: E402  (브리지가 내장 humble rclpy 경로를 잡아 준다)
import usdrt.Sdf  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import RigidPrim, SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from limo_drive import LimoDrive, TireModel, body_twist, quat_to_rot  # noqa: E402
from missions import MissionSwitcher  # noqa: E402
import cone_layouts  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from geometry_msgs.msg import Pose2D  # noqa: E402

LIMO_ASSET = "/Isaac/Robots/AgilexRobotics/limo/limo.usd"
WHEELS = ("front_left_wheel", "rear_left_wheel", "front_right_wheel", "rear_right_wheel")   # 왼쪽 2, 오른쪽 2


def quat_rpy(roll, pitch, yaw):
    """ROS rpy (rad, tf2 setRPY 와 같은 정의) → Gf.Quatf."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Gf.Quatf(cr * cp * cy + sr * sp * sy,
                    Gf.Vec3f(sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy))


def topic(name):
    return name.lstrip("/")


# ---------------------------------------------------------------- 무대
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / P["physics_hz"], rendering_dt=1.0 / P["render_hz"])
stage = omni.usd.get_context().get_stage()

dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
dome.CreateIntensityAttr(float(P["lighting"]["dome_intensity"]))
sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
sun.CreateIntensityAttr(float(P["lighting"]["sun_intensity"]))
sun.CreateAngleAttr(1.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35, 0, 25))

stage.DefinePrim("/World/Track", "Xform").GetReferences().AddReference(args.track_usd)
floor = UsdPhysics.MaterialAPI(stage.GetPrimAtPath("/World/Track/Physics/FloorMaterial"))
floor.GetStaticFrictionAttr().Set(float(P["track"]["floor_static_friction"]))
floor.GetDynamicFrictionAttr().Set(float(P["track"]["floor_dynamic_friction"]))

# 미션 시설물: LimoIsaacSIM/autorace_2026 의 tools/build_mission_usd.py 가 만든 USD (트랙과 같은 좌표).
# 신호등 색·갈림길 A/B 표지판·광고 화면은 MissionSwitcher 가 시뮬레이션 시간 주기로 바꾼다 (아래 물리 콜백).
M = P["missions"]
missions = None
if M["enabled"]:
    mission_usd = str(Path(M["usd"]).expanduser())
    layer = Sdf.Layer.FindOrOpen(mission_usd)
    if layer is None:
        carb.log_error(f"[autorace_sim] 미션 USD 없음: {mission_usd} → 시설물 없이 진행")
    else:
        switch = layer.GetPrimAtPath(layer.defaultPrim).customData.get("autorace", {}).get("switch", {})
        stage.DefinePrim("/World/Missions", "Xform").GetReferences().AddReference(mission_usd)
        missions = MissionSwitcher(stage, "/World/Missions", switch, {
            "light_s": {c: float(M["traffic_light_s"][c]) for c in ("red", "yellow", "green")},
            "light_start": str(M["traffic_light_start"]),
            "branch_period": float(M["branch_sign_period_s"]),
            "branch_start": str(M["branch_sign_start"]).upper(),
            "branch_random": bool(M["branch_sign_random"]),
            "slide_period": float(M["ad_slide_period_s"]),
        })

# ---------------------------------------------------------------- LIMO
R = P["robot"]
robot_path = R["prim_path"]
robot_usd = str(Path(R["usd"]).expanduser()) if R["usd"] else get_assets_root_path() + LIMO_ASSET
stage.DefinePrim(robot_path, "Xform").GetReferences().AddReference(robot_usd)
base_path = f"{robot_path}/chassis_link/base_link"     # limo.usd: base_link = chassis_link 과 같은 자세


def mount_frame(name):
    m = MOUNTS[name]
    xf = UsdGeom.Xform.Define(stage, f"{base_path}/{name}")
    xf.AddTranslateOp().Set(Gf.Vec3d(*m["xyz"]))
    xf.AddOrientOp().Set(quat_rpy(*m["rpy"]))
    return xf.GetPath().pathString


# 라이다 (PhysX 레이캐스트, 2D: 수직 1줄)
L = P["lidar"]
laser_path = mount_frame(F["laser"])
_, lidar_prim = omni.kit.commands.execute(
    "RangeSensorCreateLidar", path="/lidar", parent=laser_path,
    min_range=float(L["min_range"]), max_range=float(L["max_range"]),
    draw_points=False, draw_lines=False, horizontal_fov=360.0, vertical_fov=1.0,
    horizontal_resolution=float(L["resolution_deg"]), vertical_resolution=1.0,
    rotation_rate=0.0, high_lod=False, yaw_offset=0.0, enable_semantics=False)
lidar_path = lidar_prim.GetPath().pathString

# IMU
imu_frame_path = mount_frame(F["imu"])
_, imu_prim = omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor", path="/imu", parent=imu_frame_path,
    sensor_period=1.0 / float(P["imu"]["rate_hz"]))
imu_path = imu_prim.GetPath().pathString

# 카메라: camera_link(+X 전방, +Z 위) 아래에 USD 카메라(-Z 전방, +Y 위)를 돌려 붙인다
C = P["camera"]
cam_link_path = mount_frame(F["camera"])
cam = UsdGeom.Camera.Define(stage, f"{cam_link_path}/camera_sensor")
cam_rot = Gf.Matrix3d(0, -1, 0,   # 카메라 X(오른쪽) = 차체 -Y
                      0, 0, 1,    # 카메라 Y(위)     = 차체 +Z
                      -1, 0, 0)   # 카메라 Z(뒤)     = 차체 -X
cam.AddOrientOp().Set(Gf.Quatf(cam_rot.ExtractRotation().GetQuat()))
aperture = 20.955
cam.CreateHorizontalApertureAttr(aperture)
cam.CreateVerticalApertureAttr(aperture * C["height"] / C["width"])
cam.CreateFocalLengthAttr(aperture / 2 / math.tan(math.radians(C["hfov_deg"]) / 2))
cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))   # 기본 near=1 m 면 바로 앞 바닥이 잘린다
cam_path = cam.GetPath().pathString

start_pos = np.array([R["start_x"], R["start_y"], R["start_z"]])
start_quat = np.array([math.cos(math.radians(R["start_yaw_deg"]) / 2), 0, 0,
                       math.sin(math.radians(R["start_yaw_deg"]) / 2)])
robot = world.scene.add(SingleArticulation(robot_path, name="limo", position=start_pos, orientation=start_quat))

# 미션 시설물 중 물리로 움직이는 것 (M8 라바콘) — /sim/reset 때 처음 자리로
props = None
if missions is not None:
    paths = [p.GetPath().pathString for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Missions"))
             if p.HasAPI(UsdPhysics.RigidBodyAPI) and not UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Get()]
    if paths:
        props = world.scene.add(RigidPrim(paths, name="mission_props"))

# ---------------------------------------------------------------- ROS2 OmniGraph (센서·odom·clock)
rp_path = rep.create.render_product(cam_path, (int(C["width"]), int(C["height"]))).path
keys = og.Controller.Keys
sim_time = "sim_time.outputs:simulationTime"
lidar_step = max(1, round(P["render_hz"] / L["rate_hz"]))
imu_step = max(1, round(P["physics_hz"] / P["imu"]["rate_hz"]))
odom_step = max(1, round(P["physics_hz"] / P["odom_rate_hz"]))
cam_skip = max(0, round(P["render_hz"] / C["rate_hz"]) - 1)

# 렌더 틱 그래프: clock, 라이다, 카메라
nodes = [
    ("tick", "omni.graph.action.OnPlaybackTick"),
    ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
    ("lidar_gate", "isaacsim.core.nodes.IsaacSimulationGate"),
    ("lidar_read", "isaacsim.sensors.physx.IsaacReadLidarBeams"),
    ("scan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
    ("cam_rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ("cam_depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ("cam_pcl", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ("cam_info_rgb", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
    ("cam_info_depth", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
]
connect = [
    ("tick.outputs:tick", "clock.inputs:execIn"),
    (sim_time, "clock.inputs:timeStamp"),
    ("tick.outputs:tick", "lidar_gate.inputs:execIn"),
    ("lidar_gate.outputs:execOut", "lidar_read.inputs:execIn"),
    ("lidar_read.outputs:execOut", "scan.inputs:execIn"),
    (sim_time, "scan.inputs:timeStamp"),
]
for out in ("azimuthRange", "depthRange", "horizontalFov", "horizontalResolution", "intensitiesData",
            "linearDepthData", "numCols", "numRows", "rotationRate"):
    connect.append((f"lidar_read.outputs:{out}", f"scan.inputs:{out}"))
for n in ("cam_rgb", "cam_depth", "cam_pcl", "cam_info_rgb", "cam_info_depth"):
    connect.append(("tick.outputs:tick", f"{n}.inputs:execIn"))
values = [
    ("clock.inputs:topicName", "clock"),
    ("lidar_gate.inputs:step", lidar_step),
    ("lidar_read.inputs:lidarPrim", [usdrt.Sdf.Path(lidar_path)]),
    ("scan.inputs:topicName", topic(T["scan"])),
    ("scan.inputs:frameId", F["laser"]),
]
for n, typ, top, frame in (("cam_rgb", "rgb", T["color_image"], F["color_optical"]),
                           ("cam_depth", "depth", T["isaac_depth_image"], F["depth_optical"]),
                           ("cam_pcl", "depth_pcl", T["points"], F["depth_optical"])):
    values += [(f"{n}.inputs:renderProductPath", rp_path), (f"{n}.inputs:type", typ),
               (f"{n}.inputs:topicName", topic(top)), (f"{n}.inputs:frameId", frame),
               (f"{n}.inputs:frameSkipCount", cam_skip)]
for n, top, frame in (("cam_info_rgb", T["color_info"], F["color_optical"]),
                      ("cam_info_depth", T["depth_info"], F["depth_optical"])):
    values += [(f"{n}.inputs:renderProductPath", rp_path), (f"{n}.inputs:topicName", topic(top)),
               (f"{n}.inputs:frameId", frame), (f"{n}.inputs:frameSkipCount", cam_skip)]
og.Controller.edit({"graph_path": "/World/ROS_Graph", "evaluator_name": "execution"},
                   {keys.CREATE_NODES: nodes, keys.CONNECT: connect, keys.SET_VALUES: values})

# 물리 스텝 그래프: IMU, 오도메트리 (OnPhysicsStep 은 on-demand 파이프라인에서만 돈다)
nodes = [
    ("phys", "isaacsim.core.nodes.OnPhysicsStep"),
    ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("imu_gate", "isaacsim.core.nodes.IsaacSimulationGate"),
    ("imu_read", "isaacsim.sensors.physics.IsaacReadIMU"),
    ("imu_pub", "isaacsim.ros2.bridge.ROS2PublishImu"),
    ("odom_gate", "isaacsim.core.nodes.IsaacSimulationGate"),
    ("odom_calc", "isaacsim.core.nodes.IsaacComputeOdometry"),
    ("odom_pub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
]
connect = [
    ("phys.outputs:step", "imu_gate.inputs:execIn"),
    ("imu_gate.outputs:execOut", "imu_read.inputs:execIn"),
    ("imu_read.outputs:execOut", "imu_pub.inputs:execIn"),
    ("imu_read.outputs:angVel", "imu_pub.inputs:angularVelocity"),
    ("imu_read.outputs:linAcc", "imu_pub.inputs:linearAcceleration"),
    ("imu_read.outputs:orientation", "imu_pub.inputs:orientation"),
    (sim_time, "imu_pub.inputs:timeStamp"),
    ("phys.outputs:step", "odom_gate.inputs:execIn"),
    ("odom_gate.outputs:execOut", "odom_calc.inputs:execIn"),
    ("odom_calc.outputs:execOut", "odom_pub.inputs:execIn"),
    ("odom_calc.outputs:position", "odom_pub.inputs:position"),
    ("odom_calc.outputs:orientation", "odom_pub.inputs:orientation"),
    ("odom_calc.outputs:linearVelocity", "odom_pub.inputs:linearVelocity"),
    ("odom_calc.outputs:angularVelocity", "odom_pub.inputs:angularVelocity"),
    (sim_time, "odom_pub.inputs:timeStamp"),
]
values = [
    ("imu_gate.inputs:step", imu_step),
    ("imu_read.inputs:imuPrim", [usdrt.Sdf.Path(imu_path)]),
    ("imu_read.inputs:readGravity", True),
    ("imu_pub.inputs:topicName", topic(T["imu"])),
    ("imu_pub.inputs:frameId", F["imu"]),
    ("odom_gate.inputs:step", odom_step),
    ("odom_calc.inputs:chassisPrim", [usdrt.Sdf.Path(f"{robot_path}/chassis_link")]),
    ("odom_pub.inputs:topicName", topic(T["odom"])),
    ("odom_pub.inputs:odomFrameId", F["odom"]),
    ("odom_pub.inputs:chassisFrameId", F["base"]),
]
if P["publish_odom_tf"]:   # 실차 limo_base 의 pub_odom_tf 와 같은 역할
    nodes.append(("odom_tf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
    connect += [("odom_calc.outputs:execOut", "odom_tf.inputs:execIn"),
                ("odom_calc.outputs:position", "odom_tf.inputs:translation"),
                ("odom_calc.outputs:orientation", "odom_tf.inputs:rotation"),
                (sim_time, "odom_tf.inputs:timeStamp")]
    values += [("odom_tf.inputs:parentFrameId", F["odom"]), ("odom_tf.inputs:childFrameId", F["base"])]
og.Controller.edit({"graph_path": "/World/ROS_PhysicsGraph", "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
                   {keys.CREATE_NODES: nodes, keys.CONNECT: connect, keys.SET_VALUES: values})

# ---------------------------------------------------------------- /cmd_vel → 바퀴 속도
rclpy.init()
ros_node = rclpy.create_node("limo_sim_base")
cmd = {"v": 0.0, "w": 0.0, "stamp": -math.inf}


def on_cmd_vel(msg):
    cmd.update(v=msg.linear.x, w=msg.angular.z, stamp=time.monotonic())


ros_node.create_subscription(Twist, T["cmd_vel"], on_cmd_vel, 10)

D = R["drive"]
tire = None
if D["model"] == "tire":
    # 바퀴 충돌체는 하중만 받게 마찰 0 (combine 'min' 이라 바닥 마찰과 무관), 접지력은 TireModel 이 건다
    nofric = UsdShade.Material.Define(stage, "/World/Physics/WheelNoFriction")
    m = UsdPhysics.MaterialAPI.Apply(nofric.GetPrim())
    m.CreateStaticFrictionAttr(0.0)
    m.CreateDynamicFrictionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(nofric.GetPrim()).CreateFrictionCombineModeAttr("min")
    for name in WHEELS:
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(f"{robot_path}/{name}_link/collisions")).Bind(
            nofric, UsdShade.Tokens.strongerThanDescendants, "physics")
    links = ["chassis_link"] + [f"{n}_link" for n in WHEELS]
    mass = sum(UsdPhysics.MassAPI(stage.GetPrimAtPath(f"{robot_path}/{n}")).GetMassAttr().Get() for n in links)
    com_local = np.array(UsdPhysics.MassAPI(stage.GetPrimAtPath(f"{robot_path}/chassis_link")).GetCenterOfMassAttr().Get())
    half_l, half_w = float(R["wheelbase"]) / 2, float(R["track_width"]) / 2
    tire = TireModel([(half_l, half_w), (-half_l, half_w), (half_l, -half_w), (-half_l, -half_w)],
                     ground_z=-float(R["base_height"]), c_long=D["tire"]["c_long"], c_lat=D["tire"]["c_lat"],
                     mass=mass, mu=float(P["track"]["floor_static_friction"]))
    physx_sim = omni.physx.get_physx_simulation_interface()
    stage_id = omni.usd.get_context().get_stage_id()
    chassis_id = PhysicsSchemaTools.sdfPathToInt(f"{robot_path}/chassis_link")
elif D["model"] != "physx":
    raise SystemExit(f"robot.drive.model 은 tire 또는 physx: {D['model']}")

drive_cfg = dict(R, max_wheel_speed=D["max_wheel_speed"], tracking=D["tracking"])
limo = LimoDrive(drive_cfg, yaw_ratio=tire.yaw_ratio() if (tire and D["tracking"]["enabled"]) else 1.0)

world.reset()
dof = {name: i for i, name in enumerate(robot.dof_names)}
wheel_idx = np.array([dof[n] for n in WHEELS])
r = float(R["wheel_radius"])
props_pose0 = props.get_world_poses() if props is not None else None
# M8 라바콘: props 중 Cone_NN 의 순서 (USD Cone_00, 01 = 첫 쌍 왼쪽·오른쪽 …) → 배치를 바꿀 수 있게
cone_ix = sorted((i for i, p in enumerate(props.prim_paths) if "/M8/Cone_" in p),
                 key=lambda i: props.prim_paths[i]) if props is not None else []
CONES = M.get("cones", {}) if missions is not None else {}
cone_ref = {"layout": "default", "pos": props_pose0[0].copy() if props is not None else None,
            "next": str(CONES.get("layout", "default"))}
mission_t0 = {"t": 0.0}     # 미션 주기의 0 초 = 마지막 리셋 시각 (시뮬 시간)
live = {"ok": True}         # 물리 핸들이 유효한가. GUI ■ 뒤 ▶ 를 누르면 world.reset() 전까지 False (속도가 None 으로 읽힘)


def drive(dt):
    if not live["ok"]:
        return
    fresh =time.monotonic() - cmd["stamp"] < float(R["cmd_vel_timeout"])
    v_ref, w_ref = (cmd["v"], cmd["w"]) if fresh else (0.0, 0.0)
    pos, quat = robot.get_world_pose()
    lin, ang = robot.get_linear_velocity(), robot.get_angular_velocity()
    wl, wr = limo.wheel_speeds(v_ref, w_ref, *body_twist(lin, ang, quat), dt)
    speeds = np.array([wl, wl, wr, wr])
    robot.apply_action(ArticulationAction(joint_velocities=speeds, joint_indices=wheel_idx))
    if tire is not None:
        rot = quat_to_rot(quat)
        for force, at in tire.forces(speeds * r, pos, rot, pos + rot @ com_local, lin, ang):
            physx_sim.apply_force_at_pos(stage_id, chassis_id, carb.Float3(*force), carb.Float3(*at), "Force")


def switch_missions(dt):
    if live["ok"]:
        missions.update(world.current_time - mission_t0["t"])


world.add_physics_callback("limo_drive", drive)
if missions is not None:
    world.add_physics_callback("missions", switch_missions)


# M4 주차 칸 정차 LIMO (정적 충돌체 — USD 변환을 바꾸면 PhysX 도 따라 옮긴다)
park_prims = sorted((p for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Missions")) if "/M4/Limo_" in p.GetPath().pathString
                     and p.GetPath().pathString.count("/") == 4), key=lambda p: p.GetPath().pathString) if missions is not None else []
park0 = [(tuple(p.GetAttribute("xformOp:translate").Get()), float(p.GetAttribute("xformOp:rotateZ").Get())) for p in park_prims]
park_ref = {"next": "default", "now": [(t[0], t[1], r) for t, r in park0]}


start_once = {}


def park_layout(name):
    if name.startswith("seed="):
        rng = random.Random(int(name[5:]))
        out = []
        for (x, y, _), r in park0:
            out.append((x + rng.uniform(-0.05, 0.05), y + rng.uniform(-0.06, 0.06),
                        r + rng.uniform(-10, 10) + (180.0 if rng.random() < 0.5 else 0.0)))
        return out
    if ";" in name or "," in name:
        return [tuple(float(v) for v in q.split(",")) for q in name.split(";") if q.strip()]
    return [(t[0], t[1], r) for t, r in park0]


def reset_sim():
    """Isaac 을 다시 띄우지 않고 처음 상태로: LIMO 출발 자세·정지, 라바콘 원위치, 미션 주기 0 초부터.
    시뮬 시간(/clock)은 되돌리지 않는다 (ROS 노드들이 시간 역행을 겪지 않게). /odom 은 출발 자세 기준이라 0 으로 돌아온다."""
    start_p, start_q = start_once.pop("pose", (start_pos, start_quat))
    robot.set_world_pose(start_p, start_q)
    robot.set_linear_velocity(np.zeros(3))
    robot.set_angular_velocity(np.zeros(3))
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    limo.reset()
    cmd.update(v=0.0, w=0.0, stamp=-math.inf)
    moved = 0
    layout = ""
    if props is not None:
        pos, _ = props.get_world_poses()
        moved = int((np.linalg.norm(pos - cone_ref["pos"], axis=1) > 0.01).sum())
        ref = props_pose0[0].copy()
        name = cone_ref["next"]
        if name == "random":
            name = rng_layout.choice(cone_layouts.NAMES)
        if cone_ix and name != "default" and CONES.get("start"):
            xy = cone_layouts.positions(name, CONES, rng_layout)
            for k, i in enumerate(cone_ix[:len(xy)]):
                ref[i, 0], ref[i, 1] = xy[k]
        cone_ref["layout"], cone_ref["pos"] = name, ref
        props.set_world_poses(ref, props_pose0[1])
        props.set_velocities(np.zeros((len(pos), 6)))
        layout = f", 라바콘 배치 {name}"
    if park_prims:
        lay = park_layout(park_ref["next"])
        for p, (x, y, r), ((_, _, z), _) in zip(park_prims, lay, park0):
            p.GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, y, z))
            p.GetAttribute("xformOp:rotateZ").Set(float(r))
        park_ref["now"] = lay[:len(park_prims)]
        layout += f", 주차 LIMO {park_ref['next']}"
    if missions is not None:
        mission_t0["t"] = world.current_time
        missions.reset()
    return (f"LIMO 출발 자세 ({start_p[0]:.2f}, {start_p[1]:.2f}) 로, 움직였던 시설물 {moved} 개 원위치{layout}, 미션 주기 0 초부터 "
            f"(시뮬 t={world.current_time:.1f} s)")


def on_reset(_req, res):
    if not live["ok"]:
        res.success, res.message = False, "시뮬이 정지(■) 상태 — ▶ 를 누르면 처음 상태로 시작한다"
    else:
        res.success, res.message = True, reset_sim()
    print(f"[autorace_sim] reset: {res.message}", flush=True)
    return res


ros_node.create_service(Trigger, P["reset_service"], on_reset)


def on_traffic_light(msg):
    """시험용: red | yellow | green = 그 색 고정, cycle = 설정 주기."""
    if missions is None:
        return
    c = msg.data.strip().lower()
    missions.light_override = c if c in ("red", "yellow", "green") else None
    print(f"[autorace_sim] 신호등: {c if missions.light_override else '주기대로'}", flush=True)


ros_node.create_subscription(String, "/sim/traffic_light", on_traffic_light, 10)


def on_start_pose(msg):
    """시험·데이터 수집·주차 단독 실행용: 다음 /sim/reset 한 번만 쓸 출발 자세 (world x, y [m], theta [rad]). 그 뒤 리셋은 다시
    params 출발 자세. /odom 은 그 자세 기준 0 이 된다."""
    start_once["pose"] = (np.array([msg.x, msg.y, start_pos[2]]), np.array([math.cos(msg.theta / 2), 0, 0, math.sin(msg.theta / 2)]))
    print(f"[autorace_sim] 다음 리셋 출발 자세: ({msg.x:.3f}, {msg.y:.3f}) yaw {math.degrees(msg.theta):.1f} deg", flush=True)


ros_node.create_subscription(Pose2D, "/sim/start_pose", on_start_pose, 10)


def on_parking_layout(msg):
    park_ref["next"] = msg.data.strip() or "default"
    print(f"[autorace_sim] 다음 리셋 주차 LIMO 배치: {park_ref['next']}", flush=True)


ros_node.create_subscription(String, "/sim/parking_layout", on_parking_layout, 10)
park_pub = ros_node.create_publisher(String, "/sim/parking_status", 10)
park_stat = {"t": -1.0}
light_pub = ros_node.create_publisher(String, "/sim/traffic_light_state", 10)
light_stat = {"t": -1.0}
rng_layout = cone_layouts.random.Random()   # 배치 random·jitter 용


def on_cone_layout(msg):
    name = msg.data.strip()
    if name in cone_layouts.NAMES + ("random",):
        cone_ref["next"] = name
        print(f"[autorace_sim] 다음 리셋 라바콘 배치: {name}", flush=True)
    else:
        print(f"[autorace_sim] 모르는 라바콘 배치 '{name}' — {cone_layouts.NAMES + ('random',)}", flush=True)


ros_node.create_subscription(String, "/sim/cone_layout", on_cone_layout, 10)
cone_pub = ros_node.create_publisher(String, "/sim/cone_status", 10)
cone_stat = {"t": -1.0}


def publish_cone_status():
    """라바콘이 이번 배치 자리에서 얼마나 밀렸나 (1 Hz) — M8 시험 판정용."""
    if props is None or not cone_ix or world.current_time - cone_stat["t"] < 1.0:
        return
    cone_stat["t"] = world.current_time
    pos, _ = props.get_world_poses()
    d = np.linalg.norm((pos - cone_ref["pos"])[cone_ix, :2], axis=1)
    cone_pub.publish(String(data=f"max_cm={d.max() * 100:.1f} moved3={int((d > 0.03).sum())} "
                                 f"tipped={int((pos[cone_ix, 2] < cone_ref['pos'][cone_ix, 2] - 0.02).sum())} layout={cone_ref['layout']} "
                                 f"pos=" + ";".join(f"{p[0]:.3f},{p[1]:.3f}" for p in pos[cone_ix, :2])))

set_camera_view(eye=[R["start_x"] - 1.6, R["start_y"] - 1.4, 1.2],
                target=[R["start_x"] + 1.0, R["start_y"], 0.0], camera_prim_path="/OmniverseKit_Persp")

print(f"[autorace_sim] ready: LIMO at ({R['start_x']}, {R['start_y']}) yaw {R['start_yaw_deg']} deg, "
      f"floor μs/μd = {P['track']['floor_static_friction']}/{P['track']['floor_dynamic_friction']}, "
      f"drive {D['model']} (tracking {D['tracking']['enabled']}, yaw ratio {tire.yaw_ratio() if tire else 1.0:.2f}), "
      f"dofs {robot.dof_names}, missions {'on' if missions else 'off'}, reset service {P['reset_service']}")

stop = {"flag": False}
lidar_z_logged = {"done": False}


def request_stop(signum, _frame):
    print(f"[autorace_sim] signal {signum} → 종료", flush=True)
    stop["flag"] = True


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)

while app.is_running() and not stop["flag"]:
    if not world.is_playing():             # GUI 에서 일시정지(‖) 또는 정지(■)한 상태
        if world.is_stopped():
            live["ok"] = False
        app.update()
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        continue
    if not live["ok"]:                     # ■ 뒤 ▶: 물리 핸들을 다시 잡아야 /cmd_vel 로 움직인다
        world.reset()
        live["ok"] = True
        print(f"[autorace_sim] ■→▶ 재시작: {reset_sim()}", flush=True)
    t0 = time.monotonic()
    world.step(render=True)
    rclpy.spin_once(ros_node, timeout_sec=0.0)
    publish_cone_status()
    if park_prims and world.current_time - park_stat["t"] >= 1.0:
        park_stat["t"] = world.current_time
        park_pub.publish(String(data="limos=" + ";".join(f"{x:.3f},{y:.3f},{r:.1f}" for x, y, r in park_ref["now"])))
    if missions is not None and world.current_time - light_stat["t"] >= 0.2:
        light_stat["t"] = world.current_time
        light_pub.publish(String(data=missions.light_now(world.current_time - mission_t0["t"]) or "off"))
    if not lidar_z_logged["done"] and world.current_time > 1.0:
        lidar_z_logged["done"] = True
        from isaacsim.core.prims import SingleXFormPrim
        lz = SingleXFormPrim(lidar_path).get_world_pose()[0][2]
        print(f"[autorace_sim] 라이다 높이 (바닥에서): {lz:.3f} m", flush=True)
    if P["realtime"]:                       # 한 스텝(rendering_dt)이 벽시계보다 빨리 끝나면 기다린다
        time.sleep(max(0.0, 1.0 / P["render_hz"] - (time.monotonic() - t0)))
    if args.max_sim_seconds and world.current_time >= args.max_sim_seconds:
        break

ros_node.destroy_node()
rclpy.shutdown()
app.close()
