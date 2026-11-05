"""AutoRace LIMO 실행 진입점.

  ros2 launch autorace_bringup bringup.launch.py mode:=sim  [headless:=true] [params_file:=...]
  ros2 launch autorace_bringup bringup.launch.py mode:=real [params_file:=...]

두 모드 모두 같은 토픽/프레임을 낸다: /cmd_vel, /odom, /imu, /scan, /camera/{color,depth}/..., /tf, /tf_static
  sim : Isaac Sim(run_sim.py) + depth_converter + 센서 정적 TF (+ 카메라 optical 프레임)
  real: limo_base + ydlidar + Orbbec 카메라 드라이버(설치돼 있으면) + 센서 정적 TF
"""
import math
import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)   # camera_link(x 전방) → *_optical_frame(z 전방, y 아래)


def share(pkg):
    return get_package_share_directory(pkg)


def has_package(pkg):
    try:
        share(pkg)
        return True
    except PackageNotFoundError:
        return False


def static_tf(parent, child, xyz, rpy, use_sim_time):
    args = []
    for k, v in zip(("--x", "--y", "--z", "--roll", "--pitch", "--yaw"), (*xyz, *rpy)):
        args += [k, str(v)]
    return Node(package="tf2_ros", executable="static_transform_publisher", name=f"static_tf_{child}",
                arguments=args + ["--frame-id", parent, "--child-frame-id", child],
                parameters=[{"use_sim_time": use_sim_time}], output="log")


def sim_actions(doc, params, headless):
    P = doc["isaac_sim"]["ros__parameters"]
    F = P["frames"]
    track_usd = os.path.expanduser(P["track"]["usd"]) if P["track"]["usd"] else os.path.join(
        share("autorace_track"), "usd", "autorace_track.usda")
    isaac_dir = share("autorace_sim") + "/isaac"
    cmd = [f"{isaac_dir}/run_isaac.sh", f"{isaac_dir}/run_sim.py", "--params", params, "--track-usd", track_usd]
    if headless:
        cmd.append("--headless")
    env = {"ISAACSIM_DIR": os.path.expanduser(P["isaacsim_dir"])}
    return [
        static_tf(F["camera"], F["color_optical"], (0, 0, 0), OPTICAL_RPY, True),
        static_tf(F["camera"], F["depth_optical"], (0, 0, 0), OPTICAL_RPY, True),
        ExecuteProcess(cmd=cmd, additional_env=env, output="screen", name="isaac_sim"),
        Node(package="autorace_sim", executable="depth_converter", name="depth_converter",
             parameters=[params, {"use_sim_time": True}], output="screen"),
    ]


def real_actions(doc, params):
    D = doc["real_drivers"]["ros__parameters"]
    out = []

    def need(pkg, what):
        if has_package(pkg):
            return True
        out.append(LogInfo(msg=f"[autorace] 경고: '{pkg}' 패키지가 없어 {what} 를 건너뜀"))
        return False

    if D["limo_base"] and need("limo_base", "limo_base (/cmd_vel, /odom, /imu)"):
        out.append(Node(package="limo_base", executable="limo_base", name="limo_base_node",
                        parameters=[params], output="screen", emulate_tty=True))
    if D["lidar"] and need("ydlidar_ros2_driver", "라이다 (/scan)"):
        out.append(Node(package="ydlidar_ros2_driver", executable="ydlidar_ros2_driver_node",
                        name="ydlidar_ros2_driver_node", parameters=[params], output="screen"))
    if D["camera"] and need(D["camera_launch_package"], "카메라 (/camera/*)"):
        out.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share(D["camera_launch_package"]), "launch", D["camera_launch_file"]))))
    return out


def setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("sim", "real"):
        raise RuntimeError(f"mode 는 sim 또는 real (받은 값: {mode})")
    params = LaunchConfiguration("params_file").perform(context) or os.path.join(
        share("autorace_bringup"), "config", f"params_{mode}.yaml")
    with open(params) as f:
        doc = yaml.safe_load(f)
    sim = mode == "sim"
    base = (doc["isaac_sim"]["ros__parameters"]["frames"]["base"] if sim
            else doc["limo_base_node"]["ros__parameters"]["base_frame"])

    actions = [LogInfo(msg=f"[autorace] mode={mode} params={params}")]
    for child, m in doc["sensor_mounts"]["ros__parameters"].items():
        if m.get("publish_tf", True):
            actions.append(static_tf(base, child, m["xyz"], m["rpy"], sim))
    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true", "yes")
    actions += sim_actions(doc, params, headless) if sim else real_actions(doc, params)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="sim", description="sim | real"),
        DeclareLaunchArgument("params_file", default_value="",
                              description="비우면 autorace_bringup/config/params_<mode>.yaml"),
        DeclareLaunchArgument("headless", default_value="false", description="sim 전용: Isaac 창 없이 실행"),
        OpaqueFunction(function=setup),
    ])
