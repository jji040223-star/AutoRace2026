"""2D 시뮬 (Isaac 없이) — 트랙 + LIMO + 시설물, 카메라·뎁스·라이다·odom·imu·tf·/clock·/sim/reset.

  ros2 launch autorace_bringup sim2d.launch.py
  ros2 launch autorace_bringup sim2d.launch.py sim_params:=/path/real_like.yaml   # 설정 덮어쓰기
그다음 다른 터미널에서:
  ros2 launch autorace_mission drive.launch.py perception:=sim2d                    # 주행 (YOLO 대신 시뮬 검출)
  ros2 run autorace_sim sim2d_eval --until parking_done --timeout 400               # 판정 요약
YOLO 로 시험할 때 (drive.launch perception:=yolo): fake_detections:=false — 시뮬 검출과 겹치지 않게.
영상이 3 Hz 정도로만 오면 (UDP 조각 유실 — 이 PC 수신 버퍼가 작을 때) 모든 터미널에서:
  export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix autorace_bringup)/share/autorace_bringup/config/fastdds_shm.xml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    params = [os.path.join(get_package_share_directory('autorace_bringup'), 'config', 'params_sim2d.yaml')]
    extra = LaunchConfiguration('sim_params').perform(context)
    params += [p for p in extra.split(',') if p]     # 쉼표로 여러 개 (뒤가 앞을 덮음)
    fake = LaunchConfiguration('fake_detections').perform(context).lower() == 'true'
    return [Node(package='autorace_sim', executable='sim2d', name='sim2d', output='screen',
                 parameters=params + [{'fake_detections': fake}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim_params', default_value='', description='params_sim2d.yaml 을 덮어쓸 yaml'),
        DeclareLaunchArgument('fake_detections', default_value='true',
                              description='true = 시뮬이 표지판·신호등 박스를 냄 (drive perception:=sim2d). YOLO 로 시험하면 false'),
        OpaqueFunction(function=_nodes),
    ])
