"""2D 시뮬 (Isaac 없이) — 트랙 + LIMO + 시설물, 카메라·뎁스·라이다·odom·imu·tf·/clock·/sim/reset.

  ros2 launch autorace_bringup sim2d.launch.py
  ros2 launch autorace_bringup sim2d.launch.py sim_params:=/path/real_like.yaml   # 설정 덮어쓰기
그다음 다른 터미널에서:
  ros2 launch autorace_mission drive.launch.py perception:=sim2d                    # 주행 (YOLO 대신 시뮬 검출)
  ros2 run autorace_sim sim2d_eval --until parking_done --timeout 400               # 판정 요약
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
    if extra:
        params.append(extra)
    return [Node(package='autorace_sim', executable='sim2d', name='sim2d', output='screen', parameters=params)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim_params', default_value='', description='params_sim2d.yaml 을 덮어쓸 yaml'),
        OpaqueFunction(function=_nodes),
    ])
