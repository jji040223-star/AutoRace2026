"""M4 주차 단독 실행 — /parking/start 없이 주차 구역 앞에서 바로 주차·탈출 (parking_controller standalone:=true).

  ros2 launch autorace_mission parking.launch.py                      # 시뮬: 차를 주차 구역 앞 (start_*) 으로 옮긴 뒤 시작
  ros2 launch autorace_mission parking.launch.py reset_sim:=false     # 시뮬: 지금 자리에서 시작
  ros2 launch autorace_mission parking.launch.py use_sim_time:=false  # 실차: 주차 구역 앞 2차로에 세워 두고

띄우는 것: parking_controller (/parking/cmd_vel), cmd_vel_mux (주차 중이면 /parking/cmd_vel → /cmd_vel, 라이다 범퍼),
lane_follower (enable false — 칸 끝선 거리 /lane/state 만). 시뮬 리셋 = reset_sim --pose (이번 리셋만 그 출발 자세, 응답으로 확인).
RViz: /parking/markers (빈 칸 초록, 목표 화살표, 차 사각형 파랑, 네 모서리 여유 cm).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODES = ['parking_controller', 'cmd_vel_mux', 'lane_follower', 'mission_manager']


def share(pkg, *p):
    return os.path.join(get_package_share_directory(pkg), *p)


def setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)   # noqa: E731
    sim = arg('use_sim_time').lower() == 'true'
    common = {'use_sim_time': sim}
    park = Node(package='autorace_mission', executable='parking_controller', name='parking_controller', output='screen',
                parameters=[share('autorace_mission', 'config', 'parking_controller.yaml'), dict(common, standalone=True)])
    mux = Node(package='autorace_mission', executable='cmd_vel_mux', name='cmd_vel_mux', output='screen',
               parameters=[share('autorace_mission', 'config', 'cmd_vel_mux.yaml'), common])
    lane = Node(package='autorace_lane', executable='lane_follower', name='lane_follower', output='screen',
                parameters=[share('autorace_lane', 'config', 'lane_follower.yaml'), dict(common, enable=False, cmd_vel_topic='/lane/cmd_vel')])
    check = Node(package='autorace_mission', executable='check_nodes', name='check_nodes', output='screen', arguments=NODES)
    x, y, yaw = float(arg('start_x')), float(arg('start_y')), float(arg('start_yaw_deg'))
    reset = Node(package='autorace_sim', executable='reset_sim', name='reset_sim', output='screen',
                 arguments=['--pose', str(x), str(y), str(yaw)])
    nodes = [LogInfo(msg='[parking] 주차 시작'), mux, lane, park]

    def after_check(event, _ctx):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='겹치는 노드가 있음'))]
        if sim and arg('reset_sim').lower() == 'true':
            return [LogInfo(msg=f'[parking] 출발 자세 ({x}, {y}, {yaw}°) 로 리셋'), reset]
        return nodes

    def after_reset(event, _ctx):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='리셋 실패'))]
        return nodes

    return [
        RegisterEventHandler(OnProcessExit(target_action=check, on_exit=after_check)),
        RegisterEventHandler(OnProcessExit(target_action=reset, on_exit=after_reset)),
        check,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('reset_sim', default_value='true', description='시뮬: 차를 start_* 자세로 옮긴 뒤 시작'),
        DeclareLaunchArgument('start_x', default_value='5.30', description='주차 구역 앞 2차로 (시뮬 world)'),
        DeclareLaunchArgument('start_y', default_value='4.84'),
        DeclareLaunchArgument('start_yaw_deg', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
