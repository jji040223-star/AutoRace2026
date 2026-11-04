"""슬라이딩 윈도우 차선 추종.

  ros2 launch autorace_lane lane_follow.launch.py                       # 시뮬: LIMO 를 출발선으로 되돌린 뒤 시작
  ros2 launch autorace_lane lane_follow.launch.py reset_sim:=false      # 시뮬: 지금 자리에서 이어서
  ros2 launch autorace_lane lane_follow.launch.py use_sim_time:=false   # 실차 LIMO Pro (리셋 없음)
  ros2 launch autorace_lane lane_follow.launch.py enable:=false         # 인식만 (/cmd_vel 0)

차량 쪽(시뮬 또는 실차 limo_base·카메라)은 autorace_bringup bringup.launch.py 가 따로 띄운다.
시뮬에서 reset_sim 은 autorace_sim reset_sim (/sim/reset 서비스) 을 먼저 돌리고, 성공해야 lane_follower 를 띄운다.
리셋이 실패하면 (시뮬이 없거나 리셋 서비스가 없는 옛 시뮬) 차를 엉뚱한 자리에서 출발시키지 않도록 launch 를 끝낸다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _actions(context):
    arg = lambda name: LaunchConfiguration(name).perform(context).lower() == 'true'   # noqa: E731
    follower = Node(package='autorace_lane', executable='lane_follower', name='lane_follower', output='screen',
                    parameters=[LaunchConfiguration('params_file'),
                                {'use_sim_time': arg('use_sim_time'), 'enable': arg('enable')}])
    if not (arg('use_sim_time') and arg('reset_sim')):
        return [follower]
    reset = Node(package='autorace_sim', executable='reset_sim', name='reset_sim', output='screen')

    def after_reset(event, _context):
        if event.returncode == 0:
            return [follower]
        return [LogInfo(msg='[lane_follow] 리셋 실패 → 차선 추종을 띄우지 않고 끝냅니다 (위 reset_sim 메시지 참고). '
                            '리셋 없이 지금 자리에서 달리려면 reset_sim:=false'),
                EmitEvent(event=Shutdown(reason='reset_sim 실패'))]

    return [reset, RegisterEventHandler(OnProcessExit(target_action=reset, on_exit=after_reset))]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('autorace_lane'), 'config', 'lane_follower.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='시뮬 true / 실차 false'),
        DeclareLaunchArgument('enable', default_value='true', description='false = 인식만, /cmd_vel 0'),
        DeclareLaunchArgument('reset_sim', default_value='true',
                              description='시뮬에서 시작 전에 LIMO·라바콘·미션 주기를 처음 상태로 (/sim/reset)'),
        OpaqueFunction(function=_actions),
    ])
