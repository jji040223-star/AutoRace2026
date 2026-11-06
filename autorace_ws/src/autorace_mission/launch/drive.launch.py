"""주행 한 번에: YOLO + 뎁스 장애물 + 라이다 라바콘 + 신호등 판단 + 미션 매니저 + 속도 명령 선택기 + 차선 추종
+ M4 주차 (parking_controller — /parking/start 뒤 주차 구역에서 넘겨받음) (시뮬은 bringup 이 따로 띄워 둔다).

  ros2 launch autorace_mission drive.launch.py                        # 시뮬: 출발선으로 리셋한 뒤 출발
  ros2 launch autorace_mission drive.launch.py reset_sim:=false       # 시뮬: 지금 자리에서 이어서
  ros2 launch autorace_mission drive.launch.py use_sim_time:=false    # 실차 (리셋 없음)
  ros2 launch autorace_mission drive.launch.py enable:=false          # 인식·판단만, 바퀴는 안 돌림
  ros2 launch autorace_mission drive.launch.py light_source:=fake     # 신호등 박스를 YOLO 대신 시뮬이 아는 위치로 (decoy:=true 면 모니터에도)
  신호등 박스는 기본 YOLO (traffic_light_yolo = yolo_detector 에 models/traffic_light_yolov8n.pt — red/yellow/green_light,
  시뮬 영상으로 학습). 시뮬에서는 fake_traffic_light 도 띄워 진짜 신호등 박스를 /perception/traffic_light_truth 로 낸다 (시험 판정용).

순서:
  1. check_nodes   — 아래 노드가 이미 떠 있으면 시작하지 않는다 (다른 창에 남은 노드가 /cmd_vel 을 같이 냄)
  2. yolo_detector + depth_obstacle + cmd_vel_mux
     (lane_follower 는 /lane/cmd_vel 로 내고, cmd_vel_mux 가 미션 상태를 보고 lane / mission(/mission/cmd_vel) 중 하나를 /cmd_vel 로)
  3. YOLO 모델이 다 올라오면 (로그 "yolo_detector: … (device …") → reset_sim (시뮬) → 성공하면
  4. mission_manager + lane_follower — 출발. 출발하자마자 서행 표지판이 나오므로 YOLO 가 준비된 뒤에 출발해야 하고,
     mission_manager 는 리셋 뒤에 켠다 (리셋 전 자리 — 지난번 멈춘 곳 — 의 장애물·표지판을 보고 미션을 켜지 않게).
다시 달리기 = Ctrl+C → 같은 명령 (모든 노드가 처음부터, 차는 출발선으로). 로그는 이 터미널에 노드 이름을 붙여 같이 나온다.
노드 하나만 따로 띄우려면 각 패키지의 launch (yolo_detector / mission_manager / lane_follow) 를 쓴다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODES = ['lane_follower', 'yolo_detector', 'traffic_light_yolo', 'depth_obstacle', 'lidar_processor', 'traffic_light', 'fake_traffic_light',
         'parking_controller',
         'mission_manager', 'cmd_vel_mux', 'odom_imu']


def share(pkg, *p):
    return os.path.join(get_package_share_directory(pkg), *p)


def _actions(context):
    arg = lambda name: LaunchConfiguration(name).perform(context).lower() == 'true'   # noqa: E731
    sim = arg('use_sim_time')
    common = {'use_sim_time': sim}
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    ov = LaunchConfiguration('params').perform(context)
    OV = [p for p in ov.split(',') if p]        # 실차 보정값 등 — 모든 노드 기본 설정 위에 덮어씀 (노드 이름별 섹션)
    odom = Node(package='autorace_mission', executable='odom_imu', name='odom_imu', output='screen', parameters=[common])
    common = dict(common, odom_topic=odom_topic)   # 우리 노드는 휠 거리 + IMU 방향 오도메트리를 씀 (실차 휠 오도메트리는 회전을 23 % 적게 잼)
    if odom_topic == '/odom_imu':
        common['imu_topic'] = '/imu_unbiased'      # odom_imu 가 멈춰 있을 때 잰 자이로 치우침을 뺀 IMU (mission·parking 의 자이로 적분)
    yolo = Node(package='autorace_perception', executable='yolo_detector', name='yolo_detector', output='screen',
                parameters=[share('autorace_perception', 'config', 'yolo_detector.yaml')] + OV + [common])
    depth = Node(package='autorace_perception', executable='depth_obstacle', name='depth_obstacle', output='screen',
                 parameters=[share('autorace_perception', 'config', 'depth_obstacle.yaml')] + OV + [common])
    lidar = Node(package='autorace_perception', executable='lidar_processor', name='lidar_processor', output='screen',
                 parameters=[share('autorace_perception', 'config', 'lidar_processor.yaml')] + OV + [common])
    light = Node(package='autorace_perception', executable='traffic_light', name='traffic_light', output='screen',
                 parameters=[share('autorace_perception', 'config', 'traffic_light.yaml')] + OV + [common])
    src = LaunchConfiguration('light_source').perform(context).lower()
    light_yolo = Node(package='autorace_perception', executable='yolo_detector', name='traffic_light_yolo', output='screen',
                      parameters=[share('autorace_perception', 'config', 'traffic_light_yolo.yaml')] + OV + [
                                  dict(common, model_path=share('autorace_perception', 'models', 'traffic_light_yolov8n.pt'))])
    fake = Node(package='autorace_sim', executable='fake_traffic_light', name='fake_traffic_light', output='screen',
                parameters=[dict(common, decoy=arg('decoy'),
                                 detections_topic='/perception/traffic_light_detections' if src == 'fake' else '/perception/traffic_light_truth',
                                 start_x=float(LaunchConfiguration('start_x').perform(context)),
                                 start_y=float(LaunchConfiguration('start_y').perform(context)),
                                 start_yaw_deg=float(LaunchConfiguration('start_yaw_deg').perform(context)))])
    extra = LaunchConfiguration('mission_params').perform(context)
    mission = Node(package='autorace_mission', executable='mission_manager', name='mission_manager', output='screen',
                   parameters=[share('autorace_mission', 'config', 'mission_manager.yaml')] + OV + ([extra] if extra else []) + [common])
    park = Node(package='autorace_mission', executable='parking_controller', name='parking_controller', output='screen',
                parameters=[share('autorace_mission', 'config', 'parking_controller.yaml')] + OV + [common])
    mux = Node(package='autorace_mission', executable='cmd_vel_mux', name='cmd_vel_mux', output='screen',
               parameters=[share('autorace_mission', 'config', 'cmd_vel_mux.yaml')] + OV + [common])
    lane = Node(package='autorace_lane', executable='lane_follower', name='lane_follower', output='screen',
                parameters=[share('autorace_lane', 'config', 'lane_follower.yaml')] + OV + [
                            dict(common, enable=arg('enable'), cmd_vel_topic='/lane/cmd_vel', wait_mission=True)])
    reset = Node(package='autorace_sim', executable='reset_sim', name='reset_sim', output='screen')
    check = Node(package='autorace_mission', executable='check_nodes', name='check_nodes', output='screen', arguments=NODES)

    def stop(why):
        return [LogInfo(msg=f'[drive] {why} → 끝냅니다'), EmitEvent(event=Shutdown(reason=why))]

    sim2d = LaunchConfiguration('perception').perform(context).lower() == 'sim2d'

    def after_check(event, _ctx):
        if event.returncode != 0:
            return stop('겹치는 노드가 있음 (위 check_nodes 메시지)')
        if sim2d:
            # 2D 시뮬: 표지판·신호등 박스를 시뮬이 직접 낸다 (YOLO·fake_traffic_light 없음) → 바로 리셋·출발
            start = ([LogInfo(msg='[drive] 2D 시뮬 검출 → 출발선으로 리셋'), reset] if sim and arg('reset_sim')
                     else [LogInfo(msg='[drive] 2D 시뮬 검출 → 미션 매니저 + 차선 추종 출발'), mission, lane])
            return [odom, depth, lidar, light, mux, park] + start
        extra_nodes = ([light_yolo] if src == 'yolo' else []) + ([fake] if sim or src == 'fake' else [])
        return [LogInfo(msg='[drive] YOLO 모델 올리는 중 … (몇 초)'), odom, yolo, depth, lidar, light, mux, park] + extra_nodes

    fired = {'ready': False}

    def on_yolo_io(event):
        if fired['ready'] or b'(device' not in event.text:
            return None
        fired['ready'] = True
        if sim and arg('reset_sim'):
            return [LogInfo(msg='[drive] YOLO 준비됨 → 출발선으로 리셋'), reset]
        return [LogInfo(msg='[drive] YOLO 준비됨 → 미션 매니저 + 차선 추종 출발'), mission, lane]

    def after_reset(event, _ctx):
        if event.returncode != 0:
            return stop('리셋 실패 (위 reset_sim 메시지). 리셋 없이 지금 자리에서 달리려면 reset_sim:=false')
        return [LogInfo(msg='[drive] 리셋 완료 → 미션 매니저 + 차선 추종 출발'), mission, lane]

    def yolo_died(event, _ctx):
        if not fired['ready']:
            return stop('YOLO 가 준비 전에 끝남 (위 yolo_detector 에러 — ultralytics 설치·모델 경로)')
        return None

    return [
        RegisterEventHandler(OnProcessExit(target_action=check, on_exit=after_check)),
        RegisterEventHandler(OnProcessIO(target_action=yolo, on_stdout=on_yolo_io, on_stderr=on_yolo_io)),
        RegisterEventHandler(OnProcessExit(target_action=reset, on_exit=after_reset)),
        RegisterEventHandler(OnProcessExit(target_action=yolo, on_exit=yolo_died)),
        check,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='시뮬 true / 실차 false'),
        DeclareLaunchArgument('reset_sim', default_value='true', description='시뮬에서 출발 전에 LIMO·라바콘·미션 주기를 처음 상태로'),
        DeclareLaunchArgument('enable', default_value='true', description='false = 인식·판단만, /cmd_vel 0'),
        DeclareLaunchArgument('light_source', default_value='yolo', description='신호등 박스: yolo (traffic_light_yolo) | fake (시뮬 위치 투영)'),
        DeclareLaunchArgument('decoy', default_value='false', description='light_source:=fake 일 때 가짜 검출이 방해 모니터에도 박스를 냄'),
        DeclareLaunchArgument('start_x', default_value='0.74', description='시뮬 출발 자세 (params_sim robot.start_*) — 가짜 검출용'),
        DeclareLaunchArgument('start_y', default_value='5.58'),
        DeclareLaunchArgument('start_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('odom_topic', default_value='/odom_imu', description='우리 노드가 쓸 오도메트리 (/odom_imu = 휠 거리 + IMU 방향, /odom = 휠만)'),
        DeclareLaunchArgument('params', default_value='', description='모든 노드에 덮어쓸 yaml, 쉼표로 여러 개 (실차: autorace_bringup/config/real_limo.yaml)'),
        DeclareLaunchArgument('mission_params', default_value='', description='mission_manager 설정을 덮어쓸 yaml (시험용)'),
        DeclareLaunchArgument('perception', default_value='yolo',
                              description='yolo (YOLO 노드) | sim2d (2D 시뮬이 표지판·신호등 박스를 직접 냄, YOLO 없이)'),
        OpaqueFunction(function=_actions),
    ])
