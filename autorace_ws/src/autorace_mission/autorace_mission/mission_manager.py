"""미션 매니저 (v3 문서의 노드 1) — 주행 순서대로 M5 서행 → M6 서행 유도선(지그재그) → M3 보행자 정지 → M7 3차선 장애물 회피
→ M2 갈림길 → M8 라바콘 → M1 신호등.

  sub  detections_topic (기본 /perception/detections, autorace_msgs/DetectionArray) — YOLO
  sub  lane_state_topic (기본 /lane/state, autorace_msgs/LaneState)                — 선 피팅 잔차 (지그재그), 정지선 거리
  sub  obstacles_topic  (기본 /perception/obstacles, autorace_msgs/ObstacleArray)   — 뎁스 장애물 (M7)
  sub  odom_topic       (기본 /odom, nav_msgs/Odometry)                           — 거리 (직진 거리는 휠 오도메트리를 믿는다), 정지 확인
  pub  state_topic      (기본 /mission/state, std_msgs/String)                    — 지금 상태 이름, rate_hz 로 계속
  pub  lane_cmd_topic   (기본 /mission/lane_cmd, autorace_msgs/LaneCmd)           — 속도 상한·정지·윈도우 리셋, rate_hz 로 계속
  pub  cmd_vel_topic    (기본 /mission/cmd_vel, geometry_msgs/Twist)              — M7 동안 직접 운전 (cmd_vel_mux 가 골라 차로)

FSM (v3 6장 "보이면 켜지고, 조건으로 끝난다"). 우선순위: 정지 미션(M3) > 서행(M5·M6) > LANE.
  LANE / M5 / M6 → M3_APPROACH : 보행자 표지판 (YOLO crosswalk) m3_confirm_frames 장 연속 (쿨다운 중이면 무시)
  LANE           → M5_SLOW     : 서행·방지턱 표지판 m5_confirm_frames 장 연속
  LANE           → M6_SLOW     : 선 피팅 잔차 ≥ m6_resid_on 이 m6_confirm_frames 장 연속 (톱니 모양 지그재그 선)
  M5_SLOW → LANE : 표지판을 마지막으로 본 곳에서 m5_release_m
  M6_SLOW → LANE : 잔차가 m6_resid_off 아래로 내려간 뒤 m6_release_m (지그재그 선이 BEV 에서 사라짐)
  M3_APPROACH    : 정지선이 보이면 (BEV 0.37–0.75 m) 앞바퀴까지 남은 거리를 재고, 화면 아래로 사라진 뒤는 오도메트리로 이어서.
                   처음 잡은 선과 m3_line_match_m 넘게 어긋나는 측정은 다른 선 (횡단보도 뒤 가로선 등) 이라 버린다.
                   속도 = m3_kp × 남은 거리 (m3_min_speed ~ m3_approach_speed). 남은 거리 ≤ 0 → M3_STOP.
                   표지판 뒤 m3_search_m 안에 정지선을 못 찾으면 포기 → LANE
  M3_STOP        : lane_cmd.stop. 속도가 0 이 된 순간부터 m3_wait_s (규정 3 s 이상, 15 s 안 발진) → M3_GO
  M3_GO          : m3_exit_m 지나면 LANE. 이후 m3_cooldown_m 동안 보행자 표지판 무시 (같은 표지판으로 다시 켜지지 않게)
  LANE / M6 → M7_AVOID : 내 차로 (옆 ±m7_trigger_half_m) 앞 m7_trigger_m 안에 폭 m7_min_width_m~m7_max_width_m 장애물 m7_confirm_frames 장 연속,
                   그리고 노란 선이 양쪽 다 보이는 1차로 도로가 아닐 때 (3차선에서는 노란 선이 한쪽뿐이거나 없다)
  M7_AVOID       : 차로 기준 좌표 (진행 방향 s, 왼쪽 l) — 방향은 최근 (차 yaw + 차선 방향) 중앙값을 m7_axis_snap_deg 단위로 맞춘 것
                   (이 트랙 직선은 출발 방향과 평행·직각. 3차선 입구는 노란 경계선이 비스듬히 벌어져 차선 방향만으로는 30° 까지 틀린다),
                   차로 중심은 기억한 정차 LIMO 들의 옆 위치 (차로 가운데에 선다) 로 맞춘 0.52 m 격자 (옆 LIMO 는 먼 옆면을
                   m7_obstacle_width_m 로 채움). 맨 왼쪽·오른쪽 차로를 곧게 갈 때 노란 바깥선이 보이면 그 선으로 격자를 보정 (m7_line_fix).
                   차로 번호 (0 = 맨 왼쪽) 는
                   LIMO 가 세 차로에 다 있으면 그것으로, 아니면 노란 선 쪽으로 정하고 목표 차로를 ±1 로 바꾼다 (v3).
                   장애물은 오도메트리 좌표로 기억해 두어 옆을 지날 때 (뎁스 화면 밖) 도 안다. 내 차로 앞 m7_block_m 안이 막히면
                   옆 차로 중 앞이 가장 먼 곳으로 — 단, 그 차로의 [차 중심, 차 앞 + m7_enter_clear_m] 이 비어 있을 때만
                   (= 그 차로 장애물을 차 중심이 지나친 뒤. 0.9 m 간격 줄에서 '완전히 지나친 뒤' 로 하면 다음 줄 앞에서 막힌다).
                   조향 = 목표 차로 중심을 m7_lookahead_m 앞에서 보는 pure pursuit, 앞이 가까우면 감속·정지. /mission/cmd_vel 로 운전.
                   내 차로 앞이 m7_clear_m 비고 차로 변경이 끝나면 → LANE (lane_cmd.reset_window 로 슬라이딩 윈도우 새로 시작)
  M8_AVOID       : M2 뒤, 앞 m8_trigger_m 안 양쪽에 라바콘 (lidar_processor /perception/cones) m8_trigger_count 개 이상이 m8_confirm_frames 번.
                   Follow-the-Gap: 통로를 가로지르는 두 콘의 틈 중 실제 빈 거리 (중심 거리 − 라이다 높이 몸통 지름) ≥ m8_gap_min_m 인
                   것의 가운데점을 이은 길에서 앞 m8_aim_m 점으로 (그런 틈이 없으면 방향 순으로 이웃한 콘 사이 틈의 가운데로),
                   방향은 ±m8_max_heading_deg 안. 쓸기 검사 (avoid, M7 과 같은 함수) 로 라바콘 상자 (바닥판) 와 여유가 m8_slow_clear_m 이하면
                   m8_slow_speed. 통로 안 제자리 회전은 ±m8_rot_limit_deg. 앞으로 가라는데 1 s 안 움직이면 m8_back_m 후진 후 다시 계산
                   (재시도 m8_retry_max 번, 넘으면 조금이라도 앞으로 가는 쪽). 끝 = 앞 m8_end_clear_m 안에 라바콘이 없고 옆·뒤 라바콘도
                   m8_end_pass_m 지남 → LANE (윈도우 리셋, m8_exit_slow_s 동안 천천히), 이후 라바콘 무시.
  M1_APPROACH    : M8 뒤 (m1_require_m8) /perception/traffic_light 에 색이 m1_confirm_frames 번 → 정지선 쪽으로 (M3 와 같은 정지선 코드:
                   처음 잡은 선만, 화면 밖으로 사라지면 오도메트리로, 카메라→앞바퀴 보정, 남은 거리 비례 감속). 크기까지 확인한 초록이
                   m1_green_frames 번 연속이면 서지 않고 통과 (M1_GO). 아니면 앞바퀴가 선에 닿을 때 정지 → M1_STOP.
  M1_STOP        : 정지 유지 (빨강·노랑·안 보임·애매 모두). 크기까지 확인한 초록 m1_green_frames 번 연속 → M1_GO. 안 보이거나 애매하면
                   m1_log_s 마다 로그. 이 동안 진행 감시는 없음 (차선 추종에 stop 만).
  M1_GO          : m1_go_speed 로 서행. 앞바퀴가 선에 닿기 전에 크기 확인한 노랑·빨강 m1_stop_frames 번 (초록 없이) → M1_APPROACH (다시 정지선에 섬, 노랑도 정지).
                   정지선 m1_exit_m 지나면 M4_PARKING + /parking/start (std_msgs/Empty).
  M4_PARKING     : 차선 추종으로 가고, parking_controller 가 주차 구역 (앞옆 LIMO) 에서 넘겨받아 주차·탈출 (cmd_vel_mux 가 /parking/state 를
                   보고 고름). /parking/done → DONE (멈춤).
  순서 힌트 (v3 6장 "가벼운 순서 힌트") : M2 를 지난 뒤에는 M5·M6·M3·M7 을 켜지 않는다 (라바콘 구간에서 잔차·보행자 표지판·M7 오인식).
  M2 갈림길 (상태 없음, M7 안에서) : 3차선 끝 분리대 앞 표지판 (YOLO turn_a = A 왼쪽 길, turn_b = B 오른쪽 길) 이 m2_confirm_frames 장
                   연속이면 결과를 기억한다. 표지판은 달리는 중에 바뀔 수 있으므로 (규정집) 다른 결과가 다시 확인되면 마지막 것을 따른다.
                   M7 은 이 결과로 (목표 차로 A = 맨 왼쪽, B = 맨 오른쪽) ① 내 차로가 막혀 옆으로 갈 때 목표 쪽 옆 차로를 먼저 —
                   그 차로가 지금은 막혀 있어도 내 차로에서 서기 전에 비면 반대쪽으로 빠지지 않고 기다린다, ② 목표 반대쪽에 있으면
                   목표 쪽 옆 차로가 지금 차로만큼 비어 있을 때 미리 옮긴다 (앞에 장애물이 하나도 없으면 = 분리대 옆이면 안 옮김),
                   ③ 옆 차로 둘이 같을 때 고르는 쪽 (m7_prefer 대신). 분리대 옆에서 M7 이 끝나면 차선 추종이 그 길을 따라간다.
                   M7 이 끝날 때 결과를 확정하고, 다음 리셋까지 표지판을 더 보지 않는다.
"""
import bisect
import collections
import math
import signal

import numpy as np
import rclpy
from autorace_msgs.msg import DetectionArray, LaneCmd, LaneState, ObstacleArray, TrafficLight
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty, String

from autorace_mission import avoid
from autorace_mission.avoid import RetryCounter, StuckMonitor
from geometry_msgs.msg import PoseArray

SLOW_STATES = ('M5_SLOW', 'M6_SLOW')
CAR_HALF_L, CAR_HALF_W = avoid.CAR_HALF_L, avoid.CAR_HALF_W     # LIMO 반 길이·반 폭 [m] (base_link 중심)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        p = self.declare_parameter
        p('detections_topic', '/perception/detections')
        p('lane_state_topic', '/lane/state')
        p('odom_topic', '/odom')
        p('state_topic', '/mission/state')
        p('lane_cmd_topic', '/mission/lane_cmd')
        p('obstacles_topic', '/perception/obstacles')
        p('cmd_vel_topic', '/mission/cmd_vel')
        p('imu_topic', '/imu')             # M7 차로 방향 정렬·방향 (자이로 적분)
        p('scan_topic', '/scan')           # M7 후진 때 뒤쪽 확인
        p('laser_x', 0.103)                # base_link → 라이다 [m] (sensor_mounts laser_link)
        p('rate_hz', 20.0)
        # M5 서행 (방지턱)
        p('m5_labels', ['slow', 'speed_bump'])
        p('m5_min_conf', 0.5)
        p('m5_min_box_px', 20)
        p('sign_window_frames', 5)         # 표지판 (M3·M4·M5) 확인: 최근 이만큼 중 *_confirm_frames 장 잡히면 인정 (실차 run5: 띄엄띄엄 잡혀 '연속 3 장' 을 못 채워 서행 놓침, 11-06)
        p('m5_max_box_px', 200)            # 이보다 큰 박스는 오인식 (실차 3차선에서 화면 가득 478px 과속방지턱 → 괜히 서행, 11-05)
        p('m5_confirm_frames', 3)
        p('m5_speed', 0.15)
        p('m5_release_m', 1.5)
        # M6 서행 유도선 (지그재그)
        p('m6_resid_on', 7.5)              # [px] 시뮬: 지그재그 8.0–8.7, 직선 2.2–2.4, 코너 ≤ 6.9
        p('m6_resid_off', 5.0)
        p('m6_confirm_frames', 5)
        p('m6_speed', 0.15)
        p('m6_release_m', 0.3)
        # M3 보행자 표지판 정지
        p('m4_require_sign', True)         # 주차 표지판 (P) 을 본 뒤에만 주차 모드 (순서 조건만으로 켜면 주차 구역이 아닌 곳에서
                                           #   트랙 옆 물체를 주차 LIMO 로 보고 차선 추종을 놓았다 — 실차 11-05)
        p('m4_labels', ['parking'])
        p('m4_min_conf', 0.5)
        p('m4_min_box_px', 20)
        p('m4_confirm_frames', 3)
        p('m3_labels', ['crosswalk'])      # 모델 클래스 이름 (보행자 표지판 = crosswalk)
        p('m3_min_conf', 0.5)
        p('m3_min_box_px', 30)
        p('m3_confirm_frames', 3)
        p('m3_approach_speed', 0.15)
        p('m3_min_speed', 0.04)
        p('m3_kp', 0.5)                    # 속도 [m/s] = kp × 남은 거리 [m]
        p('m3_cam_to_front_axle_m', 0.016) # 카메라 → 앞바퀴 축 (앞이 +). sim: 카메라 x 0.084, 앞축 x 0.1
        p('m3_wheel_front_m', 0.045)       # 앞바퀴 축 → 바퀴 맨 앞 (바퀴 반지름). 실차 실측
        p('m3_line_margin_m', 0.025)       # 바퀴 맨 앞을 정지선 가까운 끝에서 이만큼 앞에 세움 — 정지선을 밟으면 안 됨
        p('m3_stop_gap_m', 0.0)            # 정지 명령 뒤 미끄러지는 거리 보정 (이만큼 일찍 정지 명령)
        p('m3_line_match_m', 0.10)         # 잡은 정지선과 이만큼 넘게 다른 측정 = 다른 선 (횡단보도 앞·뒤 가로선 간격 0.54 m)
        p('m3_search_m', 2.0)
        p('m3_wait_s', 4.0)
        p('m3_still_mps', 0.01)            # 이보다 느리면 "완전 정지"
        p('m3_exit_m', 0.5)
        p('m3_cooldown_m', 3.0)
        p('m3_max_box_px', 200)            # 이보다 큰 박스는 보행자 표지판 아님 (라바콘 구간에서 화면 가득 오검출 480 px)
        # M7 3차선 정적 장애물 회피
        p('m7_trigger_m', 1.2)             # 내 차로 앞 이 거리 안 장애물이면 켜짐 (뎁스 화각 ±35° — 옆 차로 장애물을 놓치지 않게 일찍)
        p('m5_after_m3_ignore', True)      # 횡단보도 (M3) 를 마친 뒤엔 서행 표지판 무시
        p('m7_trigger_overlap', True)      # 내 차로 판단: 장애물이 ±m7_trigger_half_m 에 걸치면 (false = 가운데가 그 안일 때만)
        p('m7_trigger_half_m', 0.2)        # 내 차로 = 차 중심 옆 ±이만큼
        p('m7_min_width_m', 0.12)          # 정차 LIMO 폭 0.22. 라바콘(~0.09)·표지판 기둥으로는 안 켜지게
        p('m7_max_width_m', 0.45)          # 이보다 넓은 뎁스 장애물은 버림 — 방지턱에서 차가 기울면 바닥이 폭 2 m 장애물로 보였다 (11-02)
        p('m7_min_height_m', 0.10)         # 정차 LIMO 높이 0.25. 방지턱을 넘으며 차가 기울 때 바닥이 장애물로 보이는 것 (시뮬에서 겪음) 거르기
        p('m7_max_height_m', 0.45)         # 이보다 높은 뎁스 장애물은 버림 (기억·켜짐 둘 다) — 정지선 뒤 코너에서 트랙 밖 유리벽 (뎁스 상한 0.6) 을
                                           # 정차 LIMO 로 기억해 차로 격자가 한 칸 밀림 → 빈 오른쪽 차로를 없는 차로로 봄 (실차 11-06 full_m1m7)
        p('m7_confirm_frames', 3)
        p('m7_lane_width_m', 0.52)         # 차로 중심 간격 (선 안쪽 500 + 선 20 mm)
        p('m7_lanes', 3)
        p('m7_entry_lane', 1)              # M7 에 들어갈 때 차로 (0 = 맨 왼쪽). 입구는 한 차로가 가운데 차로로 이어지는 구조 → 차 위치로
                                           # 격자를 바로 고정. −1 = 예전처럼 정차 LIMO 위치로 추측 (실차 11-06: 트랙 밖 유리벽을 LIMO 로 세어
                                           # 격자가 한 칸 밀림 → 빈 오른쪽 차로를 없는 차로로 봄, 3 번 중 2 번)
        p('m7_entry_center_fix', False)    # 진입 직전 1차로 차선 가운데 (lane_follower) 로 격자 가운데 맞춤
        p('m7_entry_center_back_m', 1.2)   #   진입점 뒤 이 거리 안에서 잰 점만
        p('m7_entry_center_max_m', 0.15)   #   이보다 크게 벗어났다고 나오면 안 씀 (차선 오인식)
        p('m7_outer_wall', True)           # 세 차로 바깥을 쓸기 검사 벽으로 (노란 선 밖으로 안 나가게)
        p('m7_outer_wall_off_m', 0.0)      #   벽 위치 = 바깥 차로 끝선 + 이만큼 (+ 바깥)
        p('m7_drop_outside', True)         # 세 차로 밖에 중심이 있는 장애물 (기억·라이다) 은 무시 — 트랙 밖 벽·물건
        p('m7_lane_margin_m', 0.05)        # 장애물이 차로에 걸친다고 볼 최소 겹침
        p('m7_obstacle_len_m', 0.33)
        p('m7_obstacle_width_m', 0.22)     # 옆에 있는 장애물은 차에서 먼 옆면이 덜 보여 폭이 좁게·중심이 차 쪽으로 잡힌다 (시뮬 rosbag 4 판:
                                           # 비스듬히 본 LIMO 중심 3–4.5 cm, 앞에서 본 것 1.5–2 cm) → 가까운 옆면 + 이 폭 (0 = 끔)
        p('m7_side_min_m', 0.10)           # 장애물 가운데가 차 옆으로 이만큼 넘게 떨어져 있을 때만 (바로 앞은 양 옆면이 다 보임)
        p('m7_oblique_deg', 25.0)          # 이보다 비스듬히 (또는 범퍼 15 cm 안) 본 관측은, 앞에서 본 관측이 3 개 넘게 있으면 상자에 안 넣음
        p('m7_side_no_len', True)          # 옆에서 본·화각 가장자리에 걸린 관측은 가정 길이를 안 붙임 (false = 11-02 까지의 동작)
        p('m7_entry_walls', True)          # 3차선 입구 흰 가로선까지는 1차로 양쪽 가상 벽 (대각 금지)
        p('m7_entry_single_m', 0.6)        #   가로선을 못 보면 M7 시작 뒤 이만큼까지를 1차로로 (시뮬: 켜진 자리 → 가로선 0.4–0.6 m)
        p('m7_entry_funnel_m', 0.55)       #   가로선 뒤 깔때기 길이 — 이 동안 벽이 45° 로 벌어짐 (트랙: 0.55 m 에 한 차로 넓어짐)
        p('m7_cam_x', 0.084)               # base_link → 카메라 [m] (depth_obstacle cam_x 와 같게)
        p('m7_cam_hfov_deg', 71.0)         # 뎁스 가로 화각 (Orbbec Dabai 컬러 기준 — 실차는 뎁스 camera_info 로 확인)
        p('m7_inside_margin_m', 0.05)      # 보인 점 가운데가 기억 상자 + 이 여유 안이면 같은 장애물 (조각 관측)
        p('m7_front_match_m', 0.08)        # 그때 보인 가까운 면이 상자 앞면에서 이보다 안쪽이면 앞면 관측이 아님 (가려진 뒷부분) → 상자에 안 넣음
        p('m7_cam_yaw_deg', 0.0)           # 카메라 좌우 각도 (depth_obstacle cam_yaw_deg 와 같게, lane_calib 으로 잼)
        p('m7_fov_edge_deg', 4.0)          # 관측의 가까운 끝이 화각 경계에서 이 안이면 '잘린 관측'
        p('m7_merge_m', 0.3)               # 관측 중심이 이 안이면 같은 장애물 (각도마다 보이는 면이 달라 0.2 m 로는 한 LIMO 가 둘로 기억됐다.
                                           # 정차 LIMO 끼리는 0.52 m 이상 떨어져 있다)       # 뎁스는 앞면만 보므로 장애물 길이를 이만큼으로 가정 (LIMO 0.322)
        p('m7_block_m', 1.2)               # 내 차로 앞이 이 거리 안에서 막히면 갈 차로를 정한다 (실제로 꺾는 때는 쓸기 검사가 정함)
        p('m7_enter_window_m', 0.30)       # 옆 차로 후보: 막은 장애물 앞면 이만큼 앞 ~ 뒷면 + 0.15 m 가 그 차로에서 비어 있어야 (차 길이쯤 — 꺾는 때는 쓸기 검사가 정함)
        p('m7_max_yaw_rate', 0.25)         # [rad/s] 이보다 빨리 돌고 있으면 (코너) 켜지 않음 — 차로 방향을 잘못 잡는다
        # 구동 방식: diff (네 바퀴 차동 — 제자리 회전) | ackermann (앞바퀴 조향 — 제자리 회전 없음, 최소 회전 반경)
        p('drive_mode', 'diff')
        p('ack_min_radius_m', 0.47)        # 아커만 최소 회전 반경 (차 중심) — limo_base 가 이보다 작은 반경은 잘라냄
        p('ack_plan_radius_m', 0.52)       # 경로 계획에 쓰는 최소 반경 (조향 지연·오차 여유)
        p('m7_ack_d1_m', 0.25)             # 아커만 M7: 첫 원호 길이 (매 순간 다시 계획 — 이 원호의 곡률로 운전)
        p('m7_ack_d2_m', 0.70)             #   둘째 원호 길이 (S 자 뒤쪽 — 먼저 꺾어도 빠져나올 수 있는지)
        p('m8_ack_arcs', False)            # 아커만 M8: 원호 후보로 고르기 (2D 시뮬 11-09 j: 매 순간 0.35 m 만 봐서 휜 통로 앞에서 막혀
                                           # 후진 8~17 번 → 끔, 예전 틈 가운데 추종 + 조향 후진·출구 추종만)
        p('m8_ack_arc_m', 0.35)            # 아커만 M8: 원호 검사 길이
        p('m7_ack_ref', True)              # 아커만 M7: 차로 바꿀 때 기준 S 자 (가장 이른 안 닿는 시작) 를 목표로
        p('m7_ack_ref_max_a_m', 1.2)       #   꺾기 시작을 이만큼 뒤까지 찾음
        p('m7_ack_ref_cross_funnel', True) #   깔때기 벽을 지켜선 S 자가 안 들어가면 깔때기 벽만 빼고 다시 찾음
        p('m7_ack_all_latch_m', 1.0)       #   ③ (깔때기 선 넘기) 로 정하면 이만큼 갈 때까지 유지
        p('m7_ack_w_out', 2.0)             #   ③ 중 두 바퀴가 선 밖인 길이 벌점 [1/m]
        p('m7_ack_w_ref_head', 0.4)        #   기준 방향과의 차 벌점 [1/rad]
        p('m7_ack_n', 11)                  #   곡률 후보 수 (−1/R ~ +1/R)
        p('m7_ack_w_lat', 2.0)             #   비용: 끝 자리 옆 오차 [1/m]
        p('m7_ack_w_head', 0.15)           #   비용: 끝 방향 오차 [1/rad] (옆 오차는 펴는 거리까지 투영해서 봄 → 작게)
        p('m7_ack_w_free', 3.0)            #   비용: 둘째 원호에서 막혀 못 가는 길이 [1/m]
        p('m7_ack_max_psi_deg', 70.0)      #   차로 방향과 이보다 틀어지는 경로는 안 씀
        p('m7_speed', 0.15)
        p('m7_change_speed', 0.10)         # 차로 바꾸는 동안 — 앞으로 덜 가면서 옆으로 빠지게
        p('m7_kv', 1.0)                    # 앞 장애물까지 거리 비례 감속 [1/s]
        p('m7_stop_gap_m', 0.07)           # 쓸기 검사 거리 (차 + m7_margin_m) 가 이보다 짧으면 그 방향으로 안 감
        p('m7_avoid_far_side', True)      # M2: 표지판 반대쪽 끝 차로 (A 면 2, B 면 0) 는 다른 후보가 없을 때만
        p('m7_final_to_goal', True)       # M2: 장애물 다 지난 뒤 목표 차로가 한 칸 옆이면 옮기고 끝냄
        p('m7_failed_keep_m', 0.6)        # 같은 자리 실패 뒤 반대쪽 재시도: 실패한 곳 + 이 거리까지 그 차로를 다시 고르지 않음
        p('m7_lidar_trim', True)           # 라이다 가까운 기억 상자는 차를 향한 면을 라이다 점까지 줄임 (실차 11-06)
        p('m7_lidar_trim_reach_m', 1.2)    #   라이다에서 상자 가까운 면까지 이 안
        p('m7_lidar_trim_pad_m', 0.03)     #   라이다 점 바깥 여유
        p('m7_lidar_trim_min_m', 0.10)     #   줄여도 이 두께는 남김
        p('m7_lidar_clear', True)         # 뎁스로 기억한 상자 자리를 라이다 빔이 뚫고 지나가면 (가짜) 지움
        p('m7_lidar_clear_scans', 4)       #   이만큼 연속 (라이다 8~10 Hz → 약 0.5 s)
        p('m7_lidar_clear_reach_m', 1.2)   #   라이다에서 상자 가까운 면까지 이 안일 때만
        p('m7_lidar_clear_shrink_m', 0.03) #   상자를 사방 이만큼 줄여서 봄 (뎁스 상자는 실제보다 큼)
        p('m7_lidar_clear_max_w', 0.6)
        p('m7_lidar_z', 0.111)             #   바닥 → 라이다 빔 높이 [m] (LIMO Pro)
        p('m7_lidar_clear_below_m', 0.03)  #   뎁스로 본 가장 낮은 점이 라이다 높이 − 이것보다 위면 (떠 있음) 안 지움     #   [rad/s] 이보다 빨리 돌 때는 안 봄 (스캔이 휨)
        p('m7_dedup_overlap', True)       # M7 중 차로 좌표 상자가 겹치는 기억 둘은 하나로 (관측 많은 쪽 남김)
        p('m7_margin_m', 0.03)             # 쓸기 검사: 차 사각형을 사방으로 이만큼 키움 (기억 상자 오차)
        p('m7_rot_margin_m', 0.015)        # 제자리 회전 검사 여유
        p('m7_check_m', 1.0)               # 쓸기 검사 길이
        p('m7_go_free_m', 0.15)            # 이 거리 이상 갈 수 있는 방향만 '갈 수 있음'
        p('m7_skip_overlap_m', 0.15)       # 장애물이 옆 차로를 이만큼 이하로 걸치면 차로를 고를 때 그 차로를 막은 것으로 안 침 (실차 11-06)
        p('m7_reach_slack_m', 0.08)        # 차로 변경 대각은 목표 차로 가운데 − 이만큼까지 비어 있어야 꺾음 (실차 11-06)
        p('m7_heading_step_deg', 10.0)     # 차로 바꿀 때 방향 후보 간격 (0 ~ m7_max_heading_deg, 목표 차로 쪽)
        p('m7_lookahead_m', 0.30)
        p('m7_k_heading', 1.8)             # ω = k × 방향 오차 − kd × 지금 회전 속도 (3.0 이면 시뮬에서 ±30° 로 흔들렸다)
        p('m7_kd_heading', 0.3)
        p('m7_max_w', 1.5)
        p('m7_pp_heading_deg', 45.0)       # pure pursuit 방향 상한 (가장 바라는 방향)
        p('m7_max_heading_deg', 60.0)      # 방향 후보 상한
        p('m7_turn_first_deg', 25.0)       # 고른 방향과 지금 방향이 이보다 다르면 제자리에서 먼저 돈다 (v = 0)
        p('m7_lane_done_m', 0.10)          # 목표 차로 중심과 이만큼 안이면 변경 끝 (여유)
        p('m7_clear_m', 1.0)               # 내 차로 앞이 이만큼 비면 끝
        p('m7_min_exit_m', 3.0)            # 3차선 구간을 이만큼 가기 전엔 '앞이 비었다' 로 끝내지 않음 (뒤 줄이 m7_clear_m 너머라 안 셈 → 끝내고 차선 추종하다 부딪힘)
        p('m7_prefer', 'left')             # 옆 차로 둘이 같으면 이쪽 (M2 갈림길 표지판을 봤으면 그 결과로)
        p('m7_max_m', 6.0)
        p('m7_stuck_check_s', 1.0)         # 명령은 움직이라는데 (v > 0.03 또는 |w| > 0.3) 이 시간 동안
        p('m7_stuck_move_m', 0.01)         #   이 거리도
        p('m7_stuck_rot_deg', 2.0)         #   이 각도도 못 움직이면 → 바로 후진
        p('m7_blocked_s', 1.0)             # 갈 방향이 없으면 이 시간 뒤 차로 다시 고르기, 두 배 뒤 후진
        p('m7_back_m', 0.15)               # 후진 거리 (오도메트리)
        p('m7_back_speed', 0.06)
        p('m7_back_timeout_s', 4.0)        # m7_back_m 한 번 기준 (더 많이 물러날 때는 비례해서 늘림)
        p('m7_back_grow', True)            # 같은 자리 n 번째 막힘이면 m7_back_m × n 만큼 물러남
        p('m7_back_grow_max', 3)
        p('m7_back_lidar_m', 0.05)         # 라이다로 본 뒤쪽 빈 거리가 이보다 짧으면 후진 멈춤
        p('m7_retry_radius_m', 0.20)       # 이 안에서 다시 막히면 '같은 자리' 재시도로 셈
        p('m7_retry_max', 3)               # 같은 자리 재시도 상한 — 넘으면 멈추고 에러 로그
        p('m7_align_deg', 3.0)             # 들어가자마자 차로 방향으로 제자리 정렬 (IMU), 이 안이면 끝
        p('m7_align_timeout_s', 3.0)
        p('m7_pass_margin_m', 0.30)        # 장애물 기억은 뒷면이 차 뒤끝보다 이만큼 뒤로 지나간 뒤에 지움 (시간으로 안 지움)
        p('m7_axis_snap_deg', 90.0)        # 차로 방향을 이 단위로 맞춤 (0 = 안 맞춤)
        p('m7_axis0_frames', 30)           # 출발 직선에서 트랙 축을 잴 프레임 수 (0 = 안 잼, odom 0° 를 트랙 축으로)
        p('m7_axis_need_yaw0', True)
        p('m7_axis0_from_start', False)    # 시뮬 전용: 출발 방향 = 트랙 축 (출발 자세가 트랙과 나란할 때만). 실차는 false (출발 직선에서 잼)
        p('yaw_drift_fix', True)            # 직선에서 차선 방향으로 IMU 방향 흐름을 재서 차로 방향에 반영
        p('yaw_drift_gain', 0.05)           #   장마다 이만큼 따라감 (13 장/초 → 약 1.5 s)
        p('yaw_drift_gate_deg', 20.0)       #   지금 추정과 이보다 다르면 버림 (코너·오인식)
        p('yaw_drift_max_curv', 0.03)
        p('yaw_drift_max_w', 0.15)          #   [rad/s] 이보다 빨리 돌면 안 잼        # 트랙 축을 못 쟀으면 90° 로 맞추지 않고 잰 방향 그대로
        p('m7_axis_frames', 20)            # 차로 방향 추정에 쓰는 최근 /lane/state 수
        p('m7_line_fix', True)             # 맨 왼쪽·맨 오른쪽 차로에서 노란 바깥선 하나만 보이면, 그 선으로 잰 차로 가운데와 격자 중심의 차이로
                                           # 격자를 옮김 (격자는 LIMO 로 잡은 그대로 + 이 보정. LIMO 가 차로 가운데에 없거나 오도메트리가 틀어져도)
        p('m7_line_fix_heading_deg', 15.0) # 차 방향이 차로 방향과 이 안이고 잡은 선 방향도 이 안일 때만 (rosbag: 15° 안 잰 가운데 오차 중앙 +0.3–0.8 cm)
        p('m7_line_fix_ahead_m', 0.60)     # center_offset_m 을 재는 곳 (base_link 앞, BEV 0.37–0.75 m + 카메라 0.084) — 방향 오차 보정용
        p('m7_line_fix_frames', 8)         # 이만큼 모이면 그 중앙값 × m7_line_fix_gain 만큼 격자를 옮김
        p('m7_line_fix_gain', 0.5)         # 한 번에 다 옮기면 측정 튐에 흔들림 (rosbag 재생: +8.7 → −5.4 cm)
        p('m7_line_fix_gate_m', 0.20)      # 격자 중심과 이보다 다르면 버림 (다른 선)
        p('m7_line_relocate', True)        # 끝 차로가 아니라고 믿는데 노란 바깥선 하나만 보이면 그 끝 차로로 격자를 다시 맞춤 (실차 11-06)
        p('m7_line_relocate_frames', 8)    #   이만큼 연속으로 모인 값의 중앙값만큼
        p('m7_line_relocate_spread_m', 0.08)  # 모인 값의 폭이 이보다 크면 안 씀
        p('m7_line_relocate_max_m', 0.60)  #   이보다 크게 옮겨야 하면 안 씀 (한 차로 0.52 + 여유 — 두 차로 건너뛰기 막음. 2D 시뮬 11-08 a7: +53.5 cm 로 반대쪽 끝)
        p('m7_line_relocate_heading_deg', 8.0)  # 차 방향·잡은 선 방향 모두 차로 방향과 이 안일 때만 (비스듬하면 노란 선 좌우를 잘못 봄)
        p('m7_line_relocate_max_s', 3.5)   #   구간 시작에서 이 안에서만 (끝의 A/B 분리대 노란 선과 헷갈리지 않게)
        p('m7_line_fix_max_m', 0.15)       # 보정 합 상한 (작은 보정만 — '위치 다시 맞춤' 은 따로, 이 상한에 안 묶임)
        # 후진 뒤 라이다 탐색 직진: 기억 (뎁스) 대신 지금 라이다로 가장 빈 방향을 골라 조금 가 본다 (사용자 11-08 — 실차 11-06: 막힌 주행마다
        # '옆 차로가 비어 있지 않음 → 유지' 후 같은 방향 후진·전진 반복)
        p('m7_entry_fix', True)            # 3차선 입구 앞 1차로 (곧고 두 노란 선 평행) 에서 잰 가운데로 격자 맞춤
        p('m7_entry_fix_curv', 0.05)       #   곡률 이보다 작은 장면만
        p('m7_entry_fix_parallel_px', 20.0)  # 두 선 간격 (BEV 아래·위) 차이가 이 안인 장면만 (깔때기 제외)
        p('m7_entry_fix_back_m', 2.0)      #   M7 켜진 자리 뒤 이 안 점만
        p('m7_entry_fix_max_m', 0.60)      #   옆 이보다 먼 점은 버림
        p('m7_entry_fix_min_pts', 5)
        p('m7_entry_stopline', True)       # 3차선 입구 흰 가로선이 보일 때 잰 1차로 가운데로 격자 맞춤
        p('m7_entry_stopline_max_m', 0.40) # 이보다 먼 점은 버림 (다른 선)
        p('m7_probe', True)
        p('m7_probe_deg', 35.0)            # 차로 방향 ± 이 안에서
        p('m7_probe_step_deg', 5.0)
        p('m7_probe_m', 0.40)              # 이만큼 가 보고 원래 로직으로
        p('m7_probe_min_m', 0.30)          # 가장 빈 방향도 이보다 짧으면 안 함
        p('m7_probe_timeout_s', 6.0)
        # M8 라바콘 (Follow-the-Gap)
        p('cones_topic', '/perception/cones')
        p('m8_require_m2', True)           # 순서 힌트: M2 를 지난 뒤에만 (시험: 라바콘 앞에서 출발하면 false)
        p('m8_trigger_m', 1.5)
        p('m8_trigger_count', 4)
        p('m8_confirm_frames', 3)
        p('m8_cone_half_m', 0.055)         # 라바콘 바닥판 반 변 (0.11 m) — 쓸기 검사 상자 (바퀴가 바닥판에 닿지 않게)
        p('m8_body_r_m', 0.022)            # 라이다 높이 몸통 반지름 — 틈 폭 = 중심 거리 − 몸통 지름 (라이다가 재는 실제 거리)
        p('m8_gap_min_m', 0.32)            # 차 폭 0.22 + 0.10
        p('m8_aim_m', 0.25)                # 틈 가운데선 위 앞 목표점 거리 (오프라인 7 배치: 0.25 가 S·default 에서 여유 최대)
        p('m8_max_heading_deg', 45.0)
        p('m8_look_m', 1.2)
        p('m8_check_m', 0.8)
        p('m8_margin_m', 0.01)             # 쓸기 검사 여유 (바닥판 상자 자체가 몸통보다 넓다)
        p('m8_slow_clear_m', 0.05)         # 이 여유로 쓸어 보면 닿으면 느리게
        p('m8_slow_check_m', 0.15)         # 그 쓸기 검사 길이
        p('m8_speed', 0.15)
        p('m8_slow_speed', 0.10)
        p('m8_stop_gap_m', 0.03)
        p('m8_rot_limit_deg', 15.0)        # 통로 안 제자리 회전 한도
        p('m8_stuck_check_s', 1.0)
        p('m8_back_m', 0.10)
        p('m8_retry_max', 3)
        p('m8_end_clear_m', 1.0)
        p('m8_end_pass_m', 0.4)
        p('m8_chain_step_min_m', 0.12)     # 틈 줄: 이어지는 두 틈 가운데점의 최소 앞뒤 간격. 대각 틈 (반 간격 0.15 m) 도 잇되 경계 (0.15) 는 피함.
                                           # 0.18 (대각 틈 뺌) + m8_chain_mu 를 같이 쓰면 S 배치에서 줄이 프레임마다 바뀌어 콘을 건드렸다 (6 번 중 3 번)
        p('m8_chain_mu', 2.5)              # 틈 줄 점수: 첫 틈이 직전 목표점에서 옆으로 1 m 떨어질 때마다 빼는 점수 (통로 끝에서 다리 줄 막기)
        p('m8_exit_slow_s', 1.5)
        p('m8_exit_deg', 6.0)              # 끝에서 통로 출구선과 방향 오차가 이 안이고
        p('m8_exit_lat_m', 0.04)           #   옆 오차가 이 안이면 차선 추종으로 넘김
        p('m8_exit_look_m', 0.3)
        p('m8_exit_s', 3.0)                # 맞추기 최대 시간
        p('m8_exit_speed', 0.15)
        p('m8_exit_lane', True)            # 끝 맞춤 선 = 차로 가운데선 (첫 쌍 가운데, 라바콘 구간 시작 때 차로 방향). false = 통로 출구선
        p('m8_exit_lane_max_deg', 35.0)    # 들어올 때 차로 방향이 본 라바콘 줄 (처음 → 마지막 쌍) 과 이보다 다르면 라바콘 줄 방향을 씀
        #                                    (2D 시뮬 11-08: 통로가 차로 옆으로 휘어 끝나면 출구선이 노란 선 위 → 코너에서 차선을 놓쳐 트랙 밖으로)
        # M1 신호등
        p('light_topic', '/perception/traffic_light')
        p('parking_start_topic', '/parking/start')
        p('m1_require_m8', True)           # 순서 힌트: M8 을 지난 뒤에만 (시험: M1 앞에서 출발하면 false)
        p('m1_confirm_frames', 5)          # 색 (크기 확인 없이도) 이만큼 연속 → M1_APPROACH
        p('m1_green_frames', 8)            # 크기까지 확인한 초록 이만큼 연속 → 발진·통과 (영상 30 Hz 면 약 0.27 s)
        p('m1_approach_speed', 0.15)
        p('m1_search_m', 3.0)              # 신호등을 본 뒤 이 거리 안에 정지선이 안 보이면 포기
        p('m1_exit_m', 0.3)
        p('m1_go_abort_m', 0.0)            # 통과하기로 한 뒤 노랑·빨강이면, 정지선까지 이만큼 넘게 남았을 때 다시 섬 (0 = 선에 닿기 전이면 언제나)
        p('m1_stop_frames', 3)             # 통과 중 크기 확인한 노랑·빨강 이만큼 (초록 없이) → 다시 섬
        p('m1_go_speed', 0.15)             # 초록 보고 통과할 때도 정지선을 지날 때까지 이 속도 (서행 — 그 사이 노랑이 되면 정지선에 설 수 있게)
        p('m1_log_s', 2.0)
        p('m1_line_max_gap_m', 1.15)       # 신호등 앞 이만큼 안의 가로선에만 섬 (횡단보도 앞 가로줄 ~0.75 m). 더 먼 선 (서행 구간 앞 정지선 ~1.65 m) 무시. 0 = 끔
        # M2 갈림길
        p('m2_label_a', 'turn_a')          # 모델 클래스 이름: A (왼쪽 길)
        p('m2_label_b', 'turn_b')          #                   B (오른쪽 길)
        p('m2_min_conf', 0.5)
        p('m2_min_box_px', 12)
        p('m2_confirm_frames', 3)
        p('m2_without_m7', True)           # M7 을 건너뛰어도 (정적 장애물 없음) 3차선에서 표지판 쪽 바깥 차로로 옮겨 갈림길을 따른다
        p('m2_zone_frames', 10)            # 노란 선이 한쪽만 보이는 프레임이 이만큼 연속 → 3차선 구간 (차로 번호: 왼쪽 선 = 맨 왼쪽 차로)
        p('m2_shift_speed', 0.2)           # 차로 옮길 때 속도 [m/s]
        p('m2_shift_look_m', 0.8)          # pure pursuit 앞 거리 (길수록 완만)
        p('m2_shift_delay_m', 0.8)         # 표지판·3차선 구간을 알아챈 뒤 이만큼 더 가서 옮김 (입구는 도로가 비스듬히 넓어져 선을 밟음)
        p('m2_axis_frames', 15)            # 차로 방향 = 3차선 구간에 들어온 뒤 (yaw + 차선 방향) 평균 (이만큼 모이면). 90° 로 맞추지 않음 —
                                           #   실차는 센서를 켠 뒤 차를 옮겨 오도메트리 0° 가 트랙과 43° 틀어져 대각선으로 나갔다 (11-05)
        p('m2_shift_abort_deg', 40.0)      # 옮기는 중 차로 방향과 이보다 틀어지면 포기
        p('m2_shift_tol_m', 0.04)          # 목표 차로 중심과 이만큼 안 + 방향 5° 안이면 옮기기 끝
        p('m2_shift_max_m', 3.0)           # 이만큼 가도 못 옮기면 포기 (차선 추종으로 돌아감)
        p('m2_hold_m', 2.5)                # 잠금 (분기점 가까움) 뒤 이만큼 갈 때까지 표지판 쪽 선만 따라감 → 갈림길 지남
        p('m2_hold_max_m', 6.0)            # 잠금을 못 해도 선 고정 시작 뒤 이만큼이면 끝
        p('m2_lock_box_px', 45)            # 이 크기 이상 (표지판 약 1.2 m 앞 = 분기점 가까이) 으로 같은 결과를 확인하면 잠금 — 더 안 바뀜
        # 연습 모드: 시설물이 없는 미션을 건너뜀 (예: ["M7", "M8", "M1"]). 건너뛴 미션은 다음 미션의 순서 조건에서 '지난 것' 으로 치고,
        # M4 주차는 남은 미션 중 마지막 (M3 → M7 → M8 → M1 순) 이 끝나면 시작한다 (다 건너뛰면 출발하자마자 주차 구역을 기다림).
        p('skip_missions', [''])
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.skip = {str(k).strip().upper() for k in g('skip_missions') if str(k).strip()}
        if self.skip - {'M3', 'M7', 'M8', 'M1'}:
            self.get_logger().warn(f"skip_missions 에 모르는 값 {sorted(self.skip - {'M3', 'M7', 'M8', 'M1'})} — "
                                   f"건너뛸 수 있는 것은 M3, M7 (M2 같이), M8, M1 (M5·M6 은 표지판·노면이 없으면 저절로 안 켜짐)")

        self.state = 'LANE'
        self._pos = None                   # 지금 오도메트리 (x, y)
        self._speed = 0.0
        self._odom_hist = collections.deque(maxlen=400)   # (시각, x, y) — 영상 시각의 위치를 찾는다
        self._hits = {'m5': 0, 'm3': 0, 'm4': 0}
        self._hit_win = {}
        self._resid_hits = 0
        self._m5_seen = self._m5_start = None
        self._m6_last_zig = self._m6_start = None
        self._m3 = {}                      # 이번 M3 의 기록
        self._m3_cooldown_from = None
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._imu = None                   # (시각, 적분 yaw, 자이로 z) — 자이로 적분, 기준은 M7 들어갈 때 odom yaw 에 맞춤
        self._scan = None                  # (시각, 점 x, 점 y) base_link 기준
        self._cones = []                   # lidar_processor 라바콘 (odom)
        self._m8, self._m8_hits, self._m8_done, self._m8_slow_until = {}, 0, False, 0.0
        self._m7_done = False
        self._m1, self._m1_hits, self._m1_done = {}, 0, False
        self._m3_done = False
        self._m4_started = False           # /parking/start 를 보냄 (또는 표지판 기다림)
        self._m4_pending = None            # 순서는 됐고 주차 표지판을 기다리는 중 (이유)
        self._m4_sign = False              # 갈림길 뒤에 주차 표지판을 봄
        self._light = None                 # 마지막 /perception/traffic_light
        self._lane = None                  # 마지막 /lane/state
        self._m2z = {'single': 0, 'side_obs': 0, 'zone': None, 'shift': None, 'hold': None, 'both': 0,
                     'dirs': collections.deque(maxlen=60), 'seen_at': None}   # M7 없이 M2 (m2_without_m7)
        self._lane_dirs = collections.deque(maxlen=60)   # 최근 (차 yaw + 차선 방향) — 노란 선을 잡은 프레임만
        self._bar_pt = None                               # M7 전에 본 3차선 입구 흰 가로선 (odom)
        self._entry_ctr = collections.deque(maxlen=60)   # 위 점 중 곧고 두 선이 평행한 장면만 (3차선 입구 앞 1차로) — M7 격자 기준
        self._line_ctr = collections.deque(maxlen=40)    # 위 점 중 흰 가로선 (3차선 입구 선) 이 보일 때 잰 것만 — M7 격자 기준
        self._lane_ctr = collections.deque(maxlen=60)    # 최근 1차로 가운데 점 (odom 좌표) — 노란 선 양쪽을 잡은 프레임만 (M7 격자 맞춤)
        self._yaw0, self._yaw0_buf = None, []   # 트랙 축 (출발 직선의 차선 방향, odom) — 차로 방향을 이 축 기준 90° 단위로 맞춤
        self._yaw_drift = self._yaw_drift_logged = 0.0   # 그 뒤 직선에서 잰 방향 흐름 (IMU 치우침)
        self._obs = []                     # 장애물 기억 (오도메트리 좌표): {'quads': 관측별 꼭짓점 4 개, 'c': 중심, 'seen': 시각}
        self._m7_hits = 0
        self._m7 = {}
        self._reset_until = 0.0            # 이 시각까지 lane_cmd.reset_window = True
        self._m2 = {'side': None, 'cand': None, 'n': 0, 'done': False, 'locked': False}   # 갈림길 결과 'A' | 'B', 확인 중인 것, 연속 수, 확정
        self.drive_pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.state_pub = self.create_publisher(String, g('state_topic'), 10)
        self.cmd_pub = self.create_publisher(LaneCmd, g('lane_cmd_topic'), 10)
        self.create_subscription(DetectionArray, g('detections_topic'), self.on_detections, 10)
        self.create_subscription(LaneState, g('lane_state_topic'), self.on_lane_state, 20)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 50)
        self.create_subscription(ObstacleArray, g('obstacles_topic'), self.on_obstacles, 10)
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseArray, g('cones_topic'), self.on_cones, 10)
        self.create_subscription(TrafficLight, g('light_topic'), self.on_light, 10)
        self.parking_pub = self.create_publisher(Empty, g('parking_start_topic'), 10)
        self.create_subscription(Empty, '/parking/done', self.on_parking_done, 10)
        self.create_timer(1.0 / float(g('rate_hz')), self.tick)
        self.get_logger().info(f"mission_manager: 상태 {self.state} — M5 {g('m5_labels')} → {g('m5_speed')} m/s, "
                               f"M6 잔차 ≥ {g('m6_resid_on')} px → {g('m6_speed')} m/s, M3 {g('m3_labels')} → 정지선 정지 {g('m3_wait_s')} s, "
                               f"M7 내 차로 앞 {g('m7_trigger_m')} m 장애물 → 회피, "
                               f"M2 {g('m2_label_a')} = A 왼쪽 / {g('m2_label_b')} = B 오른쪽 길"
                               + (f", 건너뜀 {sorted(self.skip)} (연습 모드)" if self.skip else ''))

    # ---------------------------------------------------------------- 공통
    ORDER = ('M3', 'M7', 'M2', 'M8', 'M1')   # 순서 조건이 걸린 미션 (M2 는 M7 안에서. M7 을 건너뛰면 m2_without_m7 로 따로)

    def done_or_skipped(self, key):
        """미션이 끝났는지 — 다음 미션의 순서 조건용. 건너뛴 미션은 '자기 앞 미션이 끝났을 때' 지난 것으로 친다
        (건너뛴 미션 때문에 앞쪽 순서 조건이 통째로 사라지지 않게: M8 을 건너뛰어도 M1 은 M2 를 지난 뒤에만)."""
        i = self.ORDER.index(key)
        if key in self.skip or (key == 'M2' and 'M7' in self.skip and not self.get_parameter('m2_without_m7').value):
            return True if i == 0 else self.done_or_skipped(self.ORDER[i - 1])
        return {'M3': self._m3_done, 'M7': self._m7_done, 'M2': self._m2['done'], 'M8': self._m8_done, 'M1': self._m1_done}[key]

    def m4_ready(self):
        """주차를 시작할 때: M1 이 끝났거나 (포기 포함), M1 을 건너뛰면 그 앞의 남은 마지막 미션까지 끝났을 때."""
        return self._m1_done or ('M1' in self.skip and self.done_or_skipped('M1'))

    def start_parking(self, why):
        self._m4_started = True
        if self.get_parameter('m4_require_sign').value and not self._m4_sign:
            self._m4_pending = why
            self.set_state('LANE', f'{why} → 주차 표지판을 볼 때까지 차선 추종')
            return
        self._m4_pending = None
        self.parking_pub.publish(Empty())
        self.set_state('M4_PARKING', f'{why} → /parking/start (parking_controller 가 주차 구역에서 넘겨받음)')

    def set_state(self, new, why):
        self.get_logger().info(f'[상태] {self.state} → {new} ({why})')
        self.state = new

    def _m3_exit_far(self):
        """횡단보도 (M3) 출발 자리에서 1 m 넘게 왔는가 — 횡단보도 자기 가로선을 3차선 입구 선으로 보지 않게."""
        gf = self._m3.get('go_from') if isinstance(getattr(self, '_m3', None), dict) else None
        return gf is not None and self.dist_from(gf) > 1.0

    def dist_from(self, p):
        return math.dist(self._pos, p) if (self._pos is not None and p is not None) else 0.0

    def pose_at(self, t):
        """시각 t 의 오도메트리 자세 (x, y, yaw) — 가장 가까운 기록. 없으면 지금."""
        h = self._odom_hist
        if not h:
            return (*self._pos, self._yaw) if self._pos else None
        i = bisect.bisect_left([e[0] for e in h], t)
        best = min((h[j] for j in (i - 1, i) if 0 <= j < len(h)), key=lambda e: abs(e[0] - t))
        return best[1], best[2], best[3]

    def pos_at(self, t):
        pose = self.pose_at(t)
        return None if pose is None else pose[:2]

    @staticmethod
    def fmt(p):
        return '없음' if p is None else f'({p[0]:.2f}, {p[1]:.2f})'

    def reset_all(self, why):
        """시뮬 리셋 (오도메트리가 출발 자세로 튐) 등 — 미션 상태·기억을 처음으로."""
        self.get_logger().info(f'[상태] {self.state} → LANE (처음으로: {why})')
        self.state = 'LANE'
        self._hits = {'m5': 0, 'm3': 0, 'm4': 0}
        self._hit_win = {}
        self._resid_hits = self._m7_hits = 0
        self._m3, self._m7, self._obs = {}, {}, []
        self._m2 = {'side': None, 'cand': None, 'n': 0, 'done': False, 'locked': False}
        self._m2z = {'single': 0, 'side_obs': 0, 'zone': None, 'shift': None, 'hold': None, 'both': 0,
                     'dirs': collections.deque(maxlen=60), 'seen_at': None}
        self._yaw0, self._yaw0_buf = None, []
        self._yaw_drift = self._yaw_drift_logged = 0.0
        self._m3_cooldown_from = None
        self._odom_hist.clear()
        self._cones, self._m8, self._m8_hits, self._m8_done, self._m8_slow_until = [], {}, 0, False, 0.0
        self._m7_done = False
        self._bar_pt = None
        self._m1, self._m1_hits, self._m1_done = {}, 0, False
        self._m3_done = False
        self._m4_started = False
        self._m4_pending, self._m4_sign = None, False
        self._light = None                 # 마지막 /perception/traffic_light

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self._pos is not None and math.dist(self._pos, (p.x, p.y)) > 0.5:
            self._pos = None
            self.reset_all(f'오도메트리가 {math.dist(self._odom_hist[-1][1:3], (p.x, p.y)) if self._odom_hist else 0:.2f} m 튐 — 리셋')
        self._pos = (p.x, p.y)
        self._speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self._yaw_rate = msg.twist.twist.angular.z
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._odom_hist.append((stamp_s(msg.header), p.x, p.y, self._yaw))

    # ---------------------------------------------------------------- 입력
    def hits(self, msg, key):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        labels = set(g(f'{key}_labels'))
        hit = [d for d in msg.detections
               if d.label in labels and d.confidence >= g(f'{key}_min_conf') and d.y2 - d.y1 >= g(f'{key}_min_box_px')]
        win = self._hit_win.setdefault(key, [])
        win.append(bool(hit))
        del win[:-g('sign_window_frames')]
        self._hits[key] = sum(win)          # 최근 sign_window_frames 장 중 잡힌 장 수 (한두 장 놓쳐도 0 으로 안 돌아감)
        return hit

    def on_detections(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.m2_update(msg)
        m3 = [d for d in self.hits(msg, 'm3') if d.y2 - d.y1 <= g('m3_max_box_px')]
        m5 = [d for d in self.hits(msg, 'm5') if d.y2 - d.y1 <= g('m5_max_box_px')]
        m4 = self.hits(msg, 'm4')
        if m4 and self._hits['m4'] >= g('m4_confirm_frames') and not self._m4_sign and self.done_or_skipped('M2'):
            best = max(m4, key=lambda d: d.confidence)
            self._m4_sign = True
            self.get_logger().info(f'[M4] 주차 표지판 ({best.label} {best.confidence:.2f}, 박스 높이 {best.y2 - best.y1}px, '
                                   f'최근 {g("sign_window_frames")} 장 중 {self._hits["m4"]} 장, odom {self.fmt(self._pos)})')
            if self._m4_pending:
                self.start_parking(self._m4_pending + ' + 주차 표지판')
        if self._m2['done'] or self._m8_done:
            return                         # 순서 힌트: 갈림길 (·라바콘) 뒤에는 서행·보행자 표지판 미션이 없다
        cooling = (self._m3_cooldown_from is not None
                   and self.dist_from(self._m3_cooldown_from) < g('m3_cooldown_m'))
        if (m3 and self.state in ('LANE',) + SLOW_STATES and self._hits['m3'] >= g('m3_confirm_frames') and not cooling
                and 'M3' not in self.skip):
            best = max(m3, key=lambda d: d.confidence)
            self._m3 = {'start': self._pos, 'line': None}
            self.set_state('M3_APPROACH', f'{best.label} {best.confidence:.2f} 박스 높이 {best.y2 - best.y1}px, '
                                          f'최근 {g("sign_window_frames")} 장 중 {self._hits["m3"]} 장, odom {self.fmt(self._pos)} → {g("m3_approach_speed")} m/s 로 정지선 찾기')
            return
        if m5 and self.state == 'M5_SLOW':
            self._m5_seen = self._pos
        elif (m5 and self.state == 'LANE' and self._hits['m5'] >= g('m5_confirm_frames')
              and not (g('m5_after_m3_ignore') and self._m3_done)):
            # 서행 구간은 출발 직후 — 횡단보도 (M3) 를 지난 뒤 서행 표지판은 오인식 (2D 시뮬 11-09 m2: 코너 직후 가짜 slow 153px
            # 로 M5 가 켜져 M7 이 안 켜지고 LIMO 에 부딪힘, 실차 11-05 run4 도 3차선에서 큰 가짜 박스)
            best = max(m5, key=lambda d: d.confidence)
            self._m5_seen = self._m5_start = self._pos
            self.set_state('M5_SLOW', f'{best.label} {best.confidence:.2f} 박스 높이 {best.y2 - best.y1}px, '
                                      f'최근 {g("sign_window_frames")} 장 중 {self._hits["m5"]} 장, odom {self.fmt(self._pos)}')

    def m2_update(self, msg):
        """갈림길 표지판 결과 기억 — 같은 결과 m2_confirm_frames 장 연속이면 채택, 다른 결과가 다시 확인되면 바꾼다 (마지막 것을 따름)."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        m = self._m2
        if m['done'] or m['locked']:
            return
        sides = {g('m2_label_a'): 'A', g('m2_label_b'): 'B'}
        hit = [d for d in msg.detections
               if d.label in sides and d.confidence >= g('m2_min_conf') and d.y2 - d.y1 >= g('m2_min_box_px')]
        if not hit:
            m['n'] = 0
            return
        best = max(hit, key=lambda d: d.confidence)
        side = sides[best.label]
        m['n'] = m['n'] + 1 if side == m['cand'] else 1
        m['cand'] = side
        if m['n'] >= g('m2_confirm_frames') and side != m['side']:
            old, m['side'] = m['side'], side
            if old is not None and self.state == 'M7_AVOID':
                self._m7['m2_flip'] = True     # 3차선 안에서 바뀜 → 지금 위치 차로에서 다시 고르게
            self.get_logger().info(f'[M2] 갈림길 표지판 {side} ({best.label} {best.confidence:.2f}, 박스 높이 {best.y2 - best.y1}px, '
                                   f'{m["n"]} 장 연속, odom {self.fmt(self._pos)}) → {"왼쪽" if side == "A" else "오른쪽"} 길'
                                   + (f' (전에 본 {old} 에서 바뀜 — 마지막 것을 따름)' if old else ''))
        if m['side'] == side and m['n'] >= g('m2_confirm_frames') and best.y2 - best.y1 >= g('m2_lock_box_px'):
            m['locked'] = True
            m['lock_pos'] = self._pos
            self.get_logger().info(f'[M2] 분기점 가까움 (표지판 박스 {best.y2 - best.y1}px ≥ {g("m2_lock_box_px")}px) → {side} 로 잠금, '
                                   f'odom {self.fmt(self._pos)}')

    def m2_side(self):
        """아직 지나지 않은 갈림길의 결과 'A' | 'B' | None (지난 뒤 — 다음 바퀴 3차선 등 — 에는 쓰지 않는다)."""
        return None if self._m2['done'] else self._m2['side']

    def m2_prefer(self):
        """옆 차로 둘이 같을 때 고를 쪽: 갈림길 결과가 있으면 그것, 없으면 m7_prefer."""
        if self.m2_side() is not None:
            return 'left' if self.m2_side() == 'A' else 'right'
        return self.get_parameter('m7_prefer').value

    def on_lane_state(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self._lane = msg
        if msg.yellow_side != 0 and 0.0 < msg.line_resid_px < g('m6_resid_off'):
            pose = self.pose_at(stamp_s(msg.header))
            if pose is not None:
                self._lane_dirs.append(wrap(pose[2] + msg.lane_heading_rad))
                if msg.found and msg.yellow_side == 2 and abs(msg.lane_heading_rad) < math.radians(15.0):
                    # 차선 가운데 = 차 앞 m7_line_fix_ahead_m, 옆 center_offset_m (+ 왼쪽) — odom 좌표 점으로 저장
                    px, py, yw = pose
                    ah, off = g('m7_line_fix_ahead_m'), msg.center_offset_m
                    pt = (px + ah * math.cos(yw) - off * math.sin(yw), py + ah * math.sin(yw) + off * math.cos(yw))
                    self._lane_ctr.append(pt)
                    if msg.stop_line_dist_m > 0:
                        self._line_ctr.append(pt)   # 흰 가로선 바로 앞 = 곧은 1차로 (코너 점이 안 섞임)
                    if (abs(msg.curvature) < g('m7_entry_fix_curv') and msg.width_near_px > 0 and msg.width_far_px > 0
                            and abs(msg.width_near_px - msg.width_far_px) <= g('m7_entry_fix_parallel_px')):
                        self._entry_ctr.append(pt)  # 곧고 평행 = 코너·깔때기 아닌 1차로
        # 3차선 입구 흰 가로선 기억 (M3 를 지난 뒤, M7 전) — odom 점으로. 코너에서 막 나와 비스듬해도 잡히면 씀
        if (msg.stop_line_dist_m > 0 and self._m3_done and not self._m7_done and self.state in ('LANE',) + SLOW_STATES
                and self._m3_exit_far()):
            pose = self.pose_at(stamp_s(msg.header))
            if pose is not None:
                dd = g('m7_cam_x') + msg.stop_line_dist_m
                self._bar_pt = (pose[0] + dd * math.cos(pose[2]), pose[1] + dd * math.sin(pose[2]))
        # M6: 톱니 지그재그 선 → 피팅 잔차가 커진다
        self._resid_hits = self._resid_hits + 1 if msg.line_resid_px >= g('m6_resid_on') else 0
        if self.state == 'M6_SLOW' and msg.line_resid_px >= g('m6_resid_off'):
            self._m6_last_zig = self._pos
        elif self.state == 'LANE' and self._resid_hits >= g('m6_confirm_frames') and not (self._m2['done'] or self._m8_done):
            self._m6_last_zig = self._m6_start = self._pos
            self.set_state('M6_SLOW', f'선 피팅 잔차 {msg.line_resid_px:.1f}px, {self._resid_hits} 장 연속, odom {self.fmt(self._pos)}')
        if self.state == 'M7_AVOID' and g('m7_line_fix'):
            self.m7_line_fix(msg)
        self.m2_zone_update(msg)
        self.track_axis_update(msg)
        # 정지선 거리 (M3·M1 공통)
        if self.state == 'M3_APPROACH':
            self.line_update(self._m3, msg, 'M3', '표지판')
        elif self.state == 'M1_APPROACH' and self.m1_line_near_light(msg):
            self.line_update(self._m1, msg, 'M1', '신호등')

    def line_update(self, st, msg, tag, what):
        """정지선 (카메라 → 가까운 끝) 을 영상 찍은 순간의 위치와 짝지어 둔다 — 이후는 오도메트리로 (line_remaining).
        처음 잡은 선과 m3_line_match_m 넘게 다른 측정은 다른 선 (횡단보도 뒤 가로선 등) 이라 버린다. M3·M1 이 같이 쓴다."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        if msg.stop_line_dist_m <= 0:
            return
        remain = msg.stop_line_dist_m - g('m3_cam_to_front_axle_m') - self.stop_back()
        pos = self.pos_at(stamp_s(msg.header))
        first = st.get('line') is None
        if not first:
            expect = st['line'][1] - math.dist(pos, st['line'][0])   # 잡은 선이 그 순간 있어야 할 거리
            if abs(remain - expect) > g('m3_line_match_m'):
                if not st.get('other_logged'):
                    st['other_logged'] = True
                    self.get_logger().info(f'[{tag}] 다른 가로선 무시: 측정 {remain:.3f} m, 잡은 정지선 예상 {expect:.3f} m')
                return
        st['line'] = (pos, remain)
        if first:
            self.get_logger().info(f'[{tag}] 정지선 보임: 카메라에서 {msg.stop_line_dist_m:.3f} m (덮은 비율 {msg.stop_line_cov:.2f}), '
                                   f'정지 목표까지 {remain:.3f} m, {what} 뒤 {self.dist_from(st["start"]):.2f} m')

    def on_imu(self, msg):
        t = stamp_s(msg.header)
        wz = msg.angular_velocity.z
        if self._imu is None:
            self._imu = (t, 0.0, wz)
        else:
            t0, yaw, w0 = self._imu
            dt = t - t0
            if 0.0 < dt < 0.5:
                yaw += 0.5 * (w0 + wz) * dt
            self._imu = (t, yaw, wz)

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, np.float32)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        x = r[ok] * np.cos(a[ok]) + self.get_parameter('laser_x').value
        y = r[ok] * np.sin(a[ok])
        body = (np.abs(x) < CAR_HALF_L + 0.005) & (np.abs(y) < CAR_HALF_W + 0.005)   # 내 차 몸에 맞은 점
        self._scan = (stamp_s(msg.header), x[~body], y[~body])
        rr = np.where(ok, r, np.inf)
        self._scan_raw = (stamp_s(msg.header), a, rr)   # M7 라이다로 기억 지우기 — 빔이 뚫고 지나갔는지 (안 맞은 빔 = inf) 봐야 함

    def rear_free(self):
        """라이다로 본 차 뒤 (차 폭 + 3 cm 통로) 의 빈 거리 [m]. 라이다가 없거나 0.5 s 넘게 끊겼으면 None."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._scan is None or now - self._scan[0] > 0.5:
            return None
        _, x, y = self._scan
        sel = (x < -CAR_HALF_L) & (np.abs(y) < CAR_HALF_W + 0.03)
        return float((-x[sel] - CAR_HALF_L).min()) if sel.any() else math.inf

    def heading(self):
        """M7 방향: 들어갈 때 odom yaw 에 맞춘 자이로 적분 (없으면 odom yaw)."""
        m = self._m7
        if self._imu is not None and m.get('imu_off') is not None:
            return wrap(self._imu[1] + m['imu_off'])
        return self._yaw

    # ---------------------------------------------------------------- M7 장애물
    def on_obstacles(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        pose = self.pose_at(stamp_s(msg.header))
        if pose is None:
            return
        px, py, pyaw = pose
        c, s = math.cos(pyaw), math.sin(pyaw)
        now = self.get_clock().now().nanoseconds * 1e-9
        trig = False
        for o in msg.obstacles:
            if o.y_left - o.y_right > g('m7_max_width_m'):
                continue                   # LIMO (0.22 m) 보다 훨씬 넓음 = 바닥 (기울어짐)·벽 — 기억에도 안 넣음
            if o.height > g('m7_max_height_m'):
                continue                   # LIMO (0.25 m) 보다 훨씬 높음 = 트랙 밖 벽·사람 — 기억에도 안 넣음
            # 비스듬히·옆에서 본 관측: 보이는 면이 앞면이 아니라 가정 길이가 엉뚱한 쪽으로 붙는다 (차 뒤까지 늘어난 상자로 갇힘, 11-01)
            side_view = (abs(math.atan2((o.y_left + o.y_right) / 2, max(o.x_near, 0.05))) > math.radians(g('m7_oblique_deg'))
                         or o.x_near < CAR_HALF_L + 0.15)
            # base_link 상자 → 오도메트리 좌표 꼭짓점 (뒷면은 안 보이므로 앞면 + 가정 길이).
            # m7_side_no_len: 옆에서 본·화각 가장자리에 잘린 관측은 '기억과 합칠지' 를 가정 길이 없는 상자로 판단한다.
            #   옆 차로 LIMO 를 지나갈 때 화면에 옆면 끝만 보이면 그 끝을 앞면으로 보고 0.33 m 를 앞으로 늘려, 실제보다 0.3 m 앞에
            #   상자가 생기고 기억한 상자와 중심이 m7_merge_m 넘게 떨어져 새 장애물이 됐다 (2D 시뮬 11-04: 가운데 차로에 없는 장애물
            #   → 차로 변경 못 하고 멈춤). 차로 변경 중 차가 비스듬하면 side_view 각도 조건을 빠져나가므로 화각 경계도 본다.
            #   처음 보는 장애물 (합칠 기억이 없음) 은 예전처럼 가정 길이를 붙여 보수적으로 등록 (앞 LIMO 에 가려졌다가 옆에서
            #   처음 보이는 LIMO 의 뒷부분을 모르고 일찍 끼어들지 않게 — 리뷰 11-04). 합칠 때는 상자에 안 넣고 seen 만 갱신.
            hh = g('m7_cam_hfov_deg') / 2 - g('m7_fov_edge_deg')
            clipped = any(abs(math.degrees(math.atan2(y, max(o.x_near - g('m7_cam_x'), 0.01))) - g('m7_cam_yaw_deg')) > hh
                          for y in (o.y_left, o.y_right))
            no_len = g('m7_side_no_len') and (side_view or clipped)
            yr, yl = o.y_right, o.y_left
            if g('m7_obstacle_width_m') > 0 and abs((yl + yr) / 2) > g('m7_side_min_m'):
                if (yl + yr) / 2 > 0:      # 왼쪽 장애물: 먼 옆면 = 왼쪽 끝
                    yl = max(yl, yr + g('m7_obstacle_width_m'))
                else:
                    yr = min(yr, yl - g('m7_obstacle_width_m'))

            def quad(x2):
                return [(px + c * bx - s * by, py + s * bx + c * by) for bx in (o.x_near, x2) for by in (yr, yl)]
            full = quad(max(o.x_far, o.x_near + g('m7_obstacle_len_m')))
            pts = quad(o.x_far) if no_len else full
            cx, cy = sum(p[0] for p in pts) / 4, sum(p[1] for p in pts) / 4
            # 실제로 보인 점 (가정 길이·폭 없이) 의 가운데가 기억한 상자 안 (m7_inside_margin_m 여유) 이면 같은 장애물이다.
            # 옆면 끝·화각에 잘린 끝·앞 LIMO 에 가려 뒷부분만 보인 조각은 가정 길이를 붙이면 중심이 0.3 m 넘게 옮겨져
            # 새 (가짜) 장애물이 됐다 (2D 시뮬 11-04). 같은 LIMO 의 조각이면 반드시 그 상자 안에 있다.
            seen_c = (px + c * (o.x_near + o.x_far) / 2 - s * (o.y_left + o.y_right) / 2,
                      py + s * (o.x_near + o.x_far) / 2 + c * (o.y_left + o.y_right) / 2)
            inside, front_a = (self.obs_containing(seen_c, g('m7_inside_margin_m'),
                                                   (px + c * o.x_near - s * (o.y_left + o.y_right) / 2,
                                                    py + s * o.x_near + c * (o.y_left + o.y_right) / 2))
                               if g('m7_side_no_len') else (None, None))
            fragment = inside is not None and (no_len or side_view or front_a > g('m7_front_match_m'))
            if fragment:
                inside['seen'] = now           # 조각 관측 (옆·잘림·앞면이 아닌 뒷부분) — 상자는 그대로
                cands = []
            else:
                cands = [inside] if inside is not None else self._obs
            for m in cands:                # 같은 장애물 (중심 m7_merge_m 안) 이면 관측을 모은다 (상자는 관측별 범위의 중앙값)
                if m is inside or math.dist(m['c'], (cx, cy)) < g('m7_merge_m'):
                    if (side_view and len(m['quads']) >= 3) or no_len:
                        # 앞에서 본 관측이 이미 있으면 상자는 그대로, 봤다는 것만. 길이 없이 본 (옆·잘린) 관측은 같은 장애물을 찾는 데만
                        # 쓰고 상자에는 안 넣는다 — 차로 변경 중 옆 LIMO 를 오래 보면 짧은 관측이 과반이 돼 중앙값 상자가 다시
                        # 짧아지므로 (리뷰 11-04)
                        m['seen'] = now
                        break
                    m['bottom'] = min(m.get('bottom', 9.0), o.bottom)
                    m['quads'] = (m['quads'] + [pts])[-12:]
                    cs = [(sum(p[0] for p in q) / 4, sum(p[1] for p in q) / 4) for q in m['quads']]
                    m['c'] = (sum(c[0] for c in cs) / len(cs), sum(c[1] for c in cs) / len(cs))
                    m['seen'] = now
                    break
            else:
                if not fragment:           # 합칠 기억이 없음 = 처음 보는 장애물 → 가정 길이를 붙여 보수적으로
                    fx, fy = sum(p[0] for p in full) / 4, sum(p[1] for p in full) / 4
                    self._obs.append({'quads': [full], 'c': (fx, fy), 'seen': now, 'bottom': o.bottom})
            mid = (o.y_left + o.y_right) / 2
            hm = g('m7_trigger_half_m')
            # 내 차로 = 장애물 가운데가 ±half 안, 또는 (m7_trigger_overlap) 장애물이 차가 갈 폭 ±half 에 조금이라도 걸침.
            # 코너를 막 나와 차가 왼쪽을 향한 채면 가운데 LIMO 가 차 오른쪽 0.1–0.4 m 에 보여 가운데 기준으로는 늦게 켜짐
            # → 깔때기에서 왼쪽으로 쏠린 뒤 켜져 격자가 반 차로 밀림 (2D 시뮬 11-09: 8 번 중 2–3 번, 갈림길 실패·유리벽)
            inlane = (o.y_right < hm and o.y_left > -hm) if g('m7_trigger_overlap') else abs(mid) < hm
            if (inlane and o.x_near < g('m7_trigger_m')
                    and o.y_left - o.y_right >= g('m7_min_width_m') and o.height >= g('m7_min_height_m')):
                trig = True
                # 무엇을 보고 켜졌는지 (유리벽·바닥 오검출 확인용 — 뎁스 로그는 2 s 마다라 그 장면이 안 남음)
                self._m7_trig_desc = (f'앞 {o.x_near:.2f} m 옆 {o.y_right:+.2f}~{o.y_left:+.2f} 폭 {o.y_left - o.y_right:.2f} '
                                      f'높이 {o.height:.2f}')
        self.prune_obs(now)
        self._m7_hits = self._m7_hits + 1 if trig else 0
        two_yellow = self._lane is not None and self._lane.yellow_side == 2
        turning = abs(self._yaw_rate) > g('m7_max_yaw_rate')
        if (self.state in ('LANE',) + SLOW_STATES and self._m7_hits >= g('m7_confirm_frames') and not two_yellow
                and not turning and not self._m7_done and not self._m2['done'] and not self._m8_done and 'M7' not in self.skip):
            self.m7_enter()

    def obs_containing(self, q, margin, near=None):
        """점 q (odom) 를 품는 기억 장애물 (관측별 상자 중 하나라도, 사방 margin 여유) 과, near 점이 그 상자 앞면에서
        안쪽으로 얼마나 들어가 있는지 [m] (앞면 관측이면 ≈ 0). 없으면 (None, None)."""
        for m in self._obs:
            for quad in m['quads']:
                p0, p1, p2 = quad[0], quad[1], quad[2]          # (앞, 오른쪽), (앞, 왼쪽), (뒤, 오른쪽)
                ux, uy = p2[0] - p0[0], p2[1] - p0[1]
                vx, vy = p1[0] - p0[0], p1[1] - p0[1]
                lu, lv = math.hypot(ux, uy) or 1e-6, math.hypot(vx, vy) or 1e-6
                a = ((q[0] - p0[0]) * ux + (q[1] - p0[1]) * uy) / lu
                b = ((q[0] - p0[0]) * vx + (q[1] - p0[1]) * vy) / lv
                if -margin <= a <= lu + margin and -margin <= b <= lv + margin:
                    an = 0.0 if near is None else ((near[0] - p0[0]) * ux + (near[1] - p0[1]) * uy) / lu
                    return m, an
        return None, None

    def prune_obs(self, now):
        """장애물 기억 정리. 차 가까이 (1 m 안) 붙은 것은 뎁스에 안 보여도 (카메라에 너무 가까움) 시간으로 지우지 않고,
        오도메트리로 그 장애물 뒷면이 차 뒤끝보다 m7_pass_margin_m 넘게 뒤로 간 뒤에 지운다. 4 m 넘게 멀어진 것도 지움.
        1 m 밖이고 30 s 동안 다시 안 보인 것 (잘못 본 것) 은 지움."""
        if self._pos is None:
            return
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        a = self._m7['axis'] if self.state == 'M7_AVOID' and self._m7 else self._yaw   # M7 중에는 차로 방향 기준 (차가 비스듬해도)
        c, s = math.cos(a), math.sin(a)
        keep = []
        for ob in self._obs:
            d = self.dist_from(ob['c'])
            front = max(((p[0] - self._pos[0]) * c + (p[1] - self._pos[1]) * s) for q in ob['quads'][-3:] for p in q)
            passed = front < -(CAR_HALF_L + g('m7_pass_margin_m'))
            if d < 4.0 and not passed and (d < 1.0 or now - ob['seen'] < 30.0):
                keep.append(ob)
        self._obs = keep
        if self.state == 'M7_AVOID' and self._m7 and g('m7_dedup_overlap'):
            self.dedup_obs()

    def dedup_obs(self):
        """차로 좌표 상자가 겹치는 기억 둘은 같은 장애물 (실물 LIMO 끼리는 겹칠 수 없다). 관측이 많은 쪽만 남긴다.
        2D 시뮬 11-06 (기본 배치·A, 실차 오차): 차로 1 LIMO 하나가 0.22 m 어긋난 상자 둘로 기억돼 다음 줄 (0.44 m 뒤) 까지
        막혀 보임 → 차로 0 에서 옆 차로로 못 빠지고 멈춤. 관측 수가 같으면 최근에 본 쪽을 남긴다."""
        boxes = [(ob, self.m7_box(ob)) for ob in self._obs]
        drop = set()
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                (a, ba), (b, bb) = boxes[i], boxes[j]
                if id(a) in drop or id(b) in drop:
                    continue
                if min(ba[1], bb[1]) > max(ba[0], bb[0]) and min(ba[3], bb[3]) > max(ba[2], bb[2]):
                    # 관측 수가 같으면 최근에 본 쪽 — 처음 관측은 M7 진입 순간 (차가 차로와 15° 틀어짐) 비스듬히 본 것이라 어긋남
                    # (2D 시뮬 11-09 q8: 오른쪽 LIMO 를 0.3 m 앞으로 기억 → 정렬 뒤 맞게 본 관측을 버리고 틀린 상자를 남겨 막힘·후진)
                    ka, kb = (len(a['quads']), a['seen']), (len(b['quads']), b['seen'])
                    lose, win = (a, b) if ka < kb else (b, a)
                    lb, wb = (ba, bb) if lose is a else (bb, ba)
                    win['seen'] = max(win['seen'], lose['seen'])
                    drop.add(id(lose))
                    self.get_logger().info(f'[M7] 겹친 기억 합침: s {lb[0]:.2f}~{lb[1]:.2f} l {lb[2]:+.2f}~{lb[3]:+.2f} '
                                           f'(관측 {len(lose["quads"])}) 버리고 s {wb[0]:.2f}~{wb[1]:.2f} l {wb[2]:+.2f}~{wb[3]:+.2f} '
                                           f'(관측 {len(win["quads"])}) 남김')
        if drop:
            self._obs = [ob for ob in self._obs if id(ob) not in drop]

    def track_axis_update(self, msg):
        """출발 직선 (노란 선 양쪽, 곧음) 의 차선 방향 m7_axis0_frames 장 평균 = 트랙 축. 오도메트리 0° 가 트랙과 틀어져 있어도
        (실차: 센서를 켠 뒤 차를 옮김 — 11-05 에 43°) 차로 방향을 이 축 기준 90° 단위로 맞출 수 있게."""
        n = int(self.get_parameter('m7_axis0_frames').value)
        if self._yaw0 is None and self.get_parameter('m7_axis0_from_start').value and self._pos is not None:
            self._yaw0 = self._yaw
            self.get_logger().info(f'[축] 트랙 축 = odom {math.degrees(self._yaw0):.1f}° (출발 방향 — m7_axis0_from_start)')
        if self._yaw0 is not None:
            self.yaw_drift_update(msg)
            return
        if n <= 0 or not msg.found or msg.yellow_side != 2 or abs(msg.curvature) > 0.05:
            return
        pose = self.pose_at(stamp_s(msg.header))
        if pose is None:
            return
        self._yaw0_buf.append(wrap(pose[2] + msg.lane_heading_rad))
        if len(self._yaw0_buf) >= n:
            b = self._yaw0_buf
            m = math.atan2(sum(math.sin(a) for a in b), sum(math.cos(a) for a in b))
            if max(abs(wrap(a - m)) for a in b) < math.radians(8.0):
                self._yaw0 = m
                self.get_logger().info(f'[축] 트랙 축 = odom {math.degrees(m):.1f}° (출발 직선 차선 방향 {n} 장) — '
                                       f'차로 방향은 이 축 기준 {self.get_parameter("m7_axis_snap_deg").value:.0f}° 단위')
            else:
                self._yaw0_buf = b[n // 2:]   # 흔들림 — 반만 남기고 더 모음

    def yaw_drift_update(self, msg):
        """방향 (odom yaw = IMU 적분) 이 흐른 만큼 재기. 직선 (노란 선 양쪽, 곧음, 안 돎) 에서 yaw + 차선 방향 = 트랙 축 + 90° 의 배수
        + 흐름. 그 차이를 천천히 따라가 lane_axis 가 (트랙 축 + 흐름) 기준으로 맞춘다. 2D 시뮬 가혹 조건 11-06: 자이로 치우침
        0.11°/s 로 3차선에 왔을 때 26° 틀어져 M7 이 비스듬히 가다 멈춤, 45° 넘으면 90° 틀린 차로 방향을 고름."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        if (not g('yaw_drift_fix') or not msg.found or msg.yellow_side != 2 or abs(msg.curvature) > g('yaw_drift_max_curv')
                or abs(self._yaw_rate) > g('yaw_drift_max_w') or self.state == 'M7_AVOID'):
            return
        pose = self.pose_at(stamp_s(msg.header))
        if pose is None:
            return
        est = wrap(pose[2] + msg.lane_heading_rad)
        snap = math.radians(90.0)
        base = self._yaw0 + self._yaw_drift
        k = round(wrap(est - base) / snap)
        inn = wrap(est - (base + k * snap))         # 지금 추정 흐름과의 차이
        if abs(inn) > math.radians(g('yaw_drift_gate_deg')):
            return                                 # 코너 진입·차선 오인식 — 버림
        self._yaw_drift = wrap(self._yaw_drift + g('yaw_drift_gain') * inn)
        if abs(self._yaw_drift - self._yaw_drift_logged) > math.radians(2.0):
            self._yaw_drift_logged = self._yaw_drift
            self.get_logger().info(f'[축] 방향 흐름 {math.degrees(self._yaw_drift):+.1f}° (직선에서 차선 방향으로 잼, odom {self.fmt(self._pos)})')

    def lane_axis(self):
        """차로 방향: 최근 (yaw + 차선 방향) 중앙값 → m7_axis_snap_deg 단위로 (트랙 직선은 출발 방향과 평행·직각)."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        recent = list(self._lane_dirs)[-int(g('m7_axis_frames')):]
        if recent:
            c = sum(math.cos(a) for a in recent)
            sn = sum(math.sin(a) for a in recent)
            est, how = math.atan2(sn, c), f'최근 차선 방향 {len(recent)} 장'
        else:
            est, how = self._yaw, '지금 yaw'
        snap = math.radians(g('m7_axis_snap_deg'))
        if snap <= 0:
            return est, est, how
        y0 = None if self._yaw0 is None else wrap(self._yaw0 + self._yaw_drift)
        if y0 is None:
            if g('m7_axis_need_yaw0'):
                return est, est, how + ', 트랙 축 모름 → 맞추지 않음'
            y0 = 0.0
        return wrap(y0 + round(wrap(est - y0) / snap) * snap), est, how + (f', 트랙 축 {math.degrees(self._yaw0):.1f}°' if self._yaw0 is not None else '')

    def m7_enter(self):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        # 차로 방향: 최근 (yaw + 차선 방향) 중앙값 → m7_axis_snap_deg 단위로
        axis, est, how = self.lane_axis()
        side = self._lane.yellow_side if self._lane is not None else 0
        now = self.get_clock().now().nanoseconds * 1e-9
        self._m7 = {'origin': self._pos, 'axis': axis, 'side': side, 'n': None, 'changes': 0, 'mode': None,
                    'imu_off': None if self._imu is None else wrap(self._yaw - self._imu[1]),
                    'align': now, 'blocked': None, 'back': None, 'gave_up': False,
                    'mon': StuckMonitor(g('m7_stuck_check_s'), g('m7_stuck_move_m'), g('m7_stuck_rot_deg')),
                    'retry': RetryCounter(g('m7_retry_radius_m'))}
        grid = self.m7_grid()
        n_car = round((self.m7_frame(self._pos)[1] - grid['phase']) / g('m7_lane_width_m'))
        self._m7['n'] = n_car
        bx = sorted(self.m7_box(ob) for ob in self._obs)
        self.get_logger().info('[M7] 기억한 장애물 (s 앞뒤, l 오른쪽~왼쪽, 차 기준 m): ' +
                               ', '.join(f'[{b[0]:.2f}~{b[1]:.2f}, {b[2]:+.2f}~{b[3]:+.2f}]' for b in bx[:8]))
        self.set_state('M7_AVOID', f'내 차로 앞 장애물 {self._m7_hits} 장 연속 (마지막: {getattr(self, "_m7_trig_desc", "?")}), 차로 방향 {math.degrees(axis):.0f}° '
                                   f'(추정 {math.degrees(est):.1f}°, {how}), 차로 격자 {grid["how"]}, 지금 차로 {self.m7_k(n_car)} '
                                   f'(0 = 맨 왼쪽), 기억한 장애물 {len(self._obs)} 개')

    def m7_box(self, ob):
        """기억한 장애물의 차로 좌표 상자 (s_min, s_max, l_min, l_max) — 관측별 범위의 중앙값 (모두 합치면 각도마다 조금씩
        달리 본 꼭짓점이 쌓여 실제보다 커진다)."""
        ext = []
        for q in ob['quads']:
            pts = [self.m7_frame(p) for p in q]
            ext.append((min(p[0] for p in pts), max(p[0] for p in pts), min(p[1] for p in pts), max(p[1] for p in pts)))
        return tuple(sorted(e[i] for e in ext)[len(ext) // 2] for i in range(4))

    def m7_dump(self):
        """디버그: 차 위치와 기억한 장애물 상자 (차로 좌표 s 앞, l 왼쪽 — M7 들어간 곳 기준), 차로 가운데 l."""
        if not self._m7 or self._pos is None:
            return ''
        W = self.get_parameter('m7_lane_width_m').value
        car = self.m7_frame(self._pos)
        lanes = ', '.join(f'{k}:{self.m7_lane_l(k):+.2f}' for k in range(self.get_parameter('m7_lanes').value))
        boxes = ', '.join(f's {b[0]:.2f}~{b[1]:.2f} l {b[2]:+.2f}~{b[3]:+.2f}' for b in sorted(self.m7_box(ob) for ob in self._obs))
        return f'차 s {car[0]:.2f} l {car[1]:+.2f}, 차로 가운데 l [{lanes}] (폭 {W}), 기억 {len(self._obs)} 개: {boxes}'

    def m7_frame(self, p):
        o, a = self._m7['origin'], self._m7['axis']
        dx, dy = p[0] - o[0], p[1] - o[1]
        return dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)

    def m7_grid(self):
        """차로 격자: 중심 l = phase + n × W. 차로 번호 k = n_left − n (0 = 맨 왼쪽, l 이 큰 쪽).
        정차 LIMO (폭 ≥ m7_min_width_m) 중심들의 옆 위치로 phase 를 맞추고, LIMO 가 걸친 n 이 세 개면 그게 세 차로.
        아니면 차의 지금 차로를 노란 선 쪽 (-1 왼쪽만 = 맨 왼쪽, 0 없음 = 가운데, +1 오른쪽만 = 맨 오른쪽) 으로 정한다.
        LIMO 로 세 차로를 한 번 확인하면 고정 (그 뒤 기억이 늘어 반올림이 흔들려도 차로 번호가 바뀌지 않게)."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        if self._m7.get('grid', {}).get('fixed'):
            return self._m7['grid']
        W = g('m7_lane_width_m')
        if g('m7_entry_lane') >= 0:
            # 들어간 자리 (차로 좌표 원점, l = 0) 가 m7_entry_lane 차로 가운데. 노란 선 보정 (line_corr) 은 m7_line_fix 가 phase 에 더함.
            # 진입 직전 1차로 (노란 선 양쪽) 에서 잰 차선 가운데 점들이 있으면 그 l 중앙값으로 맞춘다 — 차가 가운데에서 6 cm 쯤 치우쳐
            # 들어와 격자가 밀리고 옆 차로 LIMO 가 경계에 걸친 것처럼 보였다 (실차 11-06 full10: 노란 선 보정이 나중에 +6.5 cm)
            ctr = 0.0
            how = f'들어간 자리 = 차로 {int(g("m7_entry_lane"))} (트랙 구조)'
            ef = None
            if g('m7_entry_fix'):
                # 3차선 입구 앞 1차로 (노란 선 양쪽, 곧고 평행) 에서 잰 차선 가운데 → 가운데 차로 중심. 깔때기에서 왼쪽으로 쏠린 뒤
                # M7 이 켜져도 맞는 자리를 안다 (2D 시뮬 11-09: 8 번 중 2–3 번 0.35–0.4 m 왼쪽에서 켜져 격자가 반 차로 밀림 → 갈림길 실패·유리벽)
                pts = [self.m7_frame(q) for q in self._entry_ctr]
                ls = sorted(l for s_, l in pts if -g('m7_entry_fix_back_m') <= s_ <= 0.3 and abs(l) <= g('m7_entry_fix_max_m'))
                if len(ls) >= g('m7_entry_fix_min_pts'):
                    ef = ls[len(ls) // 2]
                    ctr = ef
                    how += f', 입구 앞 1차로에서 잰 가운데 {ef * 100:+.1f} cm 로 맞춤 ({len(ls)} 점, 흩어짐 {(ls[-1] - ls[0]) * 100:.0f} cm)'
                else:
                    how += f', 입구 앞 1차로 가운데 점 {len(ls)} 개 — 부족해 안 맞춤'
            if ef is not None:
                pass
            elif g('m7_entry_stopline'):
                # 3차선 입구 흰 가로선 앞 1차로 (노란 선 양쪽) 는 가운데 차로와 이어짐 → 그 선이 보일 때 잰 차선 가운데로 격자를 맞춘다
                # (사용자 11-08 제안. 예전 m7_entry_center_fix 는 코너 점이 섞여 −109 cm 가 나왔음)
                pts = [self.m7_frame(q) for q in self._line_ctr]
                ls = sorted(l for s_, l in pts if -1.5 <= s_ <= 0.6 and abs(l) <= g('m7_entry_stopline_max_m'))
                if len(ls) >= 3:
                    ctr = ls[len(ls) // 2]
                    how += f', 흰 가로선 앞에서 잰 차선 가운데 {ctr * 100:+.1f} cm 로 맞춤 ({len(ls)} 점)'
                else:
                    how += f', 흰 가로선 앞 차선 가운데 점 {len(ls)} 개 — 부족해 안 맞춤'
            elif g('m7_entry_center_fix'):
                pts = [self.m7_frame(q) for q in self._lane_ctr]
                ls = sorted(l for s_, l in pts if -g('m7_entry_center_back_m') <= s_ <= 0.3)
                if len(ls) >= 5:
                    med = ls[len(ls) // 2]
                    if abs(med) <= g('m7_entry_center_max_m'):
                        ctr = med
                        how += f', 진입 전 1차로 가운데 {med * 100:+.1f} cm 로 맞춤 ({len(ls)} 점)'
                    else:
                        how += f', 진입 전 1차로 가운데 {med * 100:+.1f} cm — 너무 커서 안 씀'
                else:
                    how += f', 진입 전 1차로 가운데 점 {len(ls)} 개 — 부족해 안 맞춤'
            self._m7['grid'] = {'phase': ctr + self._m7.get('line_corr', 0.0), 'n_left': int(g('m7_entry_lane')),
                                'how': how, 'fixed': True}
            self.get_logger().info(f'[M7] 차로 격자 고정: {self._m7["grid"]["how"]}')
            return self._m7['grid']
        sc, lc = self.m7_frame(self._pos)
        ls = []
        for ob in self._obs:                # 차 주변 (뒤 1 m ~, 옆 ±1.2 m) 의 정차 LIMO 만 — 지나온 표지판 등은 빼고
            _, smax, lmin, lmax = self.m7_box(ob)
            if lmax - lmin >= g('m7_min_width_m') and smax > sc - 1.0 and abs((lmin + lmax) / 2 - lc) <= 1.2:
                ls.append((lmin + lmax) / 2)
        if ls:
            ang = [2 * math.pi * (lv / W) for lv in ls]
            phase = W * math.atan2(sum(math.sin(a) for a in ang), sum(math.cos(a) for a in ang)) / (2 * math.pi)
            ns = sorted({round((lv - phase) / W) for lv in ls})
            how = f'정차 LIMO {len(ls)} 대 (차로 {len(ns)} 개)'
        else:
            phase, ns = (self._lane.center_offset_m if self._lane is not None else 0.0), []
            how = '차선 인식 (LIMO 없음)'
        phase += self._m7.get('line_corr', 0.0)   # 노란 선 보정 (m7_line_fix) — LIMO 차로 번호를 정한 뒤에 더함
        fixed = len(ns) == g('m7_lanes') and ns[-1] - ns[0] == g('m7_lanes') - 1
        if fixed:
            n_left = ns[-1]
        else:
            n_car = round((self.m7_frame(self._pos)[1] - phase) / W)
            k_car = {-1: 0, 0: 1, 1: 2}.get(self._m7.get('side', 0), 1)
            n_left = n_car + k_car
        old = self._m7.get('grid')
        if old is not None and self._m7.get('n') is not None:
            # 격자 phase 가 반 칸 넘게 바뀌면 같은 차로라도 n 이 달라진다 → 목표 차로를 위치로 다시 맞춤
            lt_old = old['phase'] + self._m7['n'] * W
            self._m7['n'] = round((lt_old - phase) / W)
        self._m7['grid'] = {'phase': phase, 'n_left': n_left, 'how': how, 'fixed': fixed}
        if fixed and not (old or {}).get('fixed'):
            self.get_logger().info(f'[M7] 차로 격자 고정: {how}, 목표 차로 {self.m7_k(self._m7["n"]) if self._m7.get("n") is not None else "?"}')
        return self._m7['grid']

    def m7_k(self, n):
        return self._m7['grid']['n_left'] - n

    def m7_line_fix(self, msg):
        """맨 왼쪽 (노란 선 왼쪽만) / 맨 오른쪽 (오른쪽만) 차로를 곧게 가는 중이면 lane_follower 가 그 노란 선으로 잰 차로 가운데
        (center_offset_m) 를 차로 좌표 l 로 바꿔 격자 중심과 비교 → m7_line_fix_frames 개 중앙값만큼 격자 phase 를 옮긴다."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        m = self._m7
        grid = m.get('grid')
        if grid is None or not msg.found or msg.yellow_side not in (-1, 1):
            return
        if abs(msg.lane_heading_rad) > math.radians(g('m7_line_fix_heading_deg')):
            return
        pose = self.pose_at(stamp_s(msg.header))
        if pose is None:
            return
        h = wrap(pose[2] - m['axis'])      # 차 방향 − 차로 방향 (왼쪽 +)
        if abs(h) > math.radians(g('m7_line_fix_heading_deg')):
            return
        W = g('m7_lane_width_m')
        sc_, lc = self.m7_frame(pose[:2])
        k = self.m7_k(round((lc - grid['phase']) / W))
        edge = {-1: 0, 1: g('m7_lanes') - 1}[msg.yellow_side]
        meas = lc + g('m7_line_fix_ahead_m') * math.sin(h) + msg.center_offset_m * math.cos(h)
        if k != edge:
            # 끝 차로가 아니라고 믿는데 그쪽 노란 바깥선만 보임 = 위치가 크게 틀림 (실차 11-06 full13: 왼쪽 노란 선 위에 있는데
            # 가운데 차로로 믿어 0.6 m 어긋남). 같은 값이 m7_line_relocate_frames 번 모이면 노란 선 쪽 끝 차로로 격자를 옮긴다.
            # 3차선 끝의 A/B 분리대 노란 선과 헷갈리지 않게 구간 앞쪽 (s < m7_line_relocate_max_s) 에서만.
            if not g('m7_line_relocate') or sc_ > g('m7_line_relocate_max_s'):
                return
            hd = math.radians(g('m7_line_relocate_heading_deg'))
            if abs(h) > hd or abs(msg.lane_heading_rad) > hd:
                return
            # 보인 노란 선의 l 위치 (lane_follower 의 차선 가운데는 그 노란 선에서 반 차로 안쪽). 이 선이 세 차로의 왼쪽 끝일 수도,
            # 오른쪽 끝일 수도 있다 — 덜 옮기는 쪽으로 본다. 차가 노란 선 밖으로 나가 있으면 왼쪽 끝 선이 차 오른쪽에 보인다
            # (2D 시뮬 11-08 a7: 왼쪽 선 밖에서 '오른쪽 노란 선만' → 오른쪽 끝으로 +53.5 cm 옮겨 벽에 부딪힘)
            line_l = meas + (W / 2 if msg.yellow_side < 0 else -W / 2)
            hi_b = self.m7_lane_l(0) + W / 2
            lo_b = self.m7_lane_l(g('m7_lanes') - 1) - W / 2
            err = min((line_l - hi_b, line_l - lo_b), key=abs)
            if abs(err) > g('m7_line_relocate_max_m'):
                return
            buf = m.setdefault('relocate', [])
            buf.append(err)
            n = g('m7_line_relocate_frames')
            if len(buf) < n:
                return
            vals = sorted(buf[-n:])
            buf.clear()
            if vals[-1] - vals[0] > g('m7_line_relocate_spread_m'):
                return                     # 측정이 흔들림 — 다른 선일 수 있음
            d = vals[len(vals) // 2]
            m['line_corr'] = m.get('line_corr', 0.0) + d
            m['fix_corr'] = 0.0            # 작은 보정 상한은 새 위치부터 다시 (안 그러면 상한이 방금 옮긴 것을 되돌림 — 실차 11-06 full14:
            #                                위치 다시 맞춤 직후 '+18.0 cm (합 −15.0)' 로 되돌아감, 2D 시뮬 11-08 도 같음)
            grid['phase'] += d
            m.setdefault('line_fix', []).clear()
            self.get_logger().warn(f'[M7] 노란 선으로 위치 다시 맞춤: 차로 {k} 로 믿었지만 {"왼" if msg.yellow_side < 0 else "오른"}쪽 '
                                   f'노란 바깥선만 보임 → 차로 {edge} 로, 격자 {d * 100:+.1f} cm 옮김 (s {sc_:.2f} m, {n} 장 중앙값, '
                                   f'odom {self.fmt(self._pos)})')
            return
        m.pop('relocate', None)
        err = meas - self.m7_lane_l(k)
        if abs(err) > g('m7_line_fix_gate_m'):
            return
        buf = m.setdefault('line_fix', [])
        buf.append(err)
        if len(buf) < g('m7_line_fix_frames'):
            return
        d = sorted(buf)[len(buf) // 2] * g('m7_line_fix_gain')
        buf.clear()
        total = m.get('fix_corr', 0.0)     # 작은 보정만의 합 (위치 다시 맞춤 제외)
        lim = g('m7_line_fix_max_m')
        d = max(-lim - total, min(lim - total, d))
        if abs(d) < 0.005:
            return
        m['fix_corr'] = total + d
        m['line_corr'] = m.get('line_corr', 0.0) + d
        grid['phase'] += d                 # 고정된 격자도 같이 (m7_grid 가 다시 계산할 때는 line_corr 를 더함)
        self.get_logger().info(f'[M7] 노란 선으로 차로 {k} 가운데 보정 {d * 100:+.1f} cm (합 {m["line_corr"] * 100:+.1f} cm, + = 왼쪽), '
                               f'odom {self.fmt(self._pos)}')

    def m7_lane_l(self, k):
        gr = self._m7['grid']
        return gr['phase'] + (gr['n_left'] - k) * self.get_parameter('m7_lane_width_m').value

    # ---------------------------------------------------------------- M7 운전: 차 사각형 쓸기 검사
    def ack(self):
        return self.get_parameter('drive_mode').value == 'ackermann'

    ACK_L = 0.2                            # LIMO 축간거리 — 아커만 원호는 뒷축 가운데가 그림 (base_link = 뒷축 + L/2 앞)
    WHEELS = ((0.1, 0.0875), (-0.1, 0.0875), (0.1, -0.0875), (-0.1, -0.0875))   # base_link 기준 바퀴 (앞왼, 뒤왼, 앞오, 뒤오)

    @classmethod
    def ack_poses(cls, s0, l0, p0, segs, step=0.025):
        """base_link (s0, l0, p0) 에서 [(곡률, 길이), ...] 원호를 차례로 갈 때 base_link 자세들 (N, 3) 과 거리.
        곡률·길이는 뒷축 가운데 궤적 기준 (실제 LIMO·sim2d ack_ref rear — 최대 조향에서 몸이 궤적보다 ~11° 안쪽을 봄,
        다른 세션 검토 11-09: 차 중심 모델이면 S 자 시작점·바퀴 검사가 시뮬과 어긋남)."""
        P, D = [], []
        h = cls.ACK_L / 2
        p = p0
        s, l = s0 - h * math.cos(p0), l0 - h * math.sin(p0)     # 뒷축 가운데
        dist = 0.0
        for kap, ln in segs:
            n = max(1, int(round(ln / step)))
            ds = ln / n
            for _ in range(n):
                if abs(kap) < 1e-6:
                    s, l = s + ds * math.cos(p), l + ds * math.sin(p)
                else:
                    p2 = p + kap * ds
                    s, l = s + (math.sin(p2) - math.sin(p)) / kap, l - (math.cos(p2) - math.cos(p)) / kap
                    p = p2
                dist += ds
                P.append((s + h * math.cos(p), l + h * math.sin(p), p))
                D.append(dist)
        return np.array(P), np.array(D)

    @classmethod
    def wheel_out(cls, P, walls):
        """자세마다: 같은 쪽 두 바퀴 (앞·뒤) 가 둘 다 선 밖 (walls 상자 안) 인가 — 규정 차로 이탈 ('한쪽 두 바퀴가 선 밖').
        몸이나 바퀴 하나가 넘는 건 감점 아님 (다른 세션 검토 11-09)."""
        if not walls or len(P) == 0:
            return np.zeros(len(P), bool)
        W_ = np.array(walls, float)
        c, s_ = np.cos(P[:, 2]), np.sin(P[:, 2])
        ins = []
        for wx, wy in cls.WHEELS:
            x = P[:, 0] + wx * c - wy * s_
            y = P[:, 1] + wx * s_ + wy * c
            ins.append(((x[:, None] > W_[None, :, 0]) & (x[:, None] < W_[None, :, 1])
                        & (y[:, None] > W_[None, :, 2]) & (y[:, None] < W_[None, :, 3])).any(axis=1))
        return (ins[0] & ins[1]) | (ins[2] & ins[3])

    def m7_ack_ref(self, sc, lc, psi, lt, boxes, mg, wheel_walls=()):
        """기준 S 자: 지금 자세에서 곧게 a 만큼 → 목표 쪽 원호 (반경 R) → 반대 원호로 방향 0 → 0.3 m 곧게, 끝 옆 = lt.
        a 를 0 부터 0.05 씩 늘려 안 닿는 가장 이른 것 (다른 세션 설계 11-09: 끼는 이유는 결정보다 '꺾기 시작' 이 늦어서 —
        지평선 0.95 m 비용만으로는 S 자 끝을 못 봐서 이른 시작을 안 고름). 지금 방향이 이미 목표 쪽이면 그대로 씀. 없으면 None."""
        g = lambda k_: self.get_parameter(k_).value   # noqa: E731
        R = g('ack_plan_radius_m')
        for a in np.arange(0.0, g('m7_ack_ref_max_a_m') + 1e-9, 0.05):
            l0 = lc + a * math.sin(psi)
            dl = lt - l0
            if abs(dl) < 0.02:
                segs = [(0.0, a), (0.0, 0.3)] if abs(psi) < 0.05 else None
                if segs is None:
                    continue
            else:
                d = 1.0 if dl > 0 else -1.0
                c = (math.cos(psi) + 1.0 - abs(dl) / R) / 2.0
                if not -1.0 <= c <= 1.0:
                    continue
                th = d * math.acos(c)
                l1 = R * abs(wrap(th - psi))
                l2 = R * abs(th)
                if d * (th - psi) < -1e-6:
                    continue                # 이미 목표 쪽으로 더 틀어짐 — 첫 원호가 반대로 돌아야 함 (이 모양 아님)
                segs = [(0.0, a), (d / R, l1), (-d / R, l2), (0.0, 0.3)]
            P, D = self.ack_poses(sc, lc, psi, [sg for sg in segs if sg[1] > 1e-6])
            if boxes and avoid.hits(P, boxes, mg).any():
                continue
            if wheel_walls and self.wheel_out(P, wheel_walls).any():
                continue
            return P, a
        return None

    def m7_ack_step(self, sc, lc, psi, lt, k, near, changing, funnel=()):
        """아커만 M7 한 번: 곡률 후보 (κ1, κ2) 마다 첫 원호 d1 → 둘째 원호 d2 를 굴려 보고 (매 순간 다시 계획 — 조향 지연은
        ack_plan_radius_m 여유로), 첫 원호가 안 닿는 것 중 비용 (끝 옆 오차·방향 오차·둘째 원호에서 막히는 길이) 이 가장 작은 것.
        돌려줌: v, w (= v·κ1), 막힘 여부, 막힘 로그."""
        g = lambda k_: self.get_parameter(k_).value   # noqa: E731
        kmax = 1.0 / g('ack_plan_radius_m')
        K = np.linspace(-kmax, kmax, int(g('m7_ack_n')))
        d1, d2 = g('m7_ack_d1_m'), g('m7_ack_d2_m')
        boxes = avoid.escape((sc, lc, psi), list(near)) or []   # 이미 닿은 상자 (기억 오차) 는 빼고 — 빠져나가는 길도 고르게
        # 이미 여유 (m7_margin_m) 안에 붙은 상자는 여유 0 — 일률 여유면 그 자리에서 어느 원호도 첫 자세부터 닿아 못 빠져나감
        # (2D 시뮬 11-09 e6: 목표 차로 옆 +0.43 까지 가서 모든 κ 빈 길이 0.00 → 후진 3 번 → 멈춤). 직선 검사 avoid.margins 와 같은 방식
        mg = avoid.margins((sc, lc, psi), boxes, g('m7_margin_m')) if boxes else np.zeros(0)
        pmax = math.radians(g('m7_ack_max_psi_deg'))
        ref = None
        wheel_walls = []
        out_walls = []
        if changing and g('m7_ack_ref'):
            m = self._m7
            key = (k, round(sc / 0.1))
            if m.get('ack_ref_key') != key:
                m['ack_ref_key'] = key
                fs = set(funnel)
                b2 = [b for b in boxes if b not in fs]
                latched = m.get('ack_all_until', -1.0) > sc and funnel
                m['ack_ref'] = None if latched else self.m7_ack_ref(sc, lc, psi, lt, boxes, mg)
                cross = False
                if m['ack_ref'] is None and funnel and not latched:
                    # ② 깔때기는 규정 (같은 쪽 두 바퀴가 선 밖) 으로만 — 몸·바퀴 하나는 넘어도 감점 아님
                    m['ack_ref'] = self.m7_ack_ref(sc, lc, psi, lt, b2, np.full(len(b2), g('m7_margin_m')), wheel_walls=list(funnel))
                    if m['ack_ref'] is not None:
                        cross = 'wheel'
                if m['ack_ref'] is None and funnel and (g('m7_ack_ref_cross_funnel') or latched):
                    # 깔때기 노란 선을 지키면 S 자가 안 들어감 (트랙: 깔때기 끝 x 8.05 → 첫 줄 x 7.85 = 0.2 m, 아커만 S 자 0.84~0.9 m) →
                    # 깔때기 벽만 빼고 (장애물·바깥 벽은 그대로) 다시. 선을 조금 넘을 수 있음 (차로 이탈 −1 위험 < 끼어서 M7 못 끝냄)
                    m['ack_ref'] = self.m7_ack_ref(sc, lc, psi, lt, b2, np.full(len(b2), g('m7_margin_m')))
                    cross = 'all' if m['ack_ref'] is not None else False
                    if cross and not latched:
                        # 한 번 ③ 으로 정하면 1 m (첫 줄을 지날 때까지) 유지 — 매 재계획 ①·② 가 될 듯 말 듯하면 선을 들락날락해
                        # 차로 이탈이 여러 번 (규정: 1 회당 −1) 될 수 있음 (다른 세션 제안 11-09)
                        m['ack_all_until'] = sc + g('m7_ack_all_latch_m')
                r_ = m['ack_ref']
                self.m7_mode(f'ref{k}{r_ is None}{cross}', f'기준 S 자 → 차로 {k}: ' + ('안 들어감 (반응형만)' if r_ is None else
                                                  f'곧게 {r_[1]:.2f} m 뒤 꺾기 시작' + {'wheel': ' (깔때기는 바퀴 규칙으로)',
                                                  'all': ' (깔때기 선 무시 — 차로 이탈 감점 위험)'}.get(cross, '')))
                m['ack_ref_cross'] = cross
            ref = m.get('ack_ref')
            if ref is not None and m.get('ack_ref_cross') and funnel:
                fs = set(funnel)              # 기준이 깔때기 몸 벽을 안 지키기로 했으면 후보 검사도 같게 (아니면 기준을 못 따라감)
                boxes = [b for b in boxes if b not in fs]
                mg = avoid.margins((sc, lc, psi), boxes, g('m7_margin_m')) if boxes else np.zeros(0)
                if m.get('ack_ref_cross') == 'wheel':
                    wheel_walls = list(funnel)
                else:
                    out_walls = list(funnel)  # ③: 두 바퀴 밖 구간이 짧은 쪽을 고르게 비용으로만
        best = None
        free1 = []
        for k1 in K:
            for k2 in K:
                P, D = self.ack_poses(sc, lc, psi, [(k1, d1), (k2, d2)])
                hit = avoid.hits(P, boxes, mg).any(axis=1) if boxes else np.zeros(len(P), bool)
                bad = hit | (np.abs(P[:, 2]) > pmax)
                if wheel_walls:
                    bad = bad | self.wheel_out(P, wheel_walls)
                fl = float(D[np.argmax(bad)] - D[0]) if bad.any() else float(D[-1])
                if k2 == K[len(K) // 2]:
                    free1.append((k1, fl))
                if fl < d1:
                    continue                    # 첫 원호에서 닿음 — 이 곡률로는 못 감
                se, le, pe = P[-1] if not bad.any() else P[max(0, np.argmax(bad) - 1)]
                # 투영 옆 오차: 끝 자세에서 최소 반경으로 펴면 옆으로 R·(1 − cos ψ) 더 감 — 끝 방향 자체를 벌하면 S 자 가운데
                # (방향이 클 수밖에 없음) 를 늘 손해로 봐서 '안 꺾는 쪽' 이 이김 (2D 시뮬 11-09 i1: +15° 로 들어와 왼쪽 차로로 가야 하는데
                # 방향만 펴다 첫 줄 LIMO 에 비스듬히 끼임, 다른 세션 검토)
                lp = le + math.copysign(g('ack_plan_radius_m') * (1.0 - math.cos(pe)), pe)
                if ref is not None:
                    # 기준 S 자 위 같은 s 의 옆·방향을 목표로 (기준이 '아직 안 꺾을 때' 와 '지금 꺾을 때' 를 정함)
                    RP = ref[0]
                    i_ = int(np.argmin(np.abs(RP[:, 0] - se)))
                    lr, pr = (RP[i_, 1], RP[i_, 2]) if se <= RP[-1, 0] else (lt, 0.0)
                    lpr = le + math.copysign(g('ack_plan_radius_m') * (1.0 - math.cos(pe - pr)), pe - pr)
                    cost_lat = g('m7_ack_w_lat') * abs(lpr - lr) + g('m7_ack_w_ref_head') * abs(pe - pr)
                else:
                    cost_lat = g('m7_ack_w_lat') * abs(lp - lt)
                if out_walls:
                    cost_lat += g('m7_ack_w_out') * float(self.wheel_out(P, out_walls).mean()) * (d1 + d2)
                cost = (cost_lat + g('m7_ack_w_head') * abs(pe)
                        + g('m7_ack_w_free') * (d1 + d2 - fl) + 0.05 * abs(k1 - k2))
                if best is None or cost < best[0]:
                    best = (cost, k1, k2, fl)
        if best is None:
            msg = '갈 원호 없음 (아커만, 첫 원호에서 닿음): ' + ', '.join(f'κ {k1:+.1f} {fl:.2f} m' for k1, fl in free1)
            return 0.0, 0.0, True, msg
        _, k1, k2, fl = best
        vmax = g('m7_change_speed') if (changing or abs(k1) > 0.3 * kmax) else g('m7_speed')
        v = min(vmax, g('m7_kv') * (fl - g('m7_stop_gap_m')))
        v = max(v, 0.04) if fl >= d1 else 0.0
        kind = 'straight' if abs(k1) < 0.15 * kmax else 'arc'
        self.m7_mode(f'{kind}{k}', f'{"직진" if kind == "straight" else "원호"} κ {k1:+.2f} → {k2:+.2f} (반경 '
                                   f'{"∞" if abs(k1) < 1e-6 else f"{1 / abs(k1):.2f}"} m, 빈 길이 {fl:.2f} m, 차로 {k})')
        return v, v * k1, False, None

    def m7_free(self, sc, lc, h, boxes, dmax, dirn=1.0):
        return avoid.free(sc, lc, h, boxes, dmax, self.get_parameter('m7_margin_m').value, dirn)

    def m7_rot_ok(self, sc, lc, psi, h, boxes):
        return avoid.rot_ok(sc, lc, psi, h, boxes, self.get_parameter('m7_rot_margin_m').value)

    def m7_lidar_clear(self, sc, lc, psi, wz):
        """뎁스로 기억한 장애물 상자를 라이다로 확인: 라이다 빔이 상자를 맞지 않고 뚫고 지나가면 '빔' 증거,
        상자 안에서 맞으면 '있음'. 빔 증거만 m7_lidar_clear_scans 번 이어지면 가짜 (유령) 로 보고 지운다.
        상자 앞에서 다른 것에 막힌 빔 (가려짐) 은 판단 안 함. 2D 시뮬 11-06: 같은 LIMO 가 0.3 m 앞으로 밀린 유령 상자로
        기억돼 옆 차로 틈을 막아 멈춤 — 라이다는 그 자리가 빈 것을 보고 있었다."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        raw = getattr(self, '_scan_raw', None)
        now = self.get_clock().now().nanoseconds * 1e-9
        if raw is None or now - raw[0] > 0.3 or raw[0] == self._m7.get('clear_t') or abs(wz) > g('m7_lidar_clear_max_w'):
            return
        self._m7['clear_t'] = raw[0]
        _, ang, rng = raw
        lx = g('laser_x')
        ls, ll = sc + math.cos(psi) * lx, lc + math.sin(psi) * lx
        sh, reach = g('m7_lidar_clear_shrink_m'), g('m7_lidar_clear_reach_m')
        drop = []
        for ob in self._obs:
            if ob.get('bottom', 0.0) > g('m7_lidar_z') - g('m7_lidar_clear_below_m'):
                if not ob.get('float_logged'):
                    ob['float_logged'] = True
                    b_ = self.m7_box(ob)
                    self.get_logger().info(f'[M7] 떠 있는 장애물 (뎁스 가장 낮은 점 {ob.get("bottom", 0.0):.2f} m) — 라이다로 안 지움: '
                                           f's {b_[0]:.2f}~{b_[1]:.2f} l {b_[2]:+.2f}~{b_[3]:+.2f}')
                continue                           # 바닥에서 떠 있는 것 (분리대 위 표지판 판면 등) — 라이다 빔이 밑으로 지나감
            s0, s1, l0, l1 = self.m7_box(ob)
            s0, s1, l0, l1 = s0 + sh, s1 - sh, l0 + sh, l1 - sh
            if s1 <= s0 or l1 <= l0:
                continue
            ds = max(s0 - ls, 0.0, ls - s1)
            dl = max(l0 - ll, 0.0, ll - l1)
            dn = math.hypot(ds, dl)
            if dn < 0.10 or dn > reach:
                continue
            corners = [(s0, l0), (s0, l1), (s1, l0), (s1, l1)]
            c0 = math.atan2((l0 + l1) / 2 - ll, (s0 + s1) / 2 - ls)
            rel = [wrap(math.atan2(cl - ll, cs - ls) - c0) for cs, cl in corners]
            lo, hi = c0 + min(rel), c0 + max(rel)
            # 이 상자를 지나는 빔 (차 기준 각도 = 차로 기준 각도 − psi)
            da = np.angle(np.exp(1j * (ang + psi - (lo + hi) / 2)))
            sel = np.abs(da) < (hi - lo) / 2 * 0.8          # 가장자리 빔은 빼고
            if sel.sum() < 3:
                continue
            hits = passes = 0
            for a_, r_ in zip((ang[sel] + psi).tolist(), rng[sel].tolist()):
                ca, sa = math.cos(a_), math.sin(a_)
                with np.errstate(divide='ignore', invalid='ignore'):
                    t1 = [(s0 - ls) / ca if ca else -math.inf, (s1 - ls) / ca if ca else math.inf]
                    t2 = [(l0 - ll) / sa if sa else -math.inf, (l1 - ll) / sa if sa else math.inf]
                tin = max(min(t1), min(t2))
                tout = min(max(t1), max(t2))
                if not (tin <= tout and tout > 0):
                    continue
                if r_ < tin - 0.03:
                    continue                       # 앞에서 다른 것에 막힘 — 판단 안 함
                if r_ <= tout + 0.05:
                    hits += 1
                else:
                    passes += 1
            if hits:
                ob['lidar_empty'] = 0
            elif passes >= 3:
                ob['lidar_empty'] = ob.get('lidar_empty', 0) + 1
                if ob['lidar_empty'] >= g('m7_lidar_clear_scans'):
                    drop.append((ob, (s0 - sh, s1 + sh, l0 - sh, l1 + sh), passes))
        for ob, b, n in drop:
            self._obs = [o for o in self._obs if o is not ob]
            self.get_logger().info(f'[M7] 라이다로 기억 지움: s {b[0]:.2f}~{b[1]:.2f} l {b[2]:+.2f}~{b[3]:+.2f} — 라이다 빔 {n} 개가 '
                                   f'맞지 않고 지나감 ({g("m7_lidar_clear_scans")} 번 연속, 관측 {len(ob["quads"])})')

    def m7_lidar_trim(self, boxes, sc, lc, psi):
        """뎁스로 기억한 상자 중 라이다에 가까운 것 (가까운 면이 m7_lidar_trim_reach_m 안) 은 차를 향한 면을 라이다 점까지 줄인다.
        실차 11-06 full11: 오른쪽 차로 LIMO 를 왼쪽으로 34 cm 넓게 기억 (라이다 앞면은 l −0.42~−0.55, 기억은 −0.06 까지) →
        가운데 차로가 막힌 것으로 보고 후진 반복. 라이다 점이 상자 근처에 3 개 이상 있을 때만 (없으면 가려졌을 수 있어 그대로).
        상자를 넓히지는 않는다 (뎁스 상자보다 라이다 점이 바깥이면 라이다 점 상자 (scan_boxes) 가 따로 막는다)."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        xy = self.scan_xy()
        if xy is None or len(xy[0]) == 0:
            return boxes
        c, s_ = math.cos(psi), math.sin(psi)
        ps = sc + xy[0] * c - xy[1] * s_
        pl = lc + xy[0] * s_ + xy[1] * c
        lx = g('laser_x')
        ls, ll = sc + c * lx, lc + s_ * lx
        reach, pad, keep = g('m7_lidar_trim_reach_m'), g('m7_lidar_trim_pad_m'), g('m7_lidar_trim_min_m')
        out = []
        trimmed = []
        for b in boxes:
            s0, s1, l0, l1 = b
            dn = math.hypot(max(s0 - ls, 0.0, ls - s1), max(l0 - ll, 0.0, ll - l1))
            if dn > reach:
                out.append(b)
                continue
            m = (ps > s0 - 0.15) & (ps < s1 + 0.15) & (pl > l0 - 0.15) & (pl < l1 + 0.15)
            if m.sum() < 3:
                out.append(b)
                continue
            qs, ql = ps[m], pl[m]
            n0, n1, m0, m1 = s0, s1, l0, l1
            if lc > l1 - 0.02:                     # 차가 상자 왼쪽 → 상자 왼쪽 면 (l1) 을 라이다 점까지
                m1 = max(min(l1, float(ql.max()) + pad), l0 + keep)
            elif lc < l0 + 0.02:                   # 차가 상자 오른쪽 → 오른쪽 면 (l0)
                m0 = min(max(l0, float(ql.min()) - pad), l1 - keep)
            if sc < s0:                            # 상자가 앞 → 앞면 (s0)
                n0 = min(max(s0, float(qs.min()) - pad), s1 - keep)
            nb = (n0, n1, m0, m1)
            if max(abs(nb[i] - b[i]) for i in range(4)) > 0.05:
                trimmed.append((b, nb))
            out.append(nb)
        if trimmed:
            key = tuple(round(v, 1) for b, nb in trimmed for v in nb)
            if key != self._m7.get('trim_key'):
                self._m7['trim_key'] = key
                self.get_logger().info('[M7] 라이다로 상자 줄임: ' + ', '.join(
                    f's {b[0]:.2f}~{b[1]:.2f} l {b[2]:+.2f}~{b[3]:+.2f} → s {nb[0]:.2f}~{nb[1]:.2f} l {nb[2]:+.2f}~{nb[3]:+.2f}'
                    for b, nb in trimmed))
        return out

    def scan_xy(self):
        """최근 라이다 점 (base_link x, y) — 0.3 s 넘게 끊겼으면 None."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._scan is None or now - self._scan[0] > 0.3:
            return None
        return self._scan[1], self._scan[2]

    def m7_publish(self, v, w):
        tw = Twist()
        tw.linear.x, tw.angular.z = float(v), float(w)
        self.drive_pub.publish(tw)

    def m7_mode(self, mode, msg):
        """운전 방식이 바뀔 때만 로그 (직진 / 대각 / 제자리 회전 / 막힘 / 후진 / 정렬)."""
        if self._m7.get('mode') != mode:
            self._m7['mode'] = mode
            self.get_logger().info(f'[M7] {msg}')

    def m7_back_start(self, now, reason):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        m = self._m7
        count = m['retry'].add(self._pos)
        m['mon'].reset()
        m['blocked'] = None
        if count > g('m7_retry_max') and not m.get('opposite'):
            # 같은 자리에서 다 실패 → 반대쪽 차로 (지금 위치 차로를 사이에 두고 목표의 반대편, 없으면 다른 옆 차로) 로 한 번
            sc, lc = self.m7_frame(self._pos)
            kh = self.m7_k(round((lc - m['grid']['phase']) / g('m7_lane_width_m')))
            k = self.m7_k(m['n'])
            opts = [j for j in ((2 * kh - k) if k != kh else None, kh - 1, kh + 1, kh)
                    if j is not None and 0 <= j < g('m7_lanes') and j != k]
            side = self.m2_side()
            if side is not None and g('m7_avoid_far_side'):
                far = g('m7_lanes') - 1 - (0 if side == 'A' else g('m7_lanes') - 1)
                opts.sort(key=lambda j: j == far)        # 반대쪽 끝 차로는 마지막 후보로
            if opts:
                m['failed'] = (k, sc)            # 이 차로는 이 자리 근처에서 다시 고르지 않음
                m['opposite'] = True
                m['n'] = m['grid']['n_left'] - opts[0]
                m['retry'].reset()
                count = m['retry'].add(self._pos)
                self.get_logger().warn(f'[M7] 같은 자리 재시도 {g("m7_retry_max")} 번 실패 ({reason}) → 반대쪽 차로 {opts[0]} '
                                       f'(가던 차로 {k}, 지금 위치 차로 {kh}) 로 다시 시도')
        if count > g('m7_retry_max'):
            m['gave_up'] = True
            self.get_logger().error(f'[M7] 같은 자리 ({self.fmt(m["retry"].at)}) 에서 {count - 1} 번 후진·재시도하고 반대쪽 차로도 '
                                    f'실패 ({reason}) — 멈춤. 장애물 배치·/perception/obstacle_markers 확인 필요 | {self.m7_dump()}')
            return
        rf = self.rear_free()
        # 같은 자리에서 다시 막히면 더 많이 물러남 (0.15 → 0.30 → 0.45 m) — 같은 거리만 물러나면 같은 판단·같은 자리로 돌아와
        # 앞뒤만 반복했다 (실차 11-06 full11: 두 상자 사이 틈으로 꺾을 거리가 모자람)
        back_m = g('m7_back_m') * (min(count, g('m7_back_grow_max')) if g('m7_back_grow') else 1)
        m['back'] = {'from': self._pos, 't': now, 'm': back_m}
        self.get_logger().warn(f'[M7] 막힘: {reason} → 후진 {back_m:.2f} m (같은 자리 {count}/{g("m7_retry_max")} 번째, '
                               f'뒤 라이다 빈 거리 {"없음(라이다 안 들어옴)" if rf is None else f"{rf:.2f} m"})')

    def m7_tick(self, now):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        m = self._m7
        self.m7_grid()
        sc, lc = self.m7_frame(self._pos)
        yaw = self.heading()
        psi = wrap(yaw - m['axis'])
        wz = self._imu[2] if self._imu is not None else self._yaw_rate
        sf = sc + CAR_HALF_L
        W = g('m7_lane_width_m')
        half_lane = W / 2 - g('m7_lane_margin_m')
        boxes = [self.m7_box(ob) for ob in self._obs]
        if g('m7_lidar_trim'):
            boxes = self.m7_lidar_trim(boxes, sc, lc, psi)
        near = [b for b in boxes if b[1] > sc - 0.8 and b[0] < sc + g('m7_check_m') + 0.5 and abs((b[2] + b[3]) / 2 - lc) < 1.3]
        # 라이다 점도 (범퍼 0.06 m 밖부터 정확) — 뎁스 기억은 가까이서 실제보다 멀게 잡힐 때가 있다 (11-02 시험: 17 cm)
        near += avoid.scan_boxes(self.scan_xy(), sc, lc, psi)
        if g('m7_drop_outside'):
            # 세 차로 밖 (맨 왼쪽 차로 왼쪽 끝 ~ 맨 오른쪽 차로 오른쪽 끝 밖) 에 중심이 있는 상자는 트랙 밖 벽·물건
            l_hi = self.m7_lane_l(0) + W / 2
            l_lo = self.m7_lane_l(g('m7_lanes') - 1) - W / 2
            inside = lambda b: l_lo <= (b[2] + b[3]) / 2 <= l_hi   # noqa: E731
            boxes = [b for b in boxes if inside(b)]
            near = [b for b in near if inside(b)]
        walls = []
        funnel = []                          # 입구 1차로·깔때기 가상 벽 (아커만 기준 S 자가 안 들어가면 이것만 빼고 다시 찾음)
        behind = []                          # 입구 벽 중 차 중심보다 완전히 뒤 조각 — 제자리 회전 검사에선 뺀다
        if g('m7_outer_wall'):
            # 세 차로 바깥 (노란 선 밖) 을 벽으로 — 쓸기 검사에서 노란 선 밖으로 나가는 방향을 막는다 (실차 11-06 full13: 노란 선 위로
            # 나가 갇힘). 차로를 고르는 데는 안 씀. 이미 벽 안에 들어가 있으면 escape 가 그 벽에서 멀어지는 방향만 허용
            hi = self.m7_lane_l(0) + W / 2 + g('m7_outer_wall_off_m')
            lo = self.m7_lane_l(g('m7_lanes') - 1) - W / 2 - g('m7_outer_wall_off_m')
            walls = [(sc - 2.0, sc + 2.0, hi, hi + 0.30), (sc - 2.0, sc + 2.0, lo - 0.30, lo)]
            near = near + walls   # 앞뒤 대칭 (벽 안에 들어가 있어도 벽에서 멀어지는 쪽으로는 나감)
        if g('m7_entry_walls'):
            # 3차선 입구 흰 가로선까지는 1차로 (노란 선 둘이 나란함) — 그 양쪽에 가상 벽을 세워 대각으로 꺾지 못하게.
            # 선을 지나면 깔때기·세 차로. 정렬 뒤 (차가 선과 나란) 차선 인식이 가로선을 보면 그 자리로, 못 보면 m7_entry_single_m 까지
            # (2D 시뮬 11-09 k1: 입구 앞 1차로 x 8.85 에서 45° 로 꺾어 노란 선 밟음, 실차 11-06 full2 같음)
            ln = self._lane
            if m.get('bar_s') is None and self._bar_pt is not None:
                bs_, bl_ = self.m7_frame(self._bar_pt)    # M7 전에 본 가로선 (odom 점)
                if -0.3 <= bs_ <= 1.2 and abs(bl_) <= 0.5:
                    m['bar_s'] = bs_
                    self.get_logger().info(f'[M7] 3차선 입구 흰 가로선 (켜지기 전에 봄): s {bs_:.2f} m — 그때까지 1차로, 뒤로 깔때기')
            if (m.get('bar_s') is None and ln is not None and ln.stop_line_dist_m > 0 and abs(psi) < math.radians(12.0)
                    and sc < 1.0 and m['align'] is None):
                m['bar_s'] = sc + (g('m7_cam_x') + ln.stop_line_dist_m) * math.cos(psi)
                self.get_logger().info(f'[M7] 3차선 입구 흰 가로선: s {m["bar_s"]:.2f} m (지금 s {sc:.2f}) — 그때까지 1차로, 뒤로 깔때기')
            end = m.get('bar_s')
            if end is None:
                end = g('m7_entry_single_m')
            fl = g('m7_entry_funnel_m')
            if sc < end + fl:
                l1 = self.m7_lane_l(int(g('m7_entry_lane')))
                ew = [(sc - 2.0, end, l1 + W / 2, l1 + W / 2 + 0.30), (sc - 2.0, end, l1 - W / 2 - 0.30, l1 - W / 2)]
                step = 0.05                        # 깔때기: 가로선 뒤 45° 로 벌어지는 노란 선을 계단 벽으로
                k = 0
                while k * step < fl:
                    s0, wid = end + k * step, W / 2 + (k + 1) * step * W / fl
                    ew += [(s0, s0 + step, l1 + wid, l1 + wid + 0.30), (s0, s0 + step, l1 - wid - 0.30, l1 - wid)]
                    k += 1
                near = near + ew
                walls = walls + ew
                funnel = ew
                # 차 중심보다 완전히 뒤에 있는 깔때기 조각은 제자리 회전 검사에서 뺀다 — 목표 차로 가운데까지 간 뒤 뒷모서리가
                # 지나온 깔때기 벽에 걸려 어느 쪽으로도 못 돌고 후진 (2D 시뮬 11-09 q2·q3: s 1.07 에서 '회전 막힘' 전 방향)
                behind = [b for b in ew if b[1] < sc]
        if g('m7_lidar_clear'):
            self.m7_lidar_clear(sc, lc, psi, wz)
        k_lim = lambda x: max(-g('m7_max_w'), min(g('m7_max_w'), x))   # noqa: E731

        def in_lane(b, k):
            lk = self.m7_lane_l(k)
            return b[3] > lk - half_lane and b[2] < lk + half_lane

        def ahead(k):                        # 차 앞 → 차로 k 의 가장 가까운 장애물 앞면 (뒷면이 아직 차 중심보다 앞인 것)
            d = [b[0] - sf for b in boxes if in_lane(b, k) and b[1] > sc]
            return min(d) if d else math.inf

        def window_free(j, s0, s1, skip=None):   # 차로 j 의 [s0, s1] 에 걸친 장애물이 없는가
            # skip: 지금 비켜 가려는 (내 차로를 막은) 장애물 — 옆 차로 선을 조금 넘었어도 (겹침 ≤ m7_skip_overlap_m) 그 차로를
            # 막은 것으로 치지 않는다. 실제로 닿는지는 쓸기 검사가 본다 (실차 11-06 full8: 가운데 LIMO 가 오른쪽 차로에
            # 5 cm 걸쳐 '오른쪽도 막힘' → 빈 오른쪽 차로로 못 감)
            lo, hi = self.m7_lane_l(j) - W / 2, self.m7_lane_l(j) + W / 2
            def blocks(b):
                # 어떤 장애물이든 차로 j 를 조금만 (≤ m7_skip_overlap_m) 걸치면 막은 것으로 안 침 — 옆 차로 LIMO 끝이 경계에 걸쳐
                # 가운데로 못 돌아오고 후진 반복 (실차 11-06 full10). 실제로 닿는지는 쓸기 검사가 본다
                if min(b[3], hi) - max(b[2], lo) <= g('m7_skip_overlap_m'):
                    return False
                return in_lane(b, j) and b[1] > s0 and b[0] < s1
            return not any(blocks(b) for b in boxes)

        def after(j, s0):                    # s0 뒤 차로 j 의 첫 장애물 앞면까지
            d = [b[0] - s0 for b in boxes if in_lane(b, j) and b[0] >= s0]
            return min(d) if d else math.inf

        if m['gave_up']:
            self.m7_publish(0.0, 0.0)
            return
        # 1) 들어가자마자 IMU 방향으로 차로 방향 정렬 (제자리) — 비스듬한 채로 차로를 판단하지 않게
        if m['align'] is not None and self.ack():
            self.get_logger().info(f'[M7] 아커만 — 제자리 정렬 생략 (방향 오차 {math.degrees(psi):+.1f}° 는 원호 경로로 맞춤)')
            m['align'] = None
        if m['align'] is not None:
            done = abs(psi) < math.radians(g('m7_align_deg')) and abs(wz) < 0.1
            late = now - m['align'] > g('m7_align_timeout_s')
            if done or late or not self.m7_rot_ok(sc, lc, psi, 0.0, [b for b in near if b not in behind]):
                self.get_logger().info(f'[M7] 차로 방향 정렬 {"끝" if done else "시간 초과" if late else "생략 (돌면 부딪힘)"}: '
                                       f'방향 오차 {math.degrees(psi):+.1f}° ({now - m["align"]:.1f} s, '
                                       f'{"IMU" if m.get("imu_off") is not None else "odom yaw"})')
                m['align'] = None
            else:
                self.m7_mode('align', f'차로 방향 정렬 시작: 방향 오차 {math.degrees(psi):+.1f}°')
                self.m7_publish(0.0, k_lim(-g('m7_k_heading') * psi - g('m7_kd_heading') * wz))
                return
        # 2) 후진 중 — 오도메트리로 m7_back_m, 뒤는 라이다 (+ 기억한 장애물) 로 확인
        if m['back'] is not None:
            bk = m['back']
            moved = self.dist_from(bk['from'])
            rf = self.rear_free()
            mem = self.m7_free(sc, lc, psi, near, 0.06, -1.0)
            bm = bk.get('m', g('m7_back_m'))
            why = ('끝' if moved >= bm else '시간 초과' if now - bk['t'] > g('m7_back_timeout_s') * bm / g('m7_back_m')
                   else f'뒤 라이다 {rf:.2f} m' if rf is not None and rf < g('m7_back_lidar_m')
                   else '뒤 기억 장애물' if mem < 0.06 else None)
            if why is None:
                self.m7_mode('back', '후진 중')
                w_b = 0.0
                if self.ack():
                    # 아커만: 곧게 후진하면 방향이 그대로라 같은 자리에 다시 낀다 (2D 시뮬 11-09 c3: 14° 틀어진 채 첫 줄 LIMO 와
                    # 바깥 벽 사이에 끼어 후진 3 번 모두 실패) → 후진하면서 차로 방향 쪽으로 편다 (후진이라 w 부호 = 방향 줄이는 쪽)
                    vb = g('m7_back_speed')
                    w_b = max(-vb / g('ack_plan_radius_m'), min(vb / g('ack_plan_radius_m'), -2.0 * vb * psi / 0.3))
                self.m7_publish(-g('m7_back_speed'), w_b)
                return
            m['back'] = None
            m['rechoose'] = True
            m['probe_req'] = bool(g('m7_probe')) and not self.ack()   # 탐색은 제자리에서 돌아 방향을 잡음 — 아커만은 생략
            self.get_logger().info(f'[M7] 후진 {why}: {moved:.2f} m 물러남 → '
                                   f'{"라이다로 가장 빈 방향 찾기" if m["probe_req"] else "차로 다시 고르기"}')
            self.m7_publish(0.0, 0.0)
            return
        # 2b) 후진 뒤 라이다 탐색 직진 — 기억 (뎁스) 은 빼고 라이다 점 + 세 차로 밖 가상 벽만 본다
        if m.get('probe_req') or m.get('probe'):
            lid = avoid.scan_boxes(self.scan_xy(), sc, lc, psi) + [b for b in walls if b not in behind]
            if m.pop('probe_req', False):
                stp, lim = math.radians(g('m7_probe_step_deg')), g('m7_probe_deg')
                res = []
                for i in range(-int(lim / g('m7_probe_step_deg')), int(lim / g('m7_probe_step_deg')) + 1):
                    h = i * stp
                    fr = self.m7_free(sc, lc, h, lid, 1.2)
                    if abs(wrap(h - psi)) >= math.radians(3.0) and not self.m7_rot_ok(sc, lc, psi, h, lid):
                        continue
                    res.append((h, fr))
                best = max(res, key=lambda r: (round(min(r[1], 1.2), 1), -abs(r[0])), default=None)
                if best is not None and best[1] >= g('m7_probe_min_m'):
                    m['probe'] = {'h': best[0], 'from': self._pos, 't': now}
                    self.get_logger().info(f'[M7] 라이다 탐색: 가장 빈 방향 {math.degrees(best[0]):+.0f}° ({best[1]:.2f} m 빔) 으로 '
                                           f'{g("m7_probe_m"):.2f} m 가 봄 | ' + ', '.join(f'{math.degrees(h):+.0f}° {fr:.2f}' for h, fr in res))
                else:
                    self.get_logger().info(f'[M7] 라이다 탐색: 충분히 빈 방향 없음 ({"없음" if best is None else f"최대 {best[1]:.2f} m"}) — 원래대로')
            pr = m.get('probe')
            if pr is not None:
                moved = self.dist_from(pr['from'])
                err = wrap(pr['h'] - psi)
                fr = self.m7_free(sc, lc, psi, lid, 0.6)
                end = ('다 감' if moved >= g('m7_probe_m') else '시간 초과' if now - pr['t'] > g('m7_probe_timeout_s')
                       else f'앞 {fr:.2f} m 막힘' if abs(err) < math.radians(5.0) and fr < g('m7_stop_gap_m') + 0.05 else None)
                if end is not None:
                    m['probe'] = None
                    m['rechoose'] = True
                    self.get_logger().info(f'[M7] 라이다 탐색 끝 ({end}): {moved:.2f} m → 차로 다시 고르기')
                else:
                    w = k_lim(g('m7_k_heading') * err - g('m7_kd_heading') * wz)
                    v = 0.0 if abs(err) > math.radians(8.0) else min(g('m7_change_speed'), g('m7_kv') * (fr - g('m7_stop_gap_m')))
                    v = 0.0 if v < 0.02 else v
                    self.m7_mode('probe', f'라이다 탐색 직진 {math.degrees(pr["h"]):+.0f}°')
                    why = m['mon'].update(now, self._pos, yaw, v, w)
                    if why is not None:
                        m['probe'] = None
                        self.m7_back_start(now, why)
                        self.m7_publish(0.0, 0.0)
                        return
                    self.m7_publish(v, w)
                    return
        # 3) 갈 차로 — 내 (목표) 차로가 m7_block_m 안에서 막히면. 변경 중이어도 막혔으면 (rechoose) 다시 고른다
        k = self.m7_k(m['n'])
        lt = self.m7_lane_l(k)
        changing = abs(lc - lt) > g('m7_lane_done_m')
        if m.pop('m2_flip', False) and changing:
            here = round((lc - m['grid']['phase']) / W)
            self.get_logger().info(f'[M7] 갈림길 표지판이 {self.m2_side()} 로 바뀜 — 차로 {k} 로 가던 것을 멈추고 '
                                   f'지금 위치 차로 {self.m7_k(here)} 에서 다시 고름')
            m['n'] = here
            m['rechoose'] = True
            k = self.m7_k(here)
            lt = self.m7_lane_l(k)
            changing = abs(lc - lt) > g('m7_lane_done_m')
        # 내 차로를 막은 장애물 = 앞면이 차 앞보다 앞인 것만. 이미 차 옆에 있는 (옆 차로에서 조금 걸친) 상자를 고르면 그 상자 기준
        # 창으로 옆 차로를 검사해 지나온 장애물에 걸려 '옆 차로도 막힘' 이 된다 (실차 11-06 full4: 왼쪽 LIMO 가 4 cm 걸쳐 멈춤).
        # 옆 상자는 쓸기 검사 (near) 에는 그대로 들어간다.
        own = [b for b in boxes if in_lane(b, k) and b[1] > sc and b[0] > sf - 0.05]
        o = min(own, key=lambda b: b[0]) if own else None
        a_own = o[0] - sf if o else math.inf
        rechoose = m.pop('rechoose', False)
        side = self.m2_side()
        goal = None if side is None else (0 if side == 'A' else g('m7_lanes') - 1)
        prefer = self.m2_prefer()
        if o is not None and a_own < g('m7_block_m') and (not changing or rechoose):
            here = round((lc - m['grid']['phase']) / W)
            kh = self.m7_k(here)
            # 후보: 막은 장애물을 지나가는 구간 [앞면 − window, 뒷면 + 0.15] 이 비어 있는 옆 차로, 변경 중이면 지금 위치 차로도
            cands = sorted({j for j in (k - 1, k + 1, kh)
                            if 0 <= j < g('m7_lanes') and j != k and window_free(j, o[0] - g('m7_enter_window_m'), o[1] + 0.15, skip=o)})
            jg = None if goal is None or goal == k else k + (1 if goal > k else -1)
            far = None if goal is None else g('m7_lanes') - 1 - goal
            fl = m.get('failed')
            near_fail = fl is not None and sc < fl[1] + g('m7_failed_keep_m')
            if near_fail:
                # 같은 자리에서 다 실패해 반대쪽으로 다시 시도 중 — 방금 실패한 차로로 되돌아가지 않고, 반대쪽 끝 차로도 허용
                # (2D 시뮬 11-06 스트레스: 차로 1 로 보낸 뒤 '갈림길 쪽 먼저' 가 실패한 차로 0 으로 되돌려 멈춤 → 갈림길보다 멈추지 않는 게 먼저)
                cands = [j for j in cands if j != fl[0]]
            elif g('m7_avoid_far_side') and far in cands and len(cands) > 1:
                cands = [j for j in cands if j != far]   # M2: 표지판 반대쪽 끝 차로는 다른 길이 없을 때만 (그 차로로 끝나면 반대 길)
            if cands:
                if jg in cands and not near_fail:
                    j = jg                   # M2: 갈 수 있으면 갈림길 쪽
                else:
                    pref = 1 if prefer == 'left' else -1
                    j = max(cands, key=lambda j: (round(min(after(j, o[1] + 0.15), 9.0), 1), -abs(j - kh), pref * (k - j)))
                aj = after(j, o[1] + 0.15)
                m['n'], m['changes'] = m['grid']['n_left'] - j, m['changes'] + 1
                self.get_logger().info(f'[M7] 차로 {k} → {j}{" (다시 고름)" if rechoose else ""}: 내 차로 앞 {a_own:.2f} m 막힘, '
                                       f'차로 {j} 는 그 장애물 지나고 {"비어 있음" if aj == math.inf else f"{aj:.2f} m 비어 있음"} (s {sc:.2f} m)')
                k, lt, changing = j, self.m7_lane_l(j), True
            elif rechoose:
                self.get_logger().info(f'[M7] 다시 골랐지만 옆 차로가 막은 장애물 옆에서 비어 있지 않음 — 차로 {k} 유지 (s {sc:.2f} m) | {self.m7_dump()}')
        ahead_any = any(ahead(j) < math.inf for j in range(g('m7_lanes')))
        if not changing and goal is not None and k != goal and ahead_any:
            # M2: 갈림길 쪽 옆 차로가 지금 차로만큼 비어 있으면 미리 그쪽으로. 앞에 장애물이 없으면 (분리대 옆) 안 옮김
            j = k + (1 if goal > k else -1)
            aj = ahead(j)
            if window_free(j, sc - g('m7_enter_window_m'), sf + 0.6) and aj >= min(a_own, g('m7_block_m')):
                m['n'], m['changes'] = m['grid']['n_left'] - j, m['changes'] + 1
                self.get_logger().info(f'[M7] 차로 {k} → {j}: 갈림길 {side} 쪽 (차로 {goal}), 내 차로 앞 '
                                       f'{"비어 있음" if a_own == math.inf else f"{a_own:.2f} m"}, 차로 {j} 앞 '
                                       f'{"비어 있음" if aj == math.inf else f"{aj:.2f} m"} (s {sc:.2f} m)')
                k, lt, changing = j, self.m7_lane_l(j), True
        blk_msg = None
        if self.ack():
            # 4a) 아커만: 원호 두 개 (첫 원호 → 둘째 원호) 후보 중 안 닿고 목표 차로 가운데·차로 방향에 가장 가까워지는 것
            v, w, blocked, blk_msg = self.m7_ack_step(sc, lc, psi, lt, k, near, changing, funnel)
        else:
            # 4) 방향·속도: 바라는 방향 (pure pursuit) 에 가까운 후보 중 쓸기 검사로 m7_go_free_m 이상 갈 수 있는 것.
            #    없으면 제자리 회전 (v = 0) 으로 더 멀리 갈 수 있는 방향, 그것도 없으면 막힘
            cap = math.radians(g('m7_pp_heading_deg'))
            psi_pp = max(-cap, min(cap, math.atan2(lt - lc, g('m7_lookahead_m'))))
            cands = [psi_pp, 0.0, psi]           # 지금 방향 그대로 (회전 없이) 도 후보 — 돌 수 없는 자리에서 빠져나가게
            if changing:
                sig = 1.0 if lt > lc else -1.0
                stp = math.radians(g('m7_heading_step_deg'))
                cands += [sig * i * stp for i in range(1, int(g('m7_max_heading_deg') / g('m7_heading_step_deg')) + 1)]
            res = []
            near_rot = [b for b in near if b not in behind]
            for h in sorted(set(round(c, 4) for c in cands)):
                dm = g('m7_check_m')
                if changing and abs(h) > 0.09:
                    dm = min(dm, abs(lt - lc) / math.sin(abs(h)) + 0.25)     # 목표 차로까지만 본다 (그 너머 차로는 상관없음)
                fr = self.m7_free(sc, lc, h, near, dm)
                rok = abs(wrap(h - psi)) < math.radians(3.0) or self.m7_rot_ok(sc, lc, psi, h, near_rot)
                res.append((h, fr, rok, dm))
            def reach_ok(r):
                # 차로 변경 중 비스듬한 방향은 목표 차로 (가운데 − m7_reach_slack_m) 까지 끝까지 비어 있을 때만 — 조금만 비어도 꺾으면
                # 옆 LIMO 옆면에 비스듬히 붙어 갇힌다 (실차 11-06 full2: +45° 0.36 m 빔 → 가운데 LIMO 옆에서 후진 반복).
                # 못 가면 지금 방향 (0°) 으로 직진하다가 옆이 열리면 꺾는다.
                if not changing or abs(r[0]) <= 0.09:
                    return True
                return r[1] * math.sin(abs(r[0])) >= abs(lt - lc) - g('m7_reach_slack_m')
            go = [r for r in res if r[2] and r[1] >= min(g('m7_go_free_m'), r[3] - 0.01) and reach_ok(r)]
            v = w = 0.0
            if go:
                h, fr, _, dm = min(go, key=lambda r: (round(abs(wrap(r[0] - psi_pp)), 3), -r[1]))
                eff = g('m7_check_m') if (dm < g('m7_check_m') and fr >= dm - 1e-6) else fr
                # 차는 돌면서 지금 향한 방향으로도 나간다 → 지금 방향의 빈 거리로도 제한 (안 하면 비스듬한 채 모서리로 다가간다)
                fr_now = self.m7_free(sc, lc, psi, near, g('m7_check_m'))
                eff = min(eff, fr_now)
                vmax = g('m7_speed') if abs(h) < math.radians(10.0) else g('m7_change_speed')
                v = min(vmax, g('m7_kv') * (eff - g('m7_stop_gap_m')))
                v = 0.0 if v < 0.02 else v
                turn_first = abs(wrap(h - psi)) > math.radians(g('m7_turn_first_deg'))
                if v == 0.0 and abs(wrap(h - psi)) >= math.radians(3.0):
                    turn_first = True              # 지금 방향으로는 못 가지만 고른 방향으로 돌 수는 있음 → 제자리에서 돈다
                if turn_first:
                    v = 0.0
                w = k_lim(g('m7_k_heading') * wrap(h - psi) - g('m7_kd_heading') * wz)
                kind = 'rotate' if turn_first else 'straight' if abs(h) < math.radians(10.0) else 'diag'
                self.m7_mode(kind, {'rotate': f'제자리 회전 → {math.degrees(h):+.0f}° (그 방향 {fr:.2f} m 빔)',
                                    'straight': f'직진 (앞 {fr:.2f} m 빔, 차로 {k})',
                                    'diag': f'대각 {math.degrees(h):+.0f}° 로 차로 {k} 쪽 (그 방향 {fr:.2f} m 빔)'}[kind])
                blocked = v == 0.0 and not turn_first
            else:
                cur = max((r[1] for r in res if abs(wrap(r[0] - psi)) < math.radians(3.0)), default=0.0)
                rots = [r for r in res if r[2] and abs(wrap(r[0] - psi)) >= math.radians(3.0) and reach_ok(r)]
                best = max(rots, key=lambda r: r[1], default=None)
                if best is not None and best[1] > max(cur, 0.0) + 0.05:
                    w = k_lim(g('m7_k_heading') * wrap(best[0] - psi) - g('m7_kd_heading') * wz)
                    self.m7_mode('rotate', f'제자리 회전 → {math.degrees(best[0]):+.0f}° (그 방향 {best[1]:.2f} m, 지금 방향 {cur:.2f} m)')
                    blocked = False
                else:
                    blocked = True
        if blocked:
            v = w = 0.0
            self.m7_mode('blocked', blk_msg or ('갈 방향 없음 (쓸기 검사): ' + ', '.join(f'{math.degrees(r[0]):+.0f}° {r[1]:.2f} m'
                                                                       + ('' if r[2] else ' (회전 막힘)') for r in res)))
            if m['blocked'] is None:
                m['blocked'] = now
            elif now - m['blocked'] > 2 * g('m7_blocked_s'):
                self.m7_back_start(now, f'{2 * g("m7_blocked_s"):.0f} s 동안 갈 방향 없음')
                self.m7_publish(0.0, 0.0)
                return
            elif now - m['blocked'] > g('m7_blocked_s') and not m.get('rechosen'):
                m['rechoose'] = m['rechosen'] = True
        else:
            m['blocked'] = None
            m['rechosen'] = False
        # 5) 명령은 움직이라는데 몸이 안 움직임 (밀고 있음) → 바로 전진을 멈추고 후진
        why = m['mon'].update(now, self._pos, yaw, v, w)
        if why is not None:
            self.m7_back_start(now, why)
            self.m7_publish(0.0, 0.0)
            return
        self.m7_publish(v, w)
        # 끝: 변경이 끝났고 내 차로 앞이 비었고, 다른 차로에도 가까운 장애물이 없을 때 (또는 안전장치 거리)
        clear = all(ahead(j) > g('m7_clear_m') for j in range(g('m7_lanes')))
        if (g('m7_final_to_goal') and not changing and clear and sc > 0.3 and goal is not None and abs(k - goal) == 1
                and not m.get('final_shift') and not self._m2['locked'] and window_free(goal, sc - g('m7_enter_window_m'), sf + 0.6)):
            # M2: 장애물을 다 지났는데 목표 차로 한 칸 옆 (= 가운데 차로, 분리대 끝으로 가는 차로) → 끝내기 전에 갈림길 쪽으로.
            #     분리대 위 표지판이 가까우면 (M2 잠금: 박스 m2_lock_box_px 이상) 분리대 옆이라 안 옮김
            m['final_shift'] = True
            m['n'], m['changes'] = m['grid']['n_left'] - goal, m['changes'] + 1
            self.get_logger().info(f'[M7] 장애물 다 지남 — 갈림길 {side} 쪽 차로 {goal} 로 옮긴 뒤 끝냄 (지금 차로 {k}, s {sc:.2f} m)')
            return
        # 최소 구간: 2D 시뮬 11-09 r9 — 오른쪽 차로가 1~3 째 줄까지 비어 s 1.29 m 에서 '앞 1.0 m 빔' 으로 끝냄 → 차선 추종으로
        # 4 째 줄 (같은 차로 2.0 m 앞, 뎁스엔 보였음) 에 부딪힘. 마지막 줄은 s 3.5 m 안 → 3.0 m 부터는 그 줄이 m7_clear_m 안에 들어옴
        if (not changing and a_own > g('m7_clear_m') and clear and sc > max(0.3, g('m7_min_exit_m'))) or sc > g('m7_max_m'):
            self._reset_until = now + 0.5
            self._m7_hits = 0
            self.set_state('LANE', f'앞 {g("m7_clear_m")} m 비어 있음, 차로 변경 {m["changes"]} 번, 지금 차로 {k}, '
                                   f'구간 {sc:.2f} m{" (안전장치 거리)" if sc > g("m7_max_m") else ""} → 슬라이딩 윈도우 리셋')
            self._m7_done = True
            self.m2_commit(k)

    def m2_commit(self, k):
        """M7 이 끝난 차로 = 들어간 갈림길. 결과를 확정하고 다음 리셋까지 표지판을 더 보지 않는다."""
        if self._m2['done']:
            return                         # 이번 주행의 갈림길은 이미 지남 (다음 바퀴 3차선 등)
        side, last = self._m2['side'], self.get_parameter('m7_lanes').value - 1
        road = {0: 'A (왼쪽)', last: 'B (오른쪽)'}.get(k, '가운데 (분리대 쪽)')
        if side is None:
            self.get_logger().warn(f'[M2] 갈림길 표지판을 못 봄 — m7_prefer ({self.get_parameter("m7_prefer").value}) 쪽, '
                                   f'{road} 길로 들어감 (차로 {k})')
        elif k == (0 if side == 'A' else last):
            self.get_logger().info(f'[M2] 표지판 {side} → {road} 길로 들어감 (차로 {k}), odom {self.fmt(self._pos)}')
        else:
            self.get_logger().warn(f'[M2] 표지판 {side} 인데 {road} 길로 들어감 (차로 {k}) — 갈림길 쪽 차로로 옮기지 못함')
        self._m2['done'] = True

    # ---------------------------------------------------------------- M2 갈림길 — M7 없이 (m2_without_m7)
    def m2_alone(self):
        """M7 을 건너뛴 연습 (정적 장애물 없음) 에서 M2 를 따로 할 때인지."""
        return ('M7' in self.skip and self.get_parameter('m2_without_m7').value and not self._m2['done']
                and self.done_or_skipped('M3') and self._pos is not None)

    def m2_zone_update(self, msg):
        """3차선 구간 알아차리기 (노란 선이 한쪽만 m2_zone_frames 연속) + 표지판 없이 지나간 경우 마무리."""
        if not self.m2_alone():
            return
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        z = self._m2z
        ys = msg.yellow_side if msg.found else 0
        if ys in (-1, 1):
            z['single'] += 1
            z['side_obs'] = ys
            if z['zone'] is not None and z['shift'] is None and 0.0 < msg.line_resid_px < g('m6_resid_off'):
                pose = self.pose_at(stamp_s(msg.header))
                if pose is not None:
                    z['dirs'].append(wrap(pose[2] + msg.lane_heading_rad))
        else:
            z['single'] = 0
        if z['zone'] is None and z['single'] >= g('m2_zone_frames') and self.state == 'LANE':
            z['zone'] = self._pos
            self.get_logger().info(f'[M2] 3차선 구간 (노란 선 {"왼쪽" if ys < 0 else "오른쪽"}만 {z["single"]} 장 연속) — 지금 '
                                   f'{"맨 왼쪽" if ys < 0 else "맨 오른쪽"} 차로, 표지판 {self._m2["side"] or "아직 못 봄"}, '
                                   f'odom {self.fmt(self._pos)}')
        z['both'] = z['both'] + 1 if (z['zone'] is not None and ys == 2) else 0
        if (z['zone'] is not None and z['shift'] is None and z['hold'] is None and z['both'] >= 15
                and self.dist_from(z['zone']) > 3.0):
            k = 0 if z['side_obs'] < 0 else g('m7_lanes') - 1
            self.m2_commit(k)              # 표지판을 끝내 못 봄 — 들어간 길만 기록

    def m2_tick(self, now, cmd):
        """LANE·M2_SHIFT 에서 매 주기: 표지판 쪽 바깥 차로로 옮기기 → 그쪽 선만 따라가기 → 갈림길 지나면 끝."""
        if not self.m2_alone():
            return
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        z, m = self._m2z, self._m2
        last = g('m7_lanes') - 1
        if m['side'] and z['seen_at'] is None:
            z['seen_at'] = self._pos
        ready = (z['zone'] is not None and z['seen_at'] is not None
                 and min(self.dist_from(z['zone']), self.dist_from(z['seen_at'])) >= g('m2_shift_delay_m')
                 and len(z['dirs']) >= g('m2_axis_frames'))
        if self.state == 'LANE' and ready and z['shift'] is None and z['hold'] is None and m['side']:
            cur = 0 if z['side_obs'] < 0 else last
            tgt = 0 if m['side'] == 'A' else last
            if cur == tgt:
                z['hold'] = {'from': self._pos, 'side': m['side']}
                self.get_logger().info(f'[M2] 표지판 {m["side"]} — 이미 {"맨 왼쪽" if tgt == 0 else "맨 오른쪽"} 차로, '
                                       f'갈림길까지 {"왼쪽" if tgt == 0 else "오른쪽"} 선만 따라감')
            else:
                recent = list(z['dirs'])[-int(g('m2_axis_frames')):]
                axis = math.atan2(sum(math.sin(a) for a in recent), sum(math.cos(a) for a in recent))
                spread = max(abs(wrap(a - axis)) for a in recent)
                if spread > math.radians(15.0) or abs(wrap(self._yaw - axis)) > math.radians(20.0):
                    return                 # 차선 방향이 아직 흔들림 (코너 끝) — 다음 주기에
                dl = (cur - tgt) * g('m7_lane_width_m')       # + = 왼쪽 (차로 번호는 왼쪽이 0)
                z['shift'] = {'origin': self._pos, 'axis': axis, 'dl': dl, 'side': m['side']}
                self.set_state('M2_SHIFT', f'표지판 {m["side"]} → 차로 {cur} 에서 {tgt} 로 {abs(dl):.2f} m '
                                           f'{"왼쪽" if dl > 0 else "오른쪽"}, 차로 방향 {math.degrees(axis):.1f}° '
                                           f'(3차선 차선 방향 {len(recent)} 장, 흩어짐 ±{math.degrees(spread):.1f}°), odom {self.fmt(self._pos)}')
        if self.state == 'M2_SHIFT':
            sh = z['shift']
            c, sn = math.cos(sh['axis']), math.sin(sh['axis'])
            dx, dy = self._pos[0] - sh['origin'][0], self._pos[1] - sh['origin'][1]
            sc, lc = dx * c + dy * sn, -dx * sn + dy * c
            psi = wrap(self._yaw - sh['axis'])
            err = sh['dl'] - lc
            if abs(err) < g('m2_shift_tol_m') and abs(psi) < math.radians(12.0):   # 남은 방향은 차선 추종이 맞춤
                self.m7_publish(0.0, 0.0)
                z['hold'] = {'from': self._pos, 'side': sh['side']}
                self._reset_until = now + 0.5
                self.set_state('LANE', f'차로 옮기기 끝 ({sc:.2f} m 가서 옆 {lc:+.2f} m), '
                                       f'갈림길까지 {"왼쪽" if sh["side"] == "A" else "오른쪽"} 선만 따라감 → 슬라이딩 윈도우 리셋')
                return
            if sc > g('m2_shift_max_m') or abs(psi) > math.radians(g('m2_shift_abort_deg')):
                self.m7_publish(0.0, 0.0)
                z['hold'] = {'from': self._pos, 'side': sh['side']}
                self._reset_until = now + 0.5
                self.get_logger().warn(f'[M2] 차로 옮기기 포기: {sc:.2f} m 감, 옆 {lc:+.2f} / 목표 {sh["dl"]:+.2f} m, '
                                       f'차로 방향과 {math.degrees(psi):+.0f}°')
                self.set_state('LANE', '차로 옮기기 포기 → 차선 추종')
                return
            L = g('m2_shift_look_m')
            v = g('m2_shift_speed')
            aim = max(-math.radians(30.0), min(math.radians(30.0), math.atan2(err, L)))   # 차로 방향과 30° 넘게 비틀지 않음
            alpha = wrap(aim - psi)
            w = max(-1.0, min(1.0, 2.0 * v * math.sin(alpha) / L))
            self.m7_publish(v, w)
            return
        if z['hold'] is not None and self.state in ('LANE',) + SLOW_STATES:
            h = z['hold']
            cmd.follow = 'left' if h['side'] == 'A' else 'right'
            lock = m.get('lock_pos')
            # 끝: 잠금 (분기점 가까움) 뒤 m2_hold_m, 또는 잠금 뒤 노란 선이 양쪽 다 보임 (갈림길 안 한 차로 도로에 들어옴), 또는 안전 거리
            both = self._lane is not None and self._lane.found and self._lane.yellow_side == 2
            h['both'] = h.get('both', 0) + 1 if (lock is not None and both) else 0
            if ((lock is not None and (self.dist_from(lock) > g('m2_hold_m') or h['both'] >= 10))
                    or self.dist_from(h['from']) > g('m2_hold_max_m')):
                k = 0 if h['side'] == 'A' else last
                self.m2_commit(k)

    # ---------------------------------------------------------------- M8 라바콘 (Follow-the-Gap, 라이다)
    def on_cones(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self._cones = [(p.position.x, p.position.y) for p in msg.poses]
        if (self.state not in ('LANE',) + SLOW_STATES or self._m8_done or self._pos is None or 'M8' in self.skip
                or (g('m8_require_m2') and not self.done_or_skipped('M2'))):
            self._m8_hits = 0
            return
        ahead = [(x, y) for x, y in self.m8_car() if 0.0 < x < g('m8_trigger_m') and abs(y) < 0.8]
        both = any(y > 0 for _, y in ahead) and any(y < 0 for _, y in ahead)
        self._m8_hits = self._m8_hits + 1 if (len(ahead) >= g('m8_trigger_count') and both) else 0
        if self._m8_hits >= g('m8_confirm_frames'):
            now = self.get_clock().now().nanoseconds * 1e-9
            self._m8 = {'start': self._pos, 't0': now, 'mode': None, 'back': None, 'target': None, 'forced': False,
                        'mon': StuckMonitor(g('m8_stuck_check_s'), g('m7_stuck_move_m'), g('m7_stuck_rot_deg')),
                        'retry': RetryCounter(g('m7_retry_radius_m')), 'backs': 0, 'slow_n': 0, 'n': 0, 'seen': [], 'exit': None,
                        'lane_dir': self.lane_axis()[0]}
            self.set_state('M8_AVOID', f'앞 {g("m8_trigger_m")} m 안 라바콘 {len(ahead)} 개 (양쪽), {self._m8_hits} 번 연속, '
                                       f'odom {self.fmt(self._pos)}')

    def m8_chain_kw(self):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        return dict(step=(g('m8_chain_step_min_m'), 0.6), mu=g('m8_chain_mu'))

    def m8_car(self):
        """기억한 라바콘 (odom) → 차 좌표 (x 앞, y 왼쪽)."""
        if self._pos is None:
            return []
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        return [((x - self._pos[0]) * c + (y - self._pos[1]) * s, -(x - self._pos[0]) * s + (y - self._pos[1]) * c)
                for x, y in self._cones]

    def m8_mode(self, mode, msg):
        if self._m8.get('mode') != mode:
            self._m8['mode'] = mode
            self.get_logger().info(f'[M8] {msg}')

    def m8_finish(self, now, why):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        m = self._m8
        self._reset_until = now + 0.5
        self._m8_slow_until = now + g('m8_exit_slow_s')
        self._m8_done = True
        self.m7_publish(0.0, 0.0)
        self.set_state('LANE', f'라바콘 통과: {now - m["t0"]:.1f} s, {self.dist_from(m["start"]):.2f} m, 후진 {m["backs"]} 번, '
                               f'느리게 {m["slow_n"]} 틱, {why} → 슬라이딩 윈도우 리셋, {g("m8_exit_slow_s"):.1f} s 동안 '
                               f'{g("m8_exit_speed")} m/s, 이후 라바콘 무시')

    def m8_axis(self):
        """통로 출구선: (마지막 쌍 가운데 (odom), 방향 = 처음 쌍 가운데 → 마지막 쌍 가운데). 본 라바콘을 주축 (PCA) 으로 줄 세워
        앞뒤 두 개씩. 끝 쌍이 비스듬하면 (S) 마지막 틈 뒤로 그 방향으로 곧게 가 차로 가운데에서 20 cm 벗어난 채 커브를 만나
        차선 추종이 가끔 바깥 선을 따라 크게 돌았다 → 이 선 위로 나오게. 라바콘 4 개 미만이면 None."""
        m = self._m8
        if len(m['seen']) < 4:
            return None
        P = np.array(m['seen'])
        u = np.linalg.eigh(np.cov((P - P.mean(axis=0)).T))[1][:, -1]
        order = np.argsort(P @ u)
        first, last = P[order[:2]].mean(axis=0), P[order[-2:]].mean(axis=0)
        d = last - first
        if d[0] * math.cos(self._yaw) + d[1] * math.sin(self._yaw) < 0:
            first, last, d = last, first, -d
        ld = m.get('lane_dir')
        if self.get_parameter('m8_exit_lane').value and ld is not None:
            # 차로 가운데선: 첫 쌍 가운데 (차선 추종으로 차로 가운데에서 들어온 곳) + 들어올 때 차로 방향
            dd = math.atan2(d[1], d[0])
            if math.cos(wrap(ld - dd)) < 0:
                ld = wrap(ld + math.pi)
            if abs(wrap(ld - dd)) <= math.radians(self.get_parameter('m8_exit_lane_max_deg').value):
                return (float(first[0]), float(first[1])), ld
            # 들어올 때 차로 방향이 본 라바콘 줄과 크게 다름 = 코너 끝이라 90° 단위 맞춤이 옆 축으로 감 → 라바콘 줄로
            # (2D 시뮬 11-09 t2: −108° 로 잡아 차를 90° 돌려 트랙을 거꾸로 달려 3차선 LIMO 에 부딪힘)
            if not m.get('ld_warned'):
                m['ld_warned'] = True
                self.get_logger().warn(f'[M8] 들어올 때 차로 방향이 라바콘 줄과 {math.degrees(abs(wrap(ld - dd))):.0f}° 다름 — 라바콘 줄 방향을 씀')
        return (float(last[0]), float(last[1])), math.atan2(d[1], d[0])

    def m8_axis_aim(self, mid, hd, look):
        """출구선 위 (차 투영점 + look) 점을 차 좌표로, 그리고 옆 오차 (선이 차 왼쪽이면 +)."""
        px, py = self._pos[0] - mid[0], self._pos[1] - mid[1]
        along, lat = px * math.cos(hd) + py * math.sin(hd), px * math.sin(hd) - py * math.cos(hd)
        a = along + look
        tx, ty = mid[0] + a * math.cos(hd) - self._pos[0], mid[1] + a * math.sin(hd) - self._pos[1]
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        return (tx * c + ty * s, -tx * s + ty * c), along, lat

    def m8_exit(self, now):
        """마지막 라바콘 뒤: 통로 출구선 위 앞 점을 따라가 방향 오차 < m8_exit_deg, 옆 오차 < m8_exit_lat_m 이면 끝
        (m8_exit_s 넘으면 그냥 끝). 크게 틀어졌으면 제자리에서 돈다."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        ex = self._m8['exit']
        herr = wrap(ex['dir'] - self._yaw)
        (ax_, ay_), _, lat = self.m8_axis_aim(ex['mid'], ex['dir'], g('m8_exit_look_m'))
        t_lim = g('m8_exit_s') * (2.0 if self.ack() else 1.0)   # 아커만은 앞으로 가며 맞추므로 더 걸림
        if (abs(herr) < math.radians(g('m8_exit_deg')) and abs(lat) < g('m8_exit_lat_m')) or now - ex['t'] > t_lim:
            self.m8_finish(now, f'통로 출구선에 맞춤 (방향 {math.degrees(herr):+.0f}°, 옆 {lat * 100:+.1f} cm, {now - ex["t"]:.1f} s)')
            return
        h = math.atan2(ay_, ax_)
        wz = self._imu[2] if self._imu is not None else self._yaw_rate
        v = g('m8_exit_speed') * 0.5 if abs(h) < math.radians(15.0) else 0.0
        w = max(-g('m7_max_w'), min(g('m7_max_w'), g('m7_k_heading') * h - g('m7_kd_heading') * wz))
        if self.ack():
            # 아커만: 제자리에서 못 돎 (2D 시뮬 11-09 k2·k7: −31~−38° 틀어진 채 3 s 서 있다 끝 → 코너에서 차선을 놓쳐 트랙 밖) →
            # 출구선 위 점을 pure pursuit 로 앞으로 가며 맞춤 (곡률 ≤ 1/최소 반경)
            v = g('m8_exit_speed') * 0.5
            kap = 2.0 * ay_ / max(1e-3, ax_ * ax_ + ay_ * ay_)
            kmax = 1.0 / g('ack_plan_radius_m')
            w = v * max(-kmax, min(kmax, kap))
        self.m8_mode('exit', f'통로 출구선 맞추는 중: 방향 {math.degrees(herr):+.0f}°, 옆 {lat * 100:+.1f} cm')
        self.m7_publish(v, w)

    def m8_tick(self, now):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        m = self._m8
        car = self.m8_car()
        wz = self._imu[2] if self._imu is not None else self._yaw_rate
        h0 = g('m8_cone_half_m')
        boxes = [(x - h0, x + h0, y - h0, y + h0) for x, y in car if -0.6 < x < 1.6 and abs(y) < 1.2]
        # 틈 (통로를 가로지르는 두 콘) 과 그중 이어지는 줄 — 한 번만 계산해 목표점과 '본 라바콘' 에 같이 씀
        br_ = g('m8_body_r_m')
        near_c = [(x, y) for x, y in car if -0.5 < x < g('m8_look_m') and abs(y) < 1.0]
        gates = avoid.cone_gates(near_c, g('m8_gap_min_m'), br_)
        prev = m['target']
        ref_y, ref_x = 0.0, None
        if prev is not None:
            cy_, sy_ = math.cos(self._yaw), math.sin(self._yaw)
            px_ = (prev[0] - self._pos[0]) * cy_ + (prev[1] - self._pos[1]) * sy_
            py_ = -(prev[0] - self._pos[0]) * sy_ + (prev[1] - self._pos[1]) * cy_
            ref_y, ref_x = (py_, px_) if px_ > 0.05 else (0.0, None)
        # 이번 M8 에서 본 라바콘 (odom) — 끝에서 통로 축을 잡는다 (지나간 콘은 lidar_processor 기억에서 지워지므로 따로 모음).
        # 이어진 틈 줄의 양 끝 콘만 — 통로 밖 물체 (사람 다리) 는 진짜 줄 틈의 끝이 안 되므로 출구선 (m8_axis) 을 비틀지 않음
        odom_of = dict(zip(car, self._cones))
        for q in avoid.gate_chain(gates, ref_y, ref_x=ref_x, **self.m8_chain_kw()):
            for cp in q[3:5]:
                cx, cy = odom_of[cp]
                if all(math.dist((cx, cy), o) > 0.08 for o in m['seen']):
                    m['seen'].append((cx, cy))
        if m['exit'] is not None:
            self.m8_exit(now)
            return
        # 끝: 앞 m8_end_clear_m 안에 라바콘이 없고, 옆·뒤 라바콘도 m8_end_pass_m 넘게 지나감
        near = [(x, y) for x, y in car if abs(y) < 0.7]
        if not any(0.0 < x < g('m8_end_clear_m') for x, y in near) and all(x < -g('m8_end_pass_m') for x, y in near):
            ax = self.m8_axis()
            if ax is not None:
                m['exit'] = {'t': now, 'mid': ax[0], 'dir': ax[1]}
                self.get_logger().info(f'[M8] 마지막 라바콘 지남 → 통로 출구선 (마지막 쌍 가운데, 처음 → 마지막 쌍 방향 '
                                       f'{math.degrees(wrap(ax[1] - self._yaw)):+.0f}°, 본 라바콘 {len(m["seen"])} 개) 에 맞춘 뒤 차선 추종')
                self.m8_exit(now)
            else:
                self.m8_finish(now, '통로 축 모름 (본 라바콘 4 개 미만)')
            return
        # 후진 중: 오도메트리로 m8_back_m, 뒤는 라이다·라바콘 상자로 확인
        if m['back'] is not None:
            bk = m['back']
            rf = self.rear_free()
            mem = avoid.free(0.0, 0.0, 0.0, boxes, 0.06, g('m8_margin_m'), -1.0)
            moved = self.dist_from(bk['from'])
            why = ('끝' if moved >= g('m8_back_m') else '시간 초과' if now - bk['t'] > 3.0
                   else f'뒤 라이다 {rf:.2f} m' if rf is not None and rf < g('m7_back_lidar_m')
                   else '뒤 라바콘' if mem < 0.06 else None)
            if why is None:
                self.m8_mode('back', '후진 중')
                w_b = 0.0
                if self.ack() and m['target'] is not None:
                    # 아커만: 후진하면서 목표 틈 쪽으로 방향을 튼다 (곧게 후진하면 같은 자세로 다시 막힘)
                    c_, s_ = math.cos(self._yaw), math.sin(self._yaw)
                    tx_, ty_ = m['target'][0] - self._pos[0], m['target'][1] - self._pos[1]
                    brg = math.atan2(-tx_ * s_ + ty_ * c_, tx_ * c_ + ty_ * s_)
                    if abs(brg) > math.radians(3.0):
                        w_b = math.copysign(g('m7_back_speed') / g('ack_plan_radius_m'), brg)
                self.m7_publish(-g('m7_back_speed'), w_b)
                return
            m['back'] = None
            m['mon'].reset()
            self.get_logger().info(f'[M8] 후진 {why}: {moved:.2f} m → 다시 계산')
        # 틈: 통로를 가로지르는 두 콘 (실제 빈 거리 = 중심 거리 − 몸통 지름 ≥ m8_gap_min_m) 의 가운데점을 이은 길에서
        # 앞 m8_aim_m 점을 따라감 (pure pursuit). 그런 틈이 없으면 (콘이 한 줄·흩어짐) 방향 순으로 이웃한 콘 사이 틈 (Follow-the-Gap)
        lim = math.radians(g('m8_max_heading_deg'))
        aim = avoid.gate_aim(gates, g('m8_aim_m'), ref_y, ref_x, **self.m8_chain_kw())
        on_axis = False
        if aim is None:                    # 마지막 틈을 지났으면 통로 출구선을 따라감 (곧게 가면 끝 쌍 방향으로 벗어남)
            ax = self.m8_axis()
            if ax is not None:
                aim, along, _ = self.m8_axis_aim(ax[0], ax[1], g('m8_aim_m'))
                on_axis = along > -0.3 and aim[0] > 0.05
                aim = aim if on_axis else None
        gap = min((q[2] for q in gates if q[0] > 0.0), default=float('nan'))
        cands = []
        if aim is None:
            pts = sorted([(x, y) for x, y in near_c if x > -0.2], key=lambda p: math.atan2(p[1], p[0]))
            for a, b in zip(pts, pts[1:]):
                gw = math.dist(a, b) - 2 * br_
                mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
                if gw >= g('m8_gap_min_m') and mx > 0.1 and abs(math.atan2(my, mx)) <= lim:
                    cands.append((math.atan2(my, mx), mx, my, gw))
        prev = m['target']
        if prev is not None:
            c, s = math.cos(self._yaw), math.sin(self._yaw)
            px, py = (prev[0] - self._pos[0]) * c + (prev[1] - self._pos[1]) * s, -(prev[0] - self._pos[0]) * s + (prev[1] - self._pos[1]) * c
            pb = math.atan2(py, px) if px > 0.05 else 0.0
        else:
            pb = 0.0
        if aim is not None:
            mx, my = aim
        elif cands:
            _, mx, my, gap = min(cands, key=lambda q: abs(wrap(q[0] - pb)) + 0.3 * q[1])
        if aim is not None or cands:
            m['target'] = (self._pos[0] + mx * math.cos(self._yaw) - my * math.sin(self._yaw),
                           self._pos[1] + mx * math.sin(self._yaw) + my * math.cos(self._yaw))
            tgt, tdist = math.atan2(my, mx), math.hypot(mx, my)
        else:
            tgt, tdist, gap = 0.0, 0.5, float('nan')   # 틈이 안 보이면 지금 방향
        tgt = max(-lim, min(lim, tgt))
        need = min(g('m7_go_free_m'), max(0.05, tdist - 0.1))
        fr_t = avoid.free(0.0, 0.0, tgt, boxes, g('m8_check_m'), g('m8_margin_m'))
        if fr_t >= need and not m['forced']:
            h, fr = tgt, fr_t
        else:
            stp = math.radians(10.0)
            heads = sorted({round(max(-lim, min(lim, tgt + k * stp)), 4) for k in (-2, -1, 1, 2)} | {0.0})
            res = [(tgt, fr_t)] + [(hh, avoid.free(0.0, 0.0, hh, boxes, g('m8_check_m'), g('m8_margin_m'))) for hh in heads]
            ok = [r for r in res if r[1] >= need]
            if ok and not m['forced']:
                h, fr = min(ok, key=lambda r: abs(r[0] - tgt))
            else:   # 막혀도 조금이라도 앞으로 가는 쪽
                h, fr = max(res, key=lambda r: r[1] * math.cos(r[0]))
        fr_now = avoid.free(0.0, 0.0, 0.0, boxes, g('m8_check_m'), g('m8_margin_m'))
        sc_ = g('m8_slow_check_m')
        tight = avoid.free(0.0, 0.0, h, boxes, sc_, g('m8_slow_clear_m')) < min(sc_, fr) - 1e-6
        vmax = g('m8_slow_speed') if tight else g('m8_speed')
        m['slow_n'] += int(tight)
        v = max(0.0, min(vmax, g('m7_kv') * (min(fr, max(fr_now, 0.0) + 0.1) - g('m8_stop_gap_m'))))
        v = 0.0 if v < 0.02 else v
        w = g('m7_k_heading') * h - g('m7_kd_heading') * wz
        if v == 0.0:            # 통로 안 제자리 회전은 작게
            w = g('m7_k_heading') * max(-math.radians(g('m8_rot_limit_deg')), min(math.radians(g('m8_rot_limit_deg')), h)) - g('m7_kd_heading') * wz
        w = max(-g('m7_max_w'), min(g('m7_max_w'), w))
        if self.ack() and g('m8_ack_arcs'):
            # 아커만: 목표점 (mx, my) 쪽 pure pursuit 곡률에 가장 가까운, 라바콘에 안 닿는 곡률 (반경 ≥ ack_plan_radius_m) 원호.
            # 제자리 회전 (v = 0, w ≠ 0) 이 없으니 다 닿으면 0 → 막힘 감시가 조향 후진
            kmax = 1.0 / g('ack_plan_radius_m')
            ux, uy = (mx, my) if (aim is not None or cands) else (0.5, 0.0)
            kpp = max(-kmax, min(kmax, 2.0 * uy / max(1e-3, ux * ux + uy * uy)))
            K = np.linspace(-kmax, kmax, 15)
            clr = avoid.arc_clear(K, boxes, g('m8_ack_arc_m'), steps=(0.0, 0.01, 0.02, 0.03))
            okk = [(k_, c_) for k_, c_ in zip(K, clr) if c_ >= 0.0]
            if okk:
                kap, cm = min(okk, key=lambda q: abs(q[0] - kpp) - 0.5 * q[1])
                v = g('m8_slow_speed') if cm < 0.02 else min(g('m8_speed'), max(g('m8_slow_speed'), v))
                w = v * kap
                m['ack_blk'] = None
            else:
                v, w = 0.0, 0.0                # 어느 원호도 닿음 → 1 s 뒤 조향 후진 (막힘 감시는 명령 0 이면 안 잡음)
                if m.get('ack_blk') is None:
                    m['ack_blk'] = now
                elif now - m['ack_blk'] > 1.0:
                    m['ack_blk'] = None
                    m['back'] = {'from': self._pos, 't': now}
                    m['backs'] += 1
                    self.get_logger().warn(f'[M8] 아커만: 안 닿는 원호 없음 (곡률 ±{kmax:.1f}) → 조향 후진 {g("m8_back_m"):.2f} m')
                    self.m7_publish(0.0, 0.0)
                    return
        kind = 'slow' if tight else 'axis' if on_axis else 'gate' if aim is not None else 'gap' if cands else 'nogap'
        self.m8_mode(kind, {'slow': f'좁음 → {g("m8_slow_speed")} m/s (방향 {math.degrees(h):+.0f}°, 빈 거리 {fr:.2f} m)',
                            'gate': f'통로 틈 {len(gates)} 개 (가장 가까운 폭 {gap:.2f} m) 가운데선 따라감 → 방향 {math.degrees(h):+.0f}° (빈 거리 {fr:.2f} m)',
                            'axis': f'마지막 틈 지남 → 통로 출구선 따라감, 방향 {math.degrees(h):+.0f}° (빈 거리 {fr:.2f} m)',
                            'gap': f'틈 {gap:.2f} m 쪽 {math.degrees(tgt):+.0f}° → 방향 {math.degrees(h):+.0f}° (빈 거리 {fr:.2f} m)',
                            'nogap': f'틈이 안 보임 → 방향 {math.degrees(h):+.0f}° (빈 거리 {fr:.2f} m)'}[kind])
        why = m['mon'].update(now, self._pos, self._yaw, v, w)
        if why is not None:
            n = m['retry'].add(self._pos)
            if n > g('m8_retry_max'):
                if not m['forced']:
                    m['forced'] = True
                    self.get_logger().warn(f'[M8] 같은 자리 재시도 {g("m8_retry_max")} 번 넘음 ({why}) — 조금이라도 앞으로 가는 쪽으로 계속')
            else:
                m['back'] = {'from': self._pos, 't': now}
                m['backs'] += 1
                rf = self.rear_free()
                self.get_logger().warn(f'[M8] 막힘: {why} → 후진 {g("m8_back_m"):.2f} m ({n}/{g("m8_retry_max")}, '
                                       f'뒤 라이다 {"없음" if rf is None else f"{rf:.2f} m"})')
                self.m7_publish(0.0, 0.0)
                return
        m['n'] += 1
        self.m7_publish(v, w)

    # ---------------------------------------------------------------- 주기
    def line_remaining(self, st):
        line = st.get('line')
        if line is None:
            return None
        return line[1] - self.dist_from(line[0])

    def m3_remaining(self):
        return self.line_remaining(self._m3)

    def stop_back(self):
        """정지 목표 (앞바퀴 축) 가 정지선 가까운 끝보다 뒤에 있는 거리 = 바퀴 반지름 + 여유 + 미끄러짐 보정.
        line_remaining 은 이 목표까지 남은 거리 (0 이하 → 정지 명령)."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        return g('m3_wheel_front_m') + g('m3_line_margin_m') + g('m3_stop_gap_m')

    def wheel_gap(self, st):
        """바퀴 맨 앞 → 정지선 가까운 끝 [m] (+ = 선 앞, 음수 = 밟음 또는 넘음), 오도메트리 추정."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        return self.line_remaining(st) + g('m3_line_margin_m') + g('m3_stop_gap_m')

    # ---------------------------------------------------------------- M1 신호등
    def on_light(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self._light = msg
        m = self._m1
        if m:
            self.m1_light_mark(m, msg)
            m['green_n'] = m.get('green_n', 0) + 1 if (msg.color == 'green' and msg.size_checked) else 0
            # 노랑·빨강 연속 수: 초록에서만 0 (램프 전환 중 애매해서 none 인 영상은 건너뜀 — 그때마다 0 이면 통과 중 정지가 늦음)
            if msg.color in ('red', 'yellow') and msg.size_checked:
                m['stop_n'] = m.get('stop_n', 0) + 1
            elif msg.color == 'green':
                m['stop_n'] = 0
            if m['green_n'] == 1:
                m['green_t'] = self.get_clock().now().nanoseconds * 1e-9
        if (self.state in ('LANE',) + SLOW_STATES and not self._m1_done and 'M1' not in self.skip
                and (self.done_or_skipped('M8') or not g('m1_require_m8'))):
            self._m1_hits = self._m1_hits + 1 if msg.color != 'none' else 0
            if self._m1_hits >= g('m1_confirm_frames'):
                now = self.get_clock().now().nanoseconds * 1e-9
                self._m1 = {'start': self._pos, 'line': None, 't0': now, 'green_n': 0, 'last_log': now}
                self.m1_light_mark(self._m1, msg)
                self.set_state('M1_APPROACH', f'신호등 {msg.color} (거리 {msg.dist_m:.2f} m, 박스 {msg.y2 - msg.y1}px, '
                                              f'{self._m1_hits} 번 연속), odom {self.fmt(self._pos)} → 정지선 찾기')

    def on_parking_done(self, _msg):
        if self.state == 'M4_PARKING':
            self.set_state('DONE', '/parking/done — 주차·탈출 끝, 멈춤')

    def m1_light_mark(self, m, msg):
        """신호등 위치를 odom 점으로 기억 (카메라 → 신호등 거리를 차 진행 방향으로). 정지선이 이 신호등 바로 앞인지 볼 때 씀."""
        if msg.color == 'none' or msg.dist_m <= 0:
            return
        pose = self.pose_at(stamp_s(msg.header))
        if pose is None:
            return
        d = self.get_parameter('m7_cam_x').value + msg.dist_m
        m['light_pt'] = (pose[0] + d * math.cos(pose[2]), pose[1] + d * math.sin(pose[2]))

    def m1_line_near_light(self, msg):
        """이 가로선이 신호등 바로 앞 (m1_line_max_gap_m 안) 인가. 신호등 앞 횡단보도 정지선에만 서고,
        그보다 앞 서행 구간 시작 정지선은 지나가게 (2D 시뮬 11-09: 신호등 1.65 m 앞 정지선 x 2.30 에 섬, 맞는 선은 0.75 m 앞 x 3.20)."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        m = self._m1
        if msg.stop_line_dist_m <= 0 or g('m1_line_max_gap_m') <= 0 or m.get('light_pt') is None or m.get('line') is not None:
            return True                    # 신호등 위치를 모르거나 이미 선을 잡았으면 원래대로
        pose = self.pose_at(stamp_s(msg.header))
        if pose is None:
            return True
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        d = g('m7_cam_x') + msg.stop_line_dist_m
        lx, ly = pose[0] + d * c, pose[1] + d * s_
        gap = (m['light_pt'][0] - lx) * c + (m['light_pt'][1] - ly) * s_   # 선 → 신호등 (진행 방향)
        if gap > g('m1_line_max_gap_m'):
            n = m.get('far_n', 0)
            m['far_n'] = n + 1
            if n == 0:
                self.get_logger().info(f'[M1] 신호등 {gap:.2f} m 앞 가로선은 지나감 (서행 구간 정지선 — {g("m1_line_max_gap_m")} m 안 선에만 섬)')
            return False
        return True

    def m1_green(self):
        return self._m1.get('green_n', 0) >= self.get_parameter('m1_green_frames').value

    def m1_light_desc(self):
        lt = self._light
        if lt is None:
            return '신호등 메시지 없음'
        return (f'{lt.color}{" (크기 확인)" if lt.size_checked else ""}, 거리 {lt.dist_m:.2f}/예상 {lt.expected_dist_m:.2f} m, '
                f'박스 {lt.candidates} 개 중 버림 {lt.rejected}' + (f' — {lt.reason[:80]}' if lt.reason else ''))

    def m1_tick(self, now, cmd):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        m = self._m1
        s = self.state
        if s == 'M1_APPROACH':
            remain = self.line_remaining(m)
            if remain is None:
                if self.dist_from(m['start']) > g('m1_search_m'):
                    self._m1_done = True
                    self.get_logger().warn(f'[M1] 신호등 뒤 {g("m1_search_m")} m 안에 정지선을 못 찾음 — 포기 ({self.m1_light_desc()})')
                    self.start_parking('M1 포기 (정지선 못 찾음) — 주차는 그대로 한다')
                    return
                cmd.max_speed = float(g('m1_approach_speed'))
            elif self.m1_green():
                m['go_t'] = now
                self.set_state('M1_GO', f'초록 (크기 확인 {m["green_n"]} 번 연속) — 정지선 {remain * 100:+.1f} cm 앞에서 서지 않고 통과')
            elif remain <= 0.0:
                cmd.stop = True
                self.set_state('M1_STOP', f'정지 목표 도달 (남은 거리 {remain * 100:+.1f} cm), 정지 — {self.m1_light_desc()}')
            else:
                cmd.max_speed = float(min(g('m1_approach_speed'), max(g('m3_min_speed'), g('m3_kp') * remain)))
        elif s == 'M1_STOP':
            cmd.stop = True
            if 'still_t' not in m and self._speed < g('m3_still_mps'):
                m['still_t'] = now
                gap = self.wheel_gap(m)
                self.get_logger().info(f'[M1] 완전 정지: 바퀴 맨 앞 → 정지선 {gap * 100:+.1f} cm (+ = 선 앞, 0 아래면 밟음, 오도메트리 추정) — '
                                       f'{self.m1_light_desc()}')
            if self.m1_green():
                cmd.stop = False
                m['go_t'] = now
                self.set_state('M1_GO', f'초록 (크기 확인 {m["green_n"]} 번 연속, 첫 초록 판정 뒤 {now - m.get("green_t", now):.2f} s) → 발진, '
                                        f'정지 {now - m.get("still_t", now):.1f} s')
            elif now - m['last_log'] >= g('m1_log_s'):
                m['last_log'] = now
                self.get_logger().info(f'[M1] 정지 유지 — {self.m1_light_desc()}')
        elif s == 'M1_GO':
            remain = self.line_remaining(m)
            if (remain is not None and remain > g('m1_go_abort_m') and m.get('stop_n', 0) >= g('m1_stop_frames')
                    and self._light is not None):
                # 초록 보고 통과하기로 했는데 앞바퀴가 정지선에 닿기 전에 노랑·빨강 — 다시 정지선에 선다 (노랑도 정지)
                m['green_n'] = 0
                m.pop('go_t', None)
                m.pop('moving_t', None)
                self.set_state('M1_APPROACH', f'통과 중 {self._light.color} (크기 확인 {m["stop_n"]} 번 연속), 정지선 '
                                              f'{remain * 100:.0f} cm 앞 → 다시 정지선에 섬')
                return
            if remain is None or remain > -g('m1_exit_m'):
                cmd.max_speed = float(g('m1_go_speed'))   # 정지선 지나 m1_exit_m 까지 서행
            if 'moving_t' not in m and self._speed > 0.02:
                m['moving_t'] = now
                self.get_logger().info(f'[M1] 움직이기 시작: 첫 초록 판정 뒤 {now - m.get("green_t", now):.2f} s, 발진 명령 뒤 '
                                       f'{now - m["go_t"]:.2f} s')
            line = m.get('line')
            past = (self.dist_from(line[0]) - line[1] - self.stop_back()) if line else 0.0   # 앞바퀴 축이 정지선을 지난 거리
            if line is None or past >= g('m1_exit_m'):
                self._m1_done = True
                self.start_parking(f'정지선 {past:.2f} m 지남 → M1 끝')

    def tick(self):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        now = self.get_clock().now().nanoseconds * 1e-9
        cmd = LaneCmd(follow='both')
        s = self.state
        if s == 'M5_SLOW':
            since = self.dist_from(self._m5_seen)
            if since >= g('m5_release_m'):
                self.set_state('LANE', f'표지판 마지막으로 본 뒤 {since:.2f} m, 서행 구간 {self.dist_from(self._m5_start):.2f} m, '
                                       f'odom {self.fmt(self._pos)}')
            else:
                cmd.max_speed = float(g('m5_speed'))
        elif s == 'M6_SLOW':
            since = self.dist_from(self._m6_last_zig)
            if since >= g('m6_release_m'):
                self.set_state('LANE', f'지그재그 선 사라진 뒤 {since:.2f} m, 서행 구간 {self.dist_from(self._m6_start):.2f} m, '
                                       f'odom {self.fmt(self._pos)}')
            else:
                cmd.max_speed = float(g('m6_speed'))
        elif s == 'M3_APPROACH':
            remain = self.m3_remaining()
            if remain is None:
                if self.dist_from(self._m3['start']) > g('m3_search_m'):
                    self.set_state('LANE', f'표지판 뒤 {g("m3_search_m")} m 안에 정지선을 못 찾음 — 포기')
                    self._m3_cooldown_from = self._pos
                    self._m3_done = True
                cmd.max_speed = float(g('m3_approach_speed'))
            elif remain <= 0.0:
                cmd.stop = True
                self.set_state('M3_STOP', f'정지 목표 도달 (남은 거리 {remain * 100:+.1f} cm), 정지 명령, odom {self.fmt(self._pos)}')
            else:
                cmd.max_speed = float(min(g('m3_approach_speed'), max(g('m3_min_speed'), g('m3_kp') * remain)))
        elif s == 'M3_STOP':
            cmd.stop = True
            if 'still_t' not in self._m3 and self._speed < g('m3_still_mps'):
                self._m3['still_t'] = now
                gap = self.wheel_gap(self._m3)
                self.get_logger().info(f'[M3] 완전 정지: 바퀴 맨 앞 → 정지선 {gap * 100:+.1f} cm '
                                       f'(+ = 선 앞, 0 아래면 밟음, 오도메트리 추정), {g("m3_wait_s")} s 대기 시작')
            if 'still_t' in self._m3 and now - self._m3['still_t'] >= g('m3_wait_s'):
                cmd.stop = False
                self._m3['go_from'] = self._pos
                self.set_state('M3_GO', f'정지 {now - self._m3["still_t"]:.1f} s 뒤 발진')
        elif s == 'M7_AVOID':
            self.m7_tick(now)
        elif s == 'M8_AVOID':
            self.m8_tick(now)
        elif s == 'DONE':
            cmd.stop = True                # 주차·탈출 끝 — 멈춤
        elif s in ('M1_APPROACH', 'M1_STOP', 'M1_GO'):
            self.m1_tick(now, cmd)
        elif s == 'M3_GO':
            if self.dist_from(self._m3.get('go_from')) >= g('m3_exit_m'):
                self._m3_cooldown_from = self._pos
                self._m3_done = True
                self.set_state('LANE', f'정지선 통과 {g("m3_exit_m")} m, 이후 {g("m3_cooldown_m")} m 동안 보행자 표지판 무시')
        if self.state in ('LANE', 'M2_SHIFT') + SLOW_STATES:
            self.m2_tick(now, cmd)
        if self.state == 'LANE' and not self._m4_started and self.m4_ready():
            self.start_parking(f'연습 모드 — 건너뛴 {sorted(self.skip)} 앞의 미션까지 끝')
        if self.state in ('LANE', 'M4_PARKING') and now < self._m8_slow_until:
            cmd.max_speed = float(g('m8_exit_speed'))     # 라바콘 뒤 차선을 다시 잡는 동안 천천히
        cmd.reset_window = now < self._reset_until
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=self.state))


def main():
    # lane_follower 와 같게: rclpy 기본 SIGINT 처리기를 끄고, 정리 중에 오는 두 번째 Ctrl+C 는 무시
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
