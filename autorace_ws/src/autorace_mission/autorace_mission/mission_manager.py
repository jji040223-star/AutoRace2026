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
        p('m3_labels', ['crosswalk'])      # 모델 클래스 이름 (보행자 표지판 = crosswalk)
        p('m3_min_conf', 0.5)
        p('m3_min_box_px', 30)
        p('m3_confirm_frames', 3)
        p('m3_approach_speed', 0.15)
        p('m3_min_speed', 0.04)
        p('m3_kp', 0.5)                    # 속도 [m/s] = kp × 남은 거리 [m]
        p('m3_cam_to_front_axle_m', 0.016) # 카메라 → 앞바퀴 축 (앞이 +). sim: 카메라 x 0.084, 앞축 x 0.1
        p('m3_stop_gap_m', 0.0)            # 앞바퀴 축을 정지선 가까운 끝에서 이만큼 앞에 세움 (판정 기준 확인 후 조정)
        p('m3_line_match_m', 0.10)         # 잡은 정지선과 이만큼 넘게 다른 측정 = 다른 선 (횡단보도 앞·뒤 가로선 간격 0.54 m)
        p('m3_search_m', 2.0)
        p('m3_wait_s', 4.0)
        p('m3_still_mps', 0.01)            # 이보다 느리면 "완전 정지"
        p('m3_exit_m', 0.5)
        p('m3_cooldown_m', 3.0)
        p('m3_max_box_px', 200)            # 이보다 큰 박스는 보행자 표지판 아님 (라바콘 구간에서 화면 가득 오검출 480 px)
        # M7 3차선 정적 장애물 회피
        p('m7_trigger_m', 1.2)             # 내 차로 앞 이 거리 안 장애물이면 켜짐 (뎁스 화각 ±35° — 옆 차로 장애물을 놓치지 않게 일찍)
        p('m7_trigger_half_m', 0.2)        # 내 차로 = 차 중심 옆 ±이만큼
        p('m7_min_width_m', 0.12)          # 정차 LIMO 폭 0.22. 라바콘(~0.09)·표지판 기둥으로는 안 켜지게
        p('m7_max_width_m', 0.45)          # 이보다 넓은 뎁스 장애물은 버림 — 방지턱에서 차가 기울면 바닥이 폭 2 m 장애물로 보였다 (11-02)
        p('m7_min_height_m', 0.10)         # 정차 LIMO 높이 0.25. 방지턱을 넘으며 차가 기울 때 바닥이 장애물로 보이는 것 (시뮬에서 겪음) 거르기
        p('m7_confirm_frames', 3)
        p('m7_lane_width_m', 0.52)         # 차로 중심 간격 (선 안쪽 500 + 선 20 mm)
        p('m7_lanes', 3)
        p('m7_lane_margin_m', 0.05)        # 장애물이 차로에 걸친다고 볼 최소 겹침
        p('m7_obstacle_len_m', 0.33)
        p('m7_obstacle_width_m', 0.22)     # 옆에 있는 장애물은 차에서 먼 옆면이 덜 보여 폭이 좁게·중심이 차 쪽으로 잡힌다 (시뮬 rosbag 4 판:
                                           # 비스듬히 본 LIMO 중심 3–4.5 cm, 앞에서 본 것 1.5–2 cm) → 가까운 옆면 + 이 폭 (0 = 끔)
        p('m7_side_min_m', 0.10)           # 장애물 가운데가 차 옆으로 이만큼 넘게 떨어져 있을 때만 (바로 앞은 양 옆면이 다 보임)
        p('m7_oblique_deg', 25.0)          # 이보다 비스듬히 (또는 범퍼 15 cm 안) 본 관측은, 앞에서 본 관측이 3 개 넘게 있으면 상자에 안 넣음
        p('m7_side_no_len', True)          # 옆에서 본·화각 가장자리에 걸린 관측은 가정 길이를 안 붙임 (false = 11-02 까지의 동작)
        p('m7_cam_x', 0.084)               # base_link → 카메라 [m] (depth_obstacle cam_x 와 같게)
        p('m7_cam_hfov_deg', 71.0)         # 뎁스 가로 화각 (Orbbec Dabai)
        p('m7_fov_edge_deg', 4.0)          # 관측의 가까운 끝이 화각 경계에서 이 안이면 '잘린 관측'
        p('m7_merge_m', 0.3)               # 관측 중심이 이 안이면 같은 장애물 (각도마다 보이는 면이 달라 0.2 m 로는 한 LIMO 가 둘로 기억됐다.
                                           # 정차 LIMO 끼리는 0.52 m 이상 떨어져 있다)       # 뎁스는 앞면만 보므로 장애물 길이를 이만큼으로 가정 (LIMO 0.322)
        p('m7_block_m', 1.2)               # 내 차로 앞이 이 거리 안에서 막히면 갈 차로를 정한다 (실제로 꺾는 때는 쓸기 검사가 정함)
        p('m7_enter_window_m', 0.30)       # 옆 차로 후보: 막은 장애물 앞면 이만큼 앞 ~ 뒷면 + 0.15 m 가 그 차로에서 비어 있어야 (차 길이쯤 — 꺾는 때는 쓸기 검사가 정함)
        p('m7_max_yaw_rate', 0.25)         # [rad/s] 이보다 빨리 돌고 있으면 (코너) 켜지 않음 — 차로 방향을 잘못 잡는다
        p('m7_speed', 0.15)
        p('m7_change_speed', 0.10)         # 차로 바꾸는 동안 — 앞으로 덜 가면서 옆으로 빠지게
        p('m7_kv', 1.0)                    # 앞 장애물까지 거리 비례 감속 [1/s]
        p('m7_stop_gap_m', 0.07)           # 쓸기 검사 거리 (차 + m7_margin_m) 가 이보다 짧으면 그 방향으로 안 감
        p('m7_margin_m', 0.03)             # 쓸기 검사: 차 사각형을 사방으로 이만큼 키움 (기억 상자 오차)
        p('m7_rot_margin_m', 0.015)        # 제자리 회전 검사 여유
        p('m7_check_m', 1.0)               # 쓸기 검사 길이
        p('m7_go_free_m', 0.15)            # 이 거리 이상 갈 수 있는 방향만 '갈 수 있음'
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
        p('m7_prefer', 'left')             # 옆 차로 둘이 같으면 이쪽 (M2 갈림길 표지판을 봤으면 그 결과로)
        p('m7_max_m', 6.0)
        p('m7_stuck_check_s', 1.0)         # 명령은 움직이라는데 (v > 0.03 또는 |w| > 0.3) 이 시간 동안
        p('m7_stuck_move_m', 0.01)         #   이 거리도
        p('m7_stuck_rot_deg', 2.0)         #   이 각도도 못 움직이면 → 바로 후진
        p('m7_blocked_s', 1.0)             # 갈 방향이 없으면 이 시간 뒤 차로 다시 고르기, 두 배 뒤 후진
        p('m7_back_m', 0.15)               # 후진 거리 (오도메트리)
        p('m7_back_speed', 0.06)
        p('m7_back_timeout_s', 4.0)
        p('m7_back_lidar_m', 0.05)         # 라이다로 본 뒤쪽 빈 거리가 이보다 짧으면 후진 멈춤
        p('m7_retry_radius_m', 0.20)       # 이 안에서 다시 막히면 '같은 자리' 재시도로 셈
        p('m7_retry_max', 3)               # 같은 자리 재시도 상한 — 넘으면 멈추고 에러 로그
        p('m7_align_deg', 3.0)             # 들어가자마자 차로 방향으로 제자리 정렬 (IMU), 이 안이면 끝
        p('m7_align_timeout_s', 3.0)
        p('m7_pass_margin_m', 0.30)        # 장애물 기억은 뒷면이 차 뒤끝보다 이만큼 뒤로 지나간 뒤에 지움 (시간으로 안 지움)
        p('m7_axis_snap_deg', 90.0)        # 차로 방향을 이 단위로 맞춤 (0 = 안 맞춤)
        p('m7_axis_frames', 20)            # 차로 방향 추정에 쓰는 최근 /lane/state 수
        p('m7_line_fix', True)             # 맨 왼쪽·맨 오른쪽 차로에서 노란 바깥선 하나만 보이면, 그 선으로 잰 차로 가운데와 격자 중심의 차이로
                                           # 격자를 옮김 (격자는 LIMO 로 잡은 그대로 + 이 보정. LIMO 가 차로 가운데에 없거나 오도메트리가 틀어져도)
        p('m7_line_fix_heading_deg', 15.0) # 차 방향이 차로 방향과 이 안이고 잡은 선 방향도 이 안일 때만 (rosbag: 15° 안 잰 가운데 오차 중앙 +0.3–0.8 cm)
        p('m7_line_fix_ahead_m', 0.60)     # center_offset_m 을 재는 곳 (base_link 앞, BEV 0.37–0.75 m + 카메라 0.084) — 방향 오차 보정용
        p('m7_line_fix_frames', 8)         # 이만큼 모이면 그 중앙값 × m7_line_fix_gain 만큼 격자를 옮김
        p('m7_line_fix_gain', 0.5)         # 한 번에 다 옮기면 측정 튐에 흔들림 (rosbag 재생: +8.7 → −5.4 cm)
        p('m7_line_fix_gate_m', 0.20)      # 격자 중심과 이보다 다르면 버림 (다른 선)
        p('m7_line_fix_max_m', 0.15)       # 보정 합 상한
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
        p('m8_exit_slow_s', 1.5)
        p('m8_exit_deg', 6.0)              # 끝에서 통로 출구선과 방향 오차가 이 안이고
        p('m8_exit_lat_m', 0.04)           #   옆 오차가 이 안이면 차선 추종으로 넘김
        p('m8_exit_look_m', 0.3)
        p('m8_exit_s', 3.0)                # 맞추기 최대 시간
        p('m8_exit_speed', 0.15)
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
        # M2 갈림길
        p('m2_label_a', 'turn_a')          # 모델 클래스 이름: A (왼쪽 길)
        p('m2_label_b', 'turn_b')          #                   B (오른쪽 길)
        p('m2_min_conf', 0.5)
        p('m2_min_box_px', 12)
        p('m2_confirm_frames', 3)
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
        self._hits = {'m5': 0, 'm3': 0}
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
        self._m4_started = False           # /parking/start 를 보냄
        self._light = None                 # 마지막 /perception/traffic_light
        self._lane = None                  # 마지막 /lane/state
        self._lane_dirs = collections.deque(maxlen=60)   # 최근 (차 yaw + 차선 방향) — 노란 선을 잡은 프레임만
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
    ORDER = ('M3', 'M7', 'M2', 'M8', 'M1')   # 순서 조건이 걸린 미션 (M2 는 M7 안에서 — M7 을 건너뛰면 같이 건너뜀)

    def done_or_skipped(self, key):
        """미션이 끝났는지 — 다음 미션의 순서 조건용. 건너뛴 미션은 '자기 앞 미션이 끝났을 때' 지난 것으로 친다
        (건너뛴 미션 때문에 앞쪽 순서 조건이 통째로 사라지지 않게: M8 을 건너뛰어도 M1 은 M2 를 지난 뒤에만)."""
        i = self.ORDER.index(key)
        if key in self.skip or (key == 'M2' and 'M7' in self.skip):
            return True if i == 0 else self.done_or_skipped(self.ORDER[i - 1])
        return {'M3': self._m3_done, 'M7': self._m7_done, 'M2': self._m2['done'], 'M8': self._m8_done, 'M1': self._m1_done}[key]

    def m4_ready(self):
        """주차를 시작할 때: M1 이 끝났거나 (포기 포함), M1 을 건너뛰면 그 앞의 남은 마지막 미션까지 끝났을 때."""
        return self._m1_done or ('M1' in self.skip and self.done_or_skipped('M1'))

    def start_parking(self, why):
        self._m4_started = True
        self.parking_pub.publish(Empty())
        self.set_state('M4_PARKING', f'{why} → /parking/start (parking_controller 가 주차 구역에서 넘겨받음)')

    def set_state(self, new, why):
        self.get_logger().info(f'[상태] {self.state} → {new} ({why})')
        self.state = new

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
        self._hits = {'m5': 0, 'm3': 0}
        self._resid_hits = self._m7_hits = 0
        self._m3, self._m7, self._obs = {}, {}, []
        self._m2 = {'side': None, 'cand': None, 'n': 0, 'done': False, 'locked': False}
        self._m3_cooldown_from = None
        self._odom_hist.clear()
        self._cones, self._m8, self._m8_hits, self._m8_done, self._m8_slow_until = [], {}, 0, False, 0.0
        self._m7_done = False
        self._m1, self._m1_hits, self._m1_done = {}, 0, False
        self._m3_done = False
        self._m4_started = False
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
        self._hits[key] = self._hits[key] + 1 if hit else 0
        return hit

    def on_detections(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.m2_update(msg)
        m3 = [d for d in self.hits(msg, 'm3') if d.y2 - d.y1 <= g('m3_max_box_px')]
        m5 = self.hits(msg, 'm5')
        if self._m2['done'] or self._m8_done:
            return                         # 순서 힌트: 갈림길 (·라바콘) 뒤에는 서행·보행자 표지판 미션이 없다
        cooling = (self._m3_cooldown_from is not None
                   and self.dist_from(self._m3_cooldown_from) < g('m3_cooldown_m'))
        if (m3 and self.state in ('LANE',) + SLOW_STATES and self._hits['m3'] >= g('m3_confirm_frames') and not cooling
                and 'M3' not in self.skip):
            best = max(m3, key=lambda d: d.confidence)
            self._m3 = {'start': self._pos, 'line': None}
            self.set_state('M3_APPROACH', f'{best.label} {best.confidence:.2f} 박스 높이 {best.y2 - best.y1}px, '
                                          f'{self._hits["m3"]} 장 연속, odom {self.fmt(self._pos)} → {g("m3_approach_speed")} m/s 로 정지선 찾기')
            return
        if m5 and self.state == 'M5_SLOW':
            self._m5_seen = self._pos
        elif m5 and self.state == 'LANE' and self._hits['m5'] >= g('m5_confirm_frames'):
            best = max(m5, key=lambda d: d.confidence)
            self._m5_seen = self._m5_start = self._pos
            self.set_state('M5_SLOW', f'{best.label} {best.confidence:.2f} 박스 높이 {best.y2 - best.y1}px, '
                                      f'{self._hits["m5"]} 장 연속, odom {self.fmt(self._pos)}')

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
        # M6: 톱니 지그재그 선 → 피팅 잔차가 커진다
        self._resid_hits = self._resid_hits + 1 if msg.line_resid_px >= g('m6_resid_on') else 0
        if self.state == 'M6_SLOW' and msg.line_resid_px >= g('m6_resid_off'):
            self._m6_last_zig = self._pos
        elif self.state == 'LANE' and self._resid_hits >= g('m6_confirm_frames') and not (self._m2['done'] or self._m8_done):
            self._m6_last_zig = self._m6_start = self._pos
            self.set_state('M6_SLOW', f'선 피팅 잔차 {msg.line_resid_px:.1f}px, {self._resid_hits} 장 연속, odom {self.fmt(self._pos)}')
        if self.state == 'M7_AVOID' and g('m7_line_fix'):
            self.m7_line_fix(msg)
        # 정지선 거리 (M3·M1 공통)
        if self.state == 'M3_APPROACH':
            self.line_update(self._m3, msg, 'M3', '표지판')
        elif self.state == 'M1_APPROACH':
            self.line_update(self._m1, msg, 'M1', '신호등')

    def line_update(self, st, msg, tag, what):
        """정지선 (카메라 → 가까운 끝) 을 영상 찍은 순간의 위치와 짝지어 둔다 — 이후는 오도메트리로 (line_remaining).
        처음 잡은 선과 m3_line_match_m 넘게 다른 측정은 다른 선 (횡단보도 뒤 가로선 등) 이라 버린다. M3·M1 이 같이 쓴다."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        if msg.stop_line_dist_m <= 0:
            return
        remain = msg.stop_line_dist_m - g('m3_cam_to_front_axle_m') - g('m3_stop_gap_m')
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
                                   f'앞바퀴까지 {remain:.3f} m, {what} 뒤 {self.dist_from(st["start"]):.2f} m')

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
            # 비스듬히·옆에서 본 관측: 보이는 면이 앞면이 아니라 가정 길이가 엉뚱한 쪽으로 붙는다 (차 뒤까지 늘어난 상자로 갇힘, 11-01)
            side_view = (abs(math.atan2((o.y_left + o.y_right) / 2, max(o.x_near, 0.05))) > math.radians(g('m7_oblique_deg'))
                         or o.x_near < CAR_HALF_L + 0.15)
            # base_link 상자 → 오도메트리 좌표 꼭짓점 (뒷면은 안 보이므로 앞면 + 가정 길이).
            # 옆에서 본 관측 (m7_side_no_len) 은 가정 길이를 안 붙인다: 화면 가장자리에 옆 차로 LIMO 옆면 끝만 보일 때 그 끝을 앞면으로 보고
            # 0.33 m 를 앞으로 늘리면 실제보다 0.3 m 앞에 상자가 생기고, 기억한 상자와 중심이 m7_merge_m 넘게 떨어져 새 장애물이 된다
            # (2D 시뮬 11-04: 가운데 차로에 없는 장애물 → 차로 변경 못 하고 멈춤). 보인 범위만 쓰면 기억한 상자와 합쳐져 버려진다.
            # 화각 가장자리에 걸린 관측도 같다: 옆 LIMO 의 차 쪽 끝이 화면 밖으로 잘려 먼 쪽 옆면만 보이는데, 차로를 바꾸느라 차가
            # 비스듬하면 side_view 각도 조건을 빠져나가 가정 길이가 붙는다 (2D 시뮬 11-04: 바로 옆 가운데 차로에 0.3 m 가짜 상자)
            edge = math.degrees(math.atan2(max(abs(o.y_left), abs(o.y_right)), max(o.x_near - g('m7_cam_x'), 0.01)))
            clipped = edge > g('m7_cam_hfov_deg') / 2 - g('m7_fov_edge_deg')
            no_len = g('m7_side_no_len') and (side_view or clipped)
            x2 = o.x_far if no_len else max(o.x_far, o.x_near + g('m7_obstacle_len_m'))
            yr, yl = o.y_right, o.y_left
            if g('m7_obstacle_width_m') > 0 and abs((yl + yr) / 2) > g('m7_side_min_m'):
                if (yl + yr) / 2 > 0:      # 왼쪽 장애물: 먼 옆면 = 왼쪽 끝
                    yl = max(yl, yr + g('m7_obstacle_width_m'))
                else:
                    yr = min(yr, yl - g('m7_obstacle_width_m'))
            pts = [(px + c * bx - s * by, py + s * bx + c * by)
                   for bx in (o.x_near, x2) for by in (yr, yl)]
            cx, cy = sum(p[0] for p in pts) / 4, sum(p[1] for p in pts) / 4
            for m in self._obs:            # 같은 장애물 (중심 m7_merge_m 안) 이면 관측을 모은다 (상자는 관측별 범위의 중앙값)
                if math.dist(m['c'], (cx, cy)) < g('m7_merge_m'):
                    if side_view and len(m['quads']) >= 3:
                        m['seen'] = now    # 앞에서 본 관측이 이미 있으면 상자는 그대로, 봤다는 것만
                        break
                    m['quads'] = (m['quads'] + [pts])[-12:]
                    cs = [(sum(p[0] for p in q) / 4, sum(p[1] for p in q) / 4) for q in m['quads']]
                    m['c'] = (sum(c[0] for c in cs) / len(cs), sum(c[1] for c in cs) / len(cs))
                    m['seen'] = now
                    break
            else:
                self._obs.append({'quads': [pts], 'c': (cx, cy), 'seen': now})
            mid = (o.y_left + o.y_right) / 2
            if (abs(mid) < g('m7_trigger_half_m') and o.x_near < g('m7_trigger_m')
                    and o.y_left - o.y_right >= g('m7_min_width_m') and o.height >= g('m7_min_height_m')):
                trig = True
        self.prune_obs(now)
        self._m7_hits = self._m7_hits + 1 if trig else 0
        two_yellow = self._lane is not None and self._lane.yellow_side == 2
        turning = abs(self._yaw_rate) > g('m7_max_yaw_rate')
        if (self.state in ('LANE', 'M6_SLOW') and self._m7_hits >= g('m7_confirm_frames') and not two_yellow
                and not turning and not self._m7_done and not self._m2['done'] and not self._m8_done and 'M7' not in self.skip):
            self.m7_enter()

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

    def m7_enter(self):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        # 차로 방향: 최근 (yaw + 차선 방향) 중앙값 → m7_axis_snap_deg 단위로
        recent = list(self._lane_dirs)[-int(g('m7_axis_frames')):]
        if recent:
            c = sum(math.cos(a) for a in recent)
            sn = sum(math.sin(a) for a in recent)
            est, how = math.atan2(sn, c), f'최근 차선 방향 {len(recent)} 장'
        else:
            est, how = self._yaw, '지금 yaw'
        snap = math.radians(g('m7_axis_snap_deg'))
        axis = round(est / snap) * snap if snap > 0 else est
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
        self.set_state('M7_AVOID', f'내 차로 앞 장애물 {self._m7_hits} 장 연속, 차로 방향 {math.degrees(axis):.0f}° '
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
        lc = self.m7_frame(pose[:2])[1]
        k = self.m7_k(round((lc - grid['phase']) / W))
        if k != {-1: 0, 1: g('m7_lanes') - 1}[msg.yellow_side]:
            return                         # 노란 선 쪽 끝 차로가 아니면 (가운데 차로에서 분리대 노란 선 등) 안 씀
        meas = lc + g('m7_line_fix_ahead_m') * math.sin(h) + msg.center_offset_m * math.cos(h)
        err = meas - self.m7_lane_l(k)
        if abs(err) > g('m7_line_fix_gate_m'):
            return
        buf = m.setdefault('line_fix', [])
        buf.append(err)
        if len(buf) < g('m7_line_fix_frames'):
            return
        d = sorted(buf)[len(buf) // 2] * g('m7_line_fix_gain')
        buf.clear()
        total = m.get('line_corr', 0.0)
        lim = g('m7_line_fix_max_m')
        d = max(-lim - total, min(lim - total, d))
        if abs(d) < 0.005:
            return
        m['line_corr'] = total + d
        grid['phase'] += d                 # 고정된 격자도 같이 (m7_grid 가 다시 계산할 때는 line_corr 를 더함)
        self.get_logger().info(f'[M7] 노란 선으로 차로 {k} 가운데 보정 {d * 100:+.1f} cm (합 {m["line_corr"] * 100:+.1f} cm, + = 왼쪽), '
                               f'odom {self.fmt(self._pos)}')

    def m7_lane_l(self, k):
        gr = self._m7['grid']
        return gr['phase'] + (gr['n_left'] - k) * self.get_parameter('m7_lane_width_m').value

    # ---------------------------------------------------------------- M7 운전: 차 사각형 쓸기 검사
    def m7_free(self, sc, lc, h, boxes, dmax, dirn=1.0):
        return avoid.free(sc, lc, h, boxes, dmax, self.get_parameter('m7_margin_m').value, dirn)

    def m7_rot_ok(self, sc, lc, psi, h, boxes):
        return avoid.rot_ok(sc, lc, psi, h, boxes, self.get_parameter('m7_rot_margin_m').value)

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
            if opts:
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
        m['back'] = {'from': self._pos, 't': now}
        self.get_logger().warn(f'[M7] 막힘: {reason} → 후진 {g("m7_back_m"):.2f} m (같은 자리 {count}/{g("m7_retry_max")} 번째, '
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
        near = [b for b in boxes if b[1] > sc - 0.8 and b[0] < sc + g('m7_check_m') + 0.5 and abs((b[2] + b[3]) / 2 - lc) < 1.3]
        # 라이다 점도 (범퍼 0.06 m 밖부터 정확) — 뎁스 기억은 가까이서 실제보다 멀게 잡힐 때가 있다 (11-02 시험: 17 cm)
        near += avoid.scan_boxes(self.scan_xy(), sc, lc, psi)
        k_lim = lambda x: max(-g('m7_max_w'), min(g('m7_max_w'), x))   # noqa: E731

        def in_lane(b, k):
            lk = self.m7_lane_l(k)
            return b[3] > lk - half_lane and b[2] < lk + half_lane

        def ahead(k):                        # 차 앞 → 차로 k 의 가장 가까운 장애물 앞면 (뒷면이 아직 차 중심보다 앞인 것)
            d = [b[0] - sf for b in boxes if in_lane(b, k) and b[1] > sc]
            return min(d) if d else math.inf

        def window_free(j, s0, s1):          # 차로 j 의 [s0, s1] 에 걸친 장애물이 없는가
            return not any(in_lane(b, j) and b[1] > s0 and b[0] < s1 for b in boxes)

        def after(j, s0):                    # s0 뒤 차로 j 의 첫 장애물 앞면까지
            d = [b[0] - s0 for b in boxes if in_lane(b, j) and b[0] >= s0]
            return min(d) if d else math.inf

        if m['gave_up']:
            self.m7_publish(0.0, 0.0)
            return
        # 1) 들어가자마자 IMU 방향으로 차로 방향 정렬 (제자리) — 비스듬한 채로 차로를 판단하지 않게
        if m['align'] is not None:
            done = abs(psi) < math.radians(g('m7_align_deg')) and abs(wz) < 0.1
            late = now - m['align'] > g('m7_align_timeout_s')
            if done or late or not self.m7_rot_ok(sc, lc, psi, 0.0, near):
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
            why = ('끝' if moved >= g('m7_back_m') else '시간 초과' if now - bk['t'] > g('m7_back_timeout_s')
                   else f'뒤 라이다 {rf:.2f} m' if rf is not None and rf < g('m7_back_lidar_m')
                   else '뒤 기억 장애물' if mem < 0.06 else None)
            if why is None:
                self.m7_mode('back', '후진 중')
                self.m7_publish(-g('m7_back_speed'), 0.0)
                return
            m['back'] = None
            m['rechoose'] = True
            self.get_logger().info(f'[M7] 후진 {why}: {moved:.2f} m 물러남 → 차로 다시 고르기')
            self.m7_publish(0.0, 0.0)
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
        own = [b for b in boxes if in_lane(b, k) and b[1] > sc]
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
                            if 0 <= j < g('m7_lanes') and j != k and window_free(j, o[0] - g('m7_enter_window_m'), o[1] + 0.15)})
            jg = None if goal is None or goal == k else k + (1 if goal > k else -1)
            if cands:
                if jg in cands:
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
        for h in sorted(set(round(c, 4) for c in cands)):
            dm = g('m7_check_m')
            if changing and abs(h) > 0.09:
                dm = min(dm, abs(lt - lc) / math.sin(abs(h)) + 0.25)     # 목표 차로까지만 본다 (그 너머 차로는 상관없음)
            fr = self.m7_free(sc, lc, h, near, dm)
            rok = abs(wrap(h - psi)) < math.radians(3.0) or self.m7_rot_ok(sc, lc, psi, h, near)
            res.append((h, fr, rok, dm))
        go = [r for r in res if r[2] and r[1] >= min(g('m7_go_free_m'), r[3] - 0.01)]
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
            rots = [r for r in res if r[2] and abs(wrap(r[0] - psi)) >= math.radians(3.0)]
            best = max(rots, key=lambda r: r[1], default=None)
            if best is not None and best[1] > max(cur, 0.0) + 0.05:
                w = k_lim(g('m7_k_heading') * wrap(best[0] - psi) - g('m7_kd_heading') * wz)
                self.m7_mode('rotate', f'제자리 회전 → {math.degrees(best[0]):+.0f}° (그 방향 {best[1]:.2f} m, 지금 방향 {cur:.2f} m)')
                blocked = False
            else:
                blocked = True
        if blocked:
            v = w = 0.0
            self.m7_mode('blocked', '갈 방향 없음 (쓸기 검사): ' + ', '.join(f'{math.degrees(r[0]):+.0f}° {r[1]:.2f} m'
                                                                       + ('' if r[2] else ' (회전 막힘)') for r in res))
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
        if (not changing and a_own > g('m7_clear_m') and clear and sc > 0.3) or sc > g('m7_max_m'):
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
                        'retry': RetryCounter(g('m7_retry_radius_m')), 'backs': 0, 'slow_n': 0, 'n': 0, 'seen': [], 'exit': None}
            self.set_state('M8_AVOID', f'앞 {g("m8_trigger_m")} m 안 라바콘 {len(ahead)} 개 (양쪽), {self._m8_hits} 번 연속, '
                                       f'odom {self.fmt(self._pos)}')

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
        if (abs(herr) < math.radians(g('m8_exit_deg')) and abs(lat) < g('m8_exit_lat_m')) or now - ex['t'] > g('m8_exit_s'):
            self.m8_finish(now, f'통로 출구선에 맞춤 (방향 {math.degrees(herr):+.0f}°, 옆 {lat * 100:+.1f} cm, {now - ex["t"]:.1f} s)')
            return
        h = math.atan2(ay_, ax_)
        wz = self._imu[2] if self._imu is not None else self._yaw_rate
        v = g('m8_exit_speed') * 0.5 if abs(h) < math.radians(15.0) else 0.0
        w = max(-g('m7_max_w'), min(g('m7_max_w'), g('m7_k_heading') * h - g('m7_kd_heading') * wz))
        self.m8_mode('exit', f'통로 출구선 맞추는 중: 방향 {math.degrees(herr):+.0f}°, 옆 {lat * 100:+.1f} cm')
        self.m7_publish(v, w)

    def m8_tick(self, now):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        m = self._m8
        car = self.m8_car()
        wz = self._imu[2] if self._imu is not None else self._yaw_rate
        h0 = g('m8_cone_half_m')
        boxes = [(x - h0, x + h0, y - h0, y + h0) for x, y in car if -0.6 < x < 1.6 and abs(y) < 1.2]
        # 이번 M8 에서 본 라바콘 (odom) — 끝에서 통로 축을 잡는다 (지나간 콘은 lidar_processor 기억에서 지워지므로 따로 모음)
        for cx, cy in self._cones:
            if all(math.dist((cx, cy), q) > 0.08 for q in m['seen']):
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
                self.m7_publish(-g('m7_back_speed'), 0.0)
                return
            m['back'] = None
            m['mon'].reset()
            self.get_logger().info(f'[M8] 후진 {why}: {moved:.2f} m → 다시 계산')
        # 틈: 통로를 가로지르는 두 콘 (실제 빈 거리 = 중심 거리 − 몸통 지름 ≥ m8_gap_min_m) 의 가운데점을 이은 길에서
        # 앞 m8_aim_m 점을 따라감 (pure pursuit). 그런 틈이 없으면 (콘이 한 줄·흩어짐) 방향 순으로 이웃한 콘 사이 틈 (Follow-the-Gap)
        lim = math.radians(g('m8_max_heading_deg'))
        br_ = g('m8_body_r_m')
        near_c = [(x, y) for x, y in car if -0.5 < x < g('m8_look_m') and abs(y) < 1.0]
        gates = avoid.cone_gates(near_c, g('m8_gap_min_m'), br_)
        aim = avoid.gate_aim(gates, g('m8_aim_m'))
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

    # ---------------------------------------------------------------- M1 신호등
    def on_light(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self._light = msg
        m = self._m1
        if m:
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
                self.set_state('M1_APPROACH', f'신호등 {msg.color} (거리 {msg.dist_m:.2f} m, 박스 {msg.y2 - msg.y1}px, '
                                              f'{self._m1_hits} 번 연속), odom {self.fmt(self._pos)} → 정지선 찾기')

    def on_parking_done(self, _msg):
        if self.state == 'M4_PARKING':
            self.set_state('DONE', '/parking/done — 주차·탈출 끝, 멈춤')

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
                self.set_state('M1_STOP', f'앞바퀴 정지선 도달 (남은 거리 {remain * 100:+.1f} cm), 정지 — {self.m1_light_desc()}')
            else:
                cmd.max_speed = float(min(g('m1_approach_speed'), max(g('m3_min_speed'), g('m3_kp') * remain)))
        elif s == 'M1_STOP':
            cmd.stop = True
            if 'still_t' not in m and self._speed < g('m3_still_mps'):
                m['still_t'] = now
                gap = self.line_remaining(m) + g('m3_stop_gap_m')
                self.get_logger().info(f'[M1] 완전 정지: 앞바퀴 축 → 정지선 가까운 끝 {gap * 100:+.1f} cm (+ = 선 앞, 규정 ±5 cm) — '
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
            past = (self.dist_from(line[0]) - line[1]) if line else 0.0
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
                self.set_state('M3_STOP', f'앞바퀴 정지선 도달 (남은 거리 {remain * 100:+.1f} cm), 정지 명령, odom {self.fmt(self._pos)}')
            else:
                cmd.max_speed = float(min(g('m3_approach_speed'), max(g('m3_min_speed'), g('m3_kp') * remain)))
        elif s == 'M3_STOP':
            cmd.stop = True
            if 'still_t' not in self._m3 and self._speed < g('m3_still_mps'):
                self._m3['still_t'] = now
                gap = self.m3_remaining() + g('m3_stop_gap_m')       # 남은 거리는 stop_gap 을 뺀 목표 기준 → 선 기준으로
                self.get_logger().info(f'[M3] 완전 정지: 앞바퀴 축 → 정지선 가까운 끝 {gap * 100:+.1f} cm '
                                       f'(+ = 선 앞, 오도메트리 추정, 규정 ±5 cm), {g("m3_wait_s")} s 대기 시작')
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
        if s == 'LANE' and not self._m4_started and self.m4_ready():
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
