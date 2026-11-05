# CLAUDE.md — autorace_ws

2026 국민대 AutoRace 트랙을 Isaac Sim 5.1 맵으로 만들고 AgileX LIMO 를 올린 ROS2 Humble 워크스페이스.
실행법·파라미터 표는 README.md. 여기에는 작업할 때 알아야 할 사실과 함정만 적는다.

## 작업 방식
- 단계마다 멈추고 결과(그림·측정값)를 보여 준 뒤 다음 단계로 간다 (사용자 요청).
- 주석·문서는 한국어. 기존 코드의 밀도·말투를 따른다.
- 같은 값은 한 곳에서: 센서 위치는 `sensor_mounts` (params_sim/params_real 두 파일에 같은 값), 토픽/프레임은 `params_*.yaml`.

## 트랙 (도면 `src/autorace_track/data/source/2026Track.dxf`)
- 도면은 1:30, A2. DXF 좌표는 시트 mm → DIMENSION 의 `dimlfac = 30` 을 곱해 실치수. 결과 10000.0 × 6000.0 mm.
- 모든 선이 `Visible` 레이어 하나에 **윤곽선만** 있다(채움 없음). 끝점이 1e-5 mm 수준으로 어긋나 있어 0.05 mm 격자 스냅 후 노딩 →
  닫힌 face 154 개 → 크기·모양·구멍/이웃으로 분류 (`tools/dxf_classify.py`, 규칙은 파일 상단 상수).
- 좌표계: 원점 = 매트 좌하단, +x = 도면 오른쪽, +y = 도면 위쪽. USD 는 m, Z-up, z = 0 이 매트 윗면.
- 치수: 차로 폭 500 (선 안쪽), 경계선 20, 정지선 40×500, 점선 20×200 (간격 150), 횡단보도 막대 70×200 + 앞뒤 가로선 30×500,
  주차 구획 500×600 × 6, 출발선 체커 2열 (40 칸, 위아래 30 칸) + 양옆 10 mm 띠 2 줄씩.
- 분류 결과(face 수): 경계선 4 (지그재그 2 를 떼어 냄), 점선 24, 정지선 7, 횡단보도 20 (2 곳), 마름모 3, '천천히' 글자 36 (3 곳),
  A/B 글자 2, 주차선 1, 출발선 흰 칸 13 + 검정 띠 4, 분리대 안쪽 1.
- 색 (참고 PNG 에서 추출): 매트 #36342e, 경계선·지그재그 노랑 #e5aa43, 나머지 흰색 #dfdaca, 출발선 띠 검정.
  (사용자에게 실제 테이프 색 확인 요청했으나 답 없음 → 참고 PNG 기준.)
- 맨 위 1 차로 왼쪽 끝(x 200–950 mm)은 도면에서 열려 있어 '트랙 밖' face 에 포함됨 = 출발 대기 구역.
- 왼쪽 직선의 40 mm 선 2 개(y 1840, 3940)는 정지선으로 분류했지만, 규정집(III.마)에 '미션 구간 시작 = 하얀 선 앞'이라 되어 있어
  미션 구간 표시로 본다. LimoIsaacSIM/autorace_2026 에서는 M8 라바콘 구간으로 썼다 (추정).
- 주행 순서(글자 방향으로 추정): 1차로 +x → 오른쪽 내려옴(-y, 지그재그·횡단보도) → 아래 A/B 분기(-x) → 왼쪽 올라감(+y) → 2차로 +x → 주차장.

## 차량 (LIMO, `Isaac/Robots/AgilexRobotics/limo/limo.usd`, Isaac 에셋 서버)
- USD 에는 센서가 없다 (third_person_camera 뿐). 4 바퀴 회전 관절 (velocity drive, damping 1e6), 바퀴 반지름 0.045,
  축거 0.2, 접지 윤거 0.175, 차체 2.156 kg + 바퀴 0.5 kg × 4, 차체 무게중심 chassis_link 기준 (0, 0, −0.1).
- chassis_link = base_link 자세, 바닥 위 0.145 m. articulation root = `/limo`.
- 실차 기준 토픽 (limo_ros2 humble): limo_base 가 `/cmd_vel` 구독, `/odom` `/imu` `/limo_status` 발행 (절대 이름, 리맵 안 됨),
  `pub_odom_tf` launch 기본 true, base_link→imu_link 항등 정적 TF 를 직접 냄. 라이다 YDLidar(EAI X2L) `/scan` laser_link.
  카메라 Orbbec Dabai (astra_camera `dabai.launch.py`) `/camera/color/*`, `/camera/depth/*` (16UC1 mm).
- 센서 위치: URDF 기준 laser (0.103, 0, −0.034), depth camera (0.084, 0, 0.03) [four_diff xacro 의 0.3 은 오타로 보고 ackermann 값 사용], imu (0,0,0).
  실차 tf_pub 노드의 laser 는 (0.1, 0, 0.05) — 불일치. 실측 필요.

## 함정 (겪은 것)
- **얇고 넓은 박스 충돌체 + 바퀴(원기둥) = 접촉 깨짐**: 바퀴가 바닥에 2~4 cm 파묻히고 차체가 끌린다. 바닥 충돌은 `UsdGeom.Plane` (PhysX 무한 평면)으로.
- **4륜 스키드, L/W > 1**: PhysX 쿨롱 마찰로는 거의 안 돈다 (제자리 ~20 %, 주행 중 ~5 %). 구 충돌체, μs=μd 로도 안 바뀜.
  → `limo_drive.py` 타이어 힘 모델 (바퀴 마찰 0 + 미끄럼 비례 힘, μ·N 포화) + 몸체 속도 PI. 이론 회전비 Cx·W²/(Cx·W²+Cy·L²) = 0.56 과 실측 일치.
- **ROS 를 source 한 셸에서 Isaac python**: PYTHONPATH 의 python3.10 rclpy 가 Isaac 내장(3.11) rclpy 를 가려 import 실패 → `isaac/run_isaac.sh` 가 ROS python 경로를 뺀다.
- **Isaac `python.sh` 는 시그널을 자식에 안 넘긴다**: launch 를 꺼도 Isaac 이 남음 → 래퍼가 INT/TERM 을 전달, run_sim.py 는 시그널에서 루프 탈출 후 `app.close()`.
- **SimulationApp.close() 는 stdout 버퍼를 안 비운다** → 리다이렉트하면 print 가 사라짐. `sys.stdout.reconfigure(line_buffering=True)`.
- **OnPhysicsStep 노드는 on-demand 파이프라인 그래프에서만 돈다** → IMU·odom 은 별도 그래프 (`GRAPH_PIPELINE_STAGE_ONDEMAND`).
- **ament_python 의 data_files 는 --symlink-install 에서도 복사**되고 갱신이 안 될 때가 있다 → bringup/sim 은 ament_cmake + `install(DIRECTORY)` (옵션 없이 써야 심링크).
- replicator annotator `detach()` 후 다른 annotator 를 쓰면 크래시 → 렌더 확인 스크립트는 detach 안 함.
- 카메라 clippingRange 기본 near = 1 m → 바로 앞 바닥이 잘린다. 항상 (0.01, …).
- 바닥 마찰을 시뮬 도중 바꿀 때는 stop → 값 설정 → reset 해야 반영된다.
- **사용자가 GUI 시뮬을 켜 둔 채로 시험하지 말 것**: 같은 `ROS_DOMAIN_ID`(30) 면 내 `/cmd_vel` 이 사용자 로봇도 움직이고 `/odom` 이 섞인다
  (2026-10-26 한 번 그랬음). headless 시험은 `ROS_DOMAIN_ID=77` 처럼 따로, 끝은 `--max-sim-seconds`·`timeout` 으로 (pkill 은 권한 거부됨).
- rclpy(Humble) 기본 SIGINT 처리기는 컨텍스트를 먼저 닫는다 → `finally` 의 정지 publish 가 RCLError. `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`.
- GUI ■ 뒤 ▶: 물리 뷰가 새로 생기지만 SingleArticulation 은 옛 핸들을 들고 있어 속도가 None (handles_initialized 는 True 라 못 믿음)
  → run_sim 이 ■ 를 보면 `live` 를 끄고 (물리 콜백 건너뜀) ▶ 뒤 `world.reset()`. ▶ 를 누른 app.update() 안에서 이미 물리 스텝이 돈다.
- `/sim/reset` 은 텔레포트 (시뮬 시간 유지). IsaacComputeOdometry 는 출발 자세 기준이라 텔레포트하면 odom 이 0 으로 돌아온다.
  라바콘은 RigidPrim 으로 묶어 첫 world.reset() 직후 자세를 저장해 두고 되돌린다. LimoDrive 적분항도 비운다 (벽에 막혀 포화됨).
- venv usd-core: `from pxr import Sdf` 만 하고 레이어를 열면 'usd' 플러그인 로드 실패 (libusd_kind) → `from pxr import Usd` 를 먼저.

## 차선 추종 (`src/autorace_lane`, SEAME 2026 Hackathon D-Racer-Kit lane_detector 의 Out 코스 상시 SW 코리도 경로를 옮김)
- 원본 HEAD 의 `lane_detector.py` 에는 `sw_concentric`·`sw_side_curv_*` 구현이 없다 (lane_node 에 선언만 남은 정리판). 빠뜨린 것 아님.
- Isaac 렌더의 노란 선은 #e5aa43 인데도 탈색돼 HSV H 18–26, S 40–120, V 120–255. 매트 S < 40, V 100–130 → `yellow_hsv_lo [15,45,130]`.
- 흰 마스크를 켜면 출발선 체커·'천천히'를 물고 트랙 밖으로 나간다. 3차선 가운데 차로는 흰 점선뿐 → 노란 선이 안 보이는 4 m 를 소실 홀드로 지남.
- 코너(시계방향이라 전부 우회전)에서 안쪽 선을 밟는 원인은 look-ahead 거리 (BEV 0.4–1.0 m ≈ 0.7 m 앞 → R 0.85 에서 ~0.29 m 파고듦) → 0.37–0.75 m.
- 원본은 서보(PID = 곡률). /cmd_vel 각속도에 바로 붙이면 감속할 때 반경이 줄어 파고든다 → ω = −scale × PID × v / v0.
- 왼쪽 위 코너(2차로 입구)가 제일 까다롭다: 안쪽 선이 시야 밖, 정면에 1차로의 열린 끝 → `curve_steer_bias` 0.5 미만이면 1차로로 넘어감.
- 원본 폭 학습은 하한(0.5×192)만 있어 방지턱 흔들림 뒤 ~100 px 선 쌍을 배우고 SW 페어 창이 따라가 차로 사이를 달렸다 (10-31)
  → `lane_width_tol: 0.25` (학습 144–240 px). lane_follow.launch 는 리셋 실패(exit≠0) 시 follower 를 안 띄우고 Shutdown.
- 끌 때 SIGINT 가 두 번 와서 finally 의 destroy_node 가 KeyboardInterrupt → finally 에서 SIGINT 무시.
- 시험 도구는 세션 scratchpad 에만 있었다 (trial.sh: 격리 도메인 headless sim + 노드 + /odom 기록, lane_score.py: 경계선까지 거리).

## 미션 (v3 설계: 카톡 받은 파일 `AutoRace2026_미션_해결방식과_진행방향_v3.pdf`, 2026-10-31)
- 위치: `~/.var/app/com.usebottles.bottles/data/bottles/bottles/KakaoTalk/drive_c/users/kb4/Documents/카카오톡 받은 파일/`.
  구조: SLAM·경로 계획 없음, FSM(보이면 켜지고 조건으로 끝남, 타임아웃은 진행 감시 비상구) + 반응형 제어. 노드 1 mission_manager
  (출발~M1), 노드 2 parking_controller (M4), cmd_vel_mux, odom_fusion (휠 거리 + IMU 각도). 미션 하나씩 붙이고 매번 커밋.
- YOLO: github.com/mark2455/2026_Autorace `yolo-training/models/best.pt` (YOLOv8n 640, crosswalk·parking·slow·speed_bump·turn_a·turn_b)
  → `autorace_perception/models/autorace_yolov8n.pt`. 시뮬 표지판 텍스처를 영상 크기 40–240 px 로 넣으면 잘 잡음
  (텍스처가 화면을 꽉 채우면 slow 를 못 잡음 — 학습 데이터가 작은 표지판 사진). ultralytics·torch 는 시스템 python3.10 user site.
- 토픽: `/perception/detections` (autorace_msgs/DetectionArray), `/mission/state` (String), `/mission/lane_cmd` (autorace_msgs/LaneCmd:
  max_speed 만 구현, follow/target_offset_m/param_set/use_white/reset_window 는 자리만). lane_follower 는 아직 `/cmd_vel` 직접 (mux 전).
- M5 (11-01): slow|speed_bump 3 연속 (conf ≥ 0.5, 박스 높이 ≥ 20 px) → 0.15 m/s, 마지막으로 본 곳 + 1.5 m 에서 해제.
  시뮬: 켜짐 world x ≈ 1.5, 방지턱 위 0.13–0.175 (평균 0.15), 해제 4.36. 새 노드도 lane_follower 처럼 SignalHandlerOptions.NO + finally SIG_IGN.
- 실행 (11-01 사용자 요청): 터미널 1 bringup (차는 멈춰 있음), 터미널 2 `ros2 launch autorace_mission drive.launch.py` 하나로
  check_nodes (겹치는 노드 있으면 중단) → yolo_detector + mission_manager → YOLO 로그 "(device" 를 OnProcessIO 로 보고 → reset_sim → lane_follower.
  출발 직후 서행 표지판이라 YOLO 준비 전에 출발하면 안 된다. 노드별 launch 도 디버깅용으로 남겨 둠. 사용자가 원하는 것: 로그가 한 터미널에, 재실행 = Ctrl+C → 같은 명령.
- M6 (11-01): 시뮬 지그재그는 촘촘한 톱니 노란 선 → SW 2차 피팅 잔차가 8.0–8.7 px (직선 2.2–2.4, 코너 ≤ 6.9). 차로 폭 변화로는 구분 안 됨.
  잔차 ≥ 7.5 px 5 장 → 0.15 m/s, < 5 px 뒤 0.3 m 해제. 조향은 그대로 (사용자: 중앙값으로 잘 가니 속도만).
- M3 (11-01): YOLO `crosswalk` = 보행자 표지판 (1.3 m 앞 박스 38 px). 정지선 = lane_follower BEV 흰색(노랑 뺀) 가로줄 cov ≥ 0.9
  (글자 0.74, 정차 LIMO 0.8, 횡단보도 막대 ~0.56). BEV 는 바닥 직사각형이라 행 ↔ 거리 선형. 정지 타깃은 횡단보도 앞 가로선 y 2.51
  (그 뒤 0.54 m 에 뒤 가로선 → 처음 잡은 선만 추적, 안 하면 56 cm 지나침). 영상 stamp 의 odom 위치로 짝지음. stop_gap 0.02 = 미끄러짐.
  시뮬 정지 오차 −0.2/−0.1 cm, 4.02 s 대기. 테스트 시뮬은 사용자 시뮬과 GPU 를 나눠 0.77× 로 느려질 수 있음 → 기록 창을 넉넉히.
- M7 (11-01): depth_obstacle (뎁스 16UC1 → 높이 ≥ 4 cm 점 → cv2.connectedComponents 묶음, scipy 는 numpy 2 와 충돌) → mission_manager
  M7_AVOID 가 /mission/cmd_vel 로 직접 운전, cmd_vel_mux 가 상태 보고 선택 (drive.launch 에서 lane_follower → /lane/cmd_vel).
  함정: ① mission_manager 를 리셋 전에 켜면 지난번 멈춘 자리의 LIMO 로 M7 이 켜져 출발선에서 직접 운전 → 리셋 뒤에 켜고, odom 튐 = 리셋 감지.
  ② 3차선 입구(x 8.3–9.0)는 노란 경계선이 비스듬히 벌어져 lane_heading 이 +30° 까지 틀림 (직선에서는 오차 ~0) → 차로 방향 = 최근
  (yaw + lane_heading) 원형 평균을 90° 로 스냅, 차로 중심 = 기억한 LIMO 옆 위치 0.52 m 격자 (세 차로 확인 시 고정).
  ③ 변경 허용을 '차 중심이 장애물 끝을 지난 뒤' 로 하면 0.9 m 간격 줄에서 늦다 → 끝이 차 중심보다 m7_enter_overlap_m 앞이어도 허용.
  ④ k_heading 3 은 시뮬 구동 지연으로 ±30° 진동 → 1.8 + 회전 속도 감쇠. ⑤ 기억 상자를 관측 합집합으로 하면 커진다 → 관측별 중앙값.
  M7 끝은 분리대 앞 표지판도 장애물로 보고 표지판 쪽 (못 봤으면 m7_prefer) 차로로 빠진 뒤 LANE.
- M2 (11-01): 상태 없이 M7 안에서. YOLO turn_a = A (분리대 아래 = 맨 왼쪽 차로 0), turn_b = B (맨 오른쪽 차로 2).
  함정: ① 이 YOLO 는 시뮬 렌더 A/B 표지판 (12 cm 판 + 지주) 을 1 m 앞 53 px 에서도 conf 0.02 까지 못 잡는다 (합성 그림은 60–90 px 부터,
  모델 박스는 지주까지 포함) → yolo_detector 보조 `branch_sign.py`: 파란 원 (HSV H 100–130) + 원 안 흰 무늬 32×32 를 A/B 그림과 정규화
  상관 (4 m 앞 14 px 부터 맞는 쪽 0.6–0.95, 반대 ≤ 0.47, 주차 P ≤ 0.46). ② YOLO 가 옆 정차 LIMO 바퀴 (흰 삼각 무늬, 화면 오른쪽 끝)
  를 turn_a 0.6–0.7 로 5 장 연속 잡음 → turn 박스 안에 파란 원이 있어야 인정. ③ 옆 차로 고를 때 '앞이 더 빈 쪽' 이 먼저면
  표지판 앞 마지막 변경에서 반대쪽으로 빠짐 → 표지판 쪽 먼저 + 그쪽이 내 차로에서 서기 전에 비면 기다림. ④ '표지판 쪽 차로로 미리
  옮기기' 를 앞에 장애물이 없을 때도 하면 분리대 옆에서 분리대를 가로지름 → 앞에 장애물이 있을 때만. ⑤ 갈림길 지난 뒤 (done) 다음
  바퀴 M7 에서 결과를 쓰면 안 됨. 시뮬 표지판은 params_sim branch_sign_period_s 30 s 마다 A↔B (A 시작) — 시험은 params_file 로 고정.
  drive.launch 의 reset_sim 이 10 s 안에 응답을 못 받아 실패한 적 1 번 (시험 PC GPU 부하, 리셋 자체는 됨).
- M7 막힘 (11-01 밤, 사용자 GUI 시뮬에서만 재현 — 실시간 1.0×, 내 시험 시뮬은 GPU 공유로 0.75×): rosbag 으로 확인하니 차로 변경을
  앞 LIMO 0.54 m 에서 시작해 모서리가 닿은 뒤 /mission/cmd_vel v +0.08 로 650 s 넘게 밀었다. ① 뎁스는 범퍼에 붙은 물체를 못 본다
  (카메라 0.08 m) + 기억을 30 s 뒤 지움 ② 속도를 '차 몸이 걸친 차로의 앞 장애물' 로 정해 비스듬히 빠져나가는 중이 안 맞음
  ③ 진행 감시가 목표 차로만 되돌림 → 차 사각형 쓸기 검사 (방향 후보 0~60° + 제자리 회전, 지금 향한 방향 빈 거리로도 속도 제한 —
  안 하면 돌면서 비스듬히 모서리로 다가가 라이다 안전 정지가 걸림), 1 s 안 움직이면 후진, 지나친 뒤에만 기억 삭제,
  mux 라이다 안전 정지 (라이다 최소 거리 0.12 m → 범퍼 0.06 m 안은 안 보임 → 물러날 때까지 래치).
  시험 도구 (scratchpad): m7_bag_rec.py (M3_GO~M7 끝 rosbag), m7_bag_eval.py (멈춤·충돌·밀기·mux 출처), layouts/ (LIMO 배치 6 가지).
  pkill/pgrep -f 패턴은 그 명령을 돌리는 내 셸 자신과도 맞는다 (스스로 죽거나 영원히 기다림) → PID 로.
- 갈림길 표지판 (11-01): 시뮬은 리셋마다 A/B 무작위, 그 판 동안 고정 (params_sim branch_sign_period_s 0 + random). M2 는 박스 45 px 에서 잠금.
- lane_follower 시간 제한은 노드 시계 (11-01 전에는 time.monotonic — 타이머는 sim 시각이라 섞여 있었음). 수정 전후 첫 바퀴 경로 차 ≤ 5 cm.
  미션 없이 2 바퀴째부터는 3차선 입구에서 위 경계 바깥 (y 1.98) 으로 가는 등 경로가 흔들린다 (수정 전에도 같음).
- M8 (11-02): lidar_processor (/scan 묶음 → odom 기억, 지나간 뒤 삭제) → M8_AVOID 가 직접 운전. 틈 = 통로를 가로지르는 두 콘
  (avoid.cone_gates), 가운데점 길 0.25 m 앞 pure pursuit (오프라인 운동학 시뮬로 정함: Stanley·곡률 호 후보는 더 나빴다).
  함정: ① 보이는 통로 끝 틈 (1 m 앞) 을 목표로 잡으면 S 에서 직선 쓸기 검사가 막혀 늦게 따라감 ② 왼쪽 직선 끝 = 오른쪽 급커브 입구
  — 틀어진 채 넘기거나, 가운데 (0.72, 4.25) 에서 윈도우를 새로 시작하면 앞을 가로지르는 왼쪽 경계 하나를 오른쪽 경계로 보고 트랙 밖
  → 끝에서 통로 방향 맞춤 + 미션 리셋 때 sliding_window.reset(center_hint=0.5). ③ narrow 를 S 와 겹치면 가운데선 그대로도 여유 1.3 cm
  → narrow 는 곧게. 시험 출발은 B 길 (3.40, 1.46, 180°) — 코너 바로 앞 (1.80, 0.85) 이나 커브 입구에서 시작하면 차선 추종이 못 잡는다.
  시험 도구: m8_trials.sh (배치 7 × 2), m8_eval.py, m8_offline.py (오프라인 계획 비교), m8_oracle.py (가운데선 이론 여유), lane_probe.sh.
- M1 (11-02): traffic_light 노드 (세 칸 밝기, 크기·위치로 모니터 거르기) + fake_traffic_light (시뮬 박스, decoy) → M1_APPROACH/STOP/GO.
  함정: ① 2차로 정지선은 커브 출구라 차가 7–18° 틀어져 봄 → 가로 행 덮개 검출이 놓침 → 기울기 13 가지 시어 검출 ② 신호등 노드가
  라바콘 직선 끝 정지선을 들고 와 예상 거리 −1 → 방향 45° 바뀌면 잊음 ③ 정지선 전에는 크기 확인이 안 돼 먼 진짜 신호등이 대비로
  버려지고 모니터 그림만 남음 → require_line ④ 시뮬 렌더러: 멈춘 카메라에서 램프를 바꾸면 꺼진 칸이 몇~십몇 초 밝음 (DLSS·디노이저
  누적) → just_lit (최근 어둡다가 밝아진 칸). FXAA 로 바꾸면 M8 뒤 커브 차선 이탈 3/8 → DLSS 유지 (params_sim anti_aliasing 3)
  ⑤ M8 끝을 방향만 맞추면 S 끝 쌍 때문에 20 cm 치우친 채 커브 → 가끔 바깥 선 추종 → 통로 출구선 따라가기.
  시험: m1_trials.sh (B 길 출발 → M8 → M1, red·green·flip·decoy × 2), m1_eval.py, m1_snap.py (정지선 화면), light_crop.py, stop_replay.py.
- M4 (11-02): parking_controller (WAIT → APPROACH → ALIGN → ROTATE_IN → MEASURE → ENTER → CHECK/FIX → HOLD → EXIT → ROTATE_OUT → DONE),
  mux 세 번째 소스 /parking/cmd_vel (/parking/state 가 IDLE·WAIT·DONE 아니면). 단독 parking.launch.py (reset_sim --pose 5.30 4.84 0).
  함정: ① 스키드 구동 지연 ~0.45 s, 0.15 rad/s 아래는 거의 안 돎 → 회전은 최소 0.2 + 지연 예측 멈춤 (pred = err − wz·0.45) + 가라앉을 때까지 대기
  (0.3 고정이면 ±4.5° 진동) ② 시뮬 odom 원점 = params 출발 자세 (리셋으로 옮긴 자리 아님) ③ ros2 topic pub --once 는 늦거나 사라짐 →
  /sim/start_pose 는 reset_sim --pose (응답 문구로 확인 후 재시도), /sim/parking_layout 은 -t 5 -r 5 ④ mux 가 WAIT 를 주차 중으로 보면
  0 명령 → 통합에서 영원히 WAIT ⑤ 시작 2 s 는 막힘 검사 끔, 칸 모를 때 막히면 APPROACH 로 ⑥ 도메인 77 에 시뮬 두 개 뜨면 토픽 섞여
  "Unable to convert call argument" 로 노드 죽음.
  시험: m4_trials.sh (배치 10), m4_rec.py, m4_eval.py (시뮬 실제 자세로 모서리·회전 오차·LIMO 간격), m14_trials.sh (B 길 → M8 → M1 → M4).
- 11-02 수정 (사용자 GUI 판): ① M1 초록 보고 통과 결정 (정지선 47 cm 앞) 뒤 선 15 cm 앞에서 노랑 → '25 cm 넘게 남았을 때만 다시 섬' 이라 통과
  → 선에 닿기 전이면 언제나 다시 섬 (m1_go_abort_m 0, m1_stop_frames 3, 애매(none) 영상은 노랑·빨강 연속 수를 안 지움) + 통과도 m1_go_speed 0.15 서행.
  ② M7 뎁스: 옆 LIMO 는 먼 옆면이 덜 보여 중심이 차 쪽으로 1.5–4.5 cm (rosbag 4 판, 비스듬히 볼수록) → 가까운 옆면 + m7_obstacle_width_m.
  노란 바깥선 하나로 잰 차로 가운데 (center_offset_m + 0.6 m·sin(방향 오차)) 는 방향 15° 안이면 실제와 0.3–0.8 cm → m7_line_fix 로 격자 보정.
  시험 도구 (scratchpad): m7_lat.py·m7_lat2.py (3차선 옆 위치·선 밟음), obs_err2.py (뎁스 관측 모서리 오차), yl.py (노란 선 가운데 정확도).
- 다음: 8 미션 모두 붙음 → 한 바퀴 전체 통합. (예전 메모) v3 7장 구현 순서는 정지선+M3 → M5·M6 → M2 → M1 → M7·M8 → M4. 사용자는 주행 순서(M5 → M6 → M3 → M7 → M2 → M8 → M1 → M4)로 진행 요청.

## 검증 도구
- `tools/check_track_isaac.py`: 미리보기 렌더 4 장 + 박스 미끄럼으로 μ_eff 측정 (μ 0.70 → 0.71, 런타임 0.35 → 0.353).
- `ros2 run autorace_sim smoke_test [--drive] [--out DIR]`: 토픽 주기, TF, 구동.

## 남은 일
- 4 단계: 미션 시설물은 `~/LimoIsaacSIM/autorace_2026` 에서 만든다 (config/missions.yaml → tools/build_mission_usd.py, 2026-10-26).
  이 워크스페이스 시뮬은 `params_sim.yaml` `missions.usd` 로 그 USD 를 올린다 (`isaac/missions.py` = autorace_2026 쪽 사본, 같이 고칠 것).
  규정집: `~/Downloads/Autorace2026_규정집 (1).pdf`, 미션 상세: `..._mission_detail_draft.docx (1).pdf`.
- 사용자 증상 '한 번 달리면 다시 못 달림' (10-26) = 차가 M7 정차 LIMO 에 박혀 선이 안 보임 → `/sim/reset`, lane_follow.launch 가 시작 전 리셋.
- 미션 동작: M5 서행, M6 지그재그 서행, M3 보행자 정지, M7 3차선 회피, M2 갈림길, M8 라바콘, M1 신호등 (위 "미션"). 나머지(주차)는 아직.
- 차선 추종 실차 튜닝: 색 문턱, 카메라 높이·기울기 실측 (`lane_replay` 로).
- 실차 `mode:=real` 검증, 센서 위치 실측, YDLidar 파라미터 대조 (limo_bringup/param/ydlidar.yaml).
