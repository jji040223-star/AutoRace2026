# AutoRace2026

2026 국민대 AutoRace 를 AgileX **LIMO Pro** 로 나가기 위한 자율주행 스택과, 그것을 시험하는 **Isaac Sim 5.1 트랙 시뮬레이션**.
ROS2 Humble, 실차와 같은 토픽 이름 (`/cmd_vel`, `/odom`, `/imu`, `/scan`, `/camera/color/*`, `/camera/depth/*`) 을 쓰므로 같은 노드를 시뮬과 실차에서 돌린다.

> 자세한 설명 (실행 로그 읽는 법, 미션별 파라미터 표, 시뮬 시험 결과, 문제 해결) 은 **[README_AutoRace2026_LIMO.md](README_AutoRace2026_LIMO.md)**.

## 저장소 파일

| 파일 | 내용 | 언제 쓰나 |
|---|---|---|
| `autorace_ws_B_20261102_미션포함.zip` (32 MB) | 아래 `전체통합` 전부 + 미션 시설물 USD (`LimoIsaacSIM/autorace_2026/data/mission`) + 정차 LIMO 모형이 참조하는 WeGo LIMO 모델 2 개 | **다른 PC 에서 미션까지 시뮬할 때 — 이것 하나면 된다** |
| `autorace_ws_B_20261102_전체통합.zip` (17 MB) | `autorace_ws` 워크스페이스 소스 (7 패키지, 트랙 USD, YOLO 모델 2 개, 문서) | 이미 `~/LimoIsaacSIM` 이 있는 PC, 실차 LIMO 에 옮길 때 |
| `README_AutoRace2026_LIMO.md` | 전체 정리 문서 (zip 안에도 같은 것) | |
| `2024-11-20-17-28-06.zip` | 예전 업로드 (이번 작업과 무관) | |
| [`yolo/`](yolo/) | YOLO 학습용 데이터셋 (신호등 `red sign`·`yellow sign`·`green sign`, 515 장) | 신호등 YOLO 모델 학습할 때 — 아래 **YOLO** 참고 |

두 zip 의 `autorace_ws` 는 같은 소스 (2026-11-02, 로컬 git 커밋 `2094c4d`) 이고, git 기록 (`.git`) 은 들어 있지 않다.
WeGo LIMO 모델은 WeGo-Robotics 배포물이므로 저장소를 공개하거나 팀 밖에 줄 때 주의.

## YOLO

YOLO 데이터셋은 [`yolo/`](yolo/) 폴더에 모았다. 자세한 설명은 **[yolo/README.md](yolo/README.md)** 를 보면 된다.

| 데이터셋 | 클래스 | 이미지 | 형식 |
|---|---|---|---|
| [`yolo/traffic_light/`](yolo/traffic_light/) | `green sign` (0), `red sign` (1), `yellow sign` (2) | 515 (train 360 / valid 103 / test 52) | YOLOv8 (Roboflow 내보내기 구조) |

- 정면 3구 신호등 몸체만 라벨링했다. 옆면 보행자 신호와 다른 표지판은 라벨링하지 않았다.
- zip 이 100 MB 를 넘어서 3 조각 (`.part00`–`.part02`) 으로 나눠 올렸다. 합쳐서 풀면 된다:
  ```bash
  cd yolo/traffic_light && cat traffic_light_yolo.zip.part* > traffic_light_yolo.zip && unzip traffic_light_yolo.zip
  ```

## 다른 PC 에서 실행

필요: Ubuntu 22.04, NVIDIA RTX (VRAM 8 GB↑), ROS2 Humble, Isaac Sim 5.1 (`~/isaacsim`), 첫 실행 때 인터넷 (주행 LIMO 모델을 NVIDIA 에셋 서버에서 받음).

```bash
cd ~
unzip autorace_ws_B_20261102_미션포함.zip     # → ~/autorace_ws, ~/LimoIsaacSIM (홈에 풀어야 경로가 맞음)

sudo apt install -y python3-colcon-common-extensions python3-rosdep ros-humble-teleop-twist-keyboard
pip3 install --user ultralytics==8.4.153      # 미션 YOLO
source /opt/ros/humble/setup.bash
cd ~/autorace_ws && rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install

# ~/.bashrc
source ~/autorace_ws/install/setup.bash
export ROS_DOMAIN_ID=30
```

자세한 단계·Isaac 경로가 다를 때·오프라인 LIMO 모델: zip 안 `autorace_ws/INSTALL_다른PC.md`.

```bash
# 터미널 1: 시뮬 (트랙 + LIMO + 미션 시설물). 차는 멈춰 있다
ros2 launch autorace_bringup bringup.launch.py mode:=sim

# 터미널 2: 주행 — YOLO 준비 → 출발선 리셋 → 미션 매니저 + 차선 추종
ros2 launch autorace_mission drive.launch.py
# 다시 달리기: Ctrl+C → 같은 명령 / 바퀴 안 돌리고 인식만: enable:=false
```

시뮬 로그에 `미션 USD 없음 → 시설물 없이 진행` 이 나오면 `~/LimoIsaacSIM/...` 경로가 틀린 것 (`params_sim.yaml` 의 `missions.usd`).

## 구성

| 패키지 | 하는 일 |
|---|---|
| `autorace_track` | 트랙 도면 (`2026Track.dxf`, 10 × 6 m) → 선 분류 → Isaac USD |
| `autorace_sim` | Isaac Sim 실행 (`run_sim.py`): LIMO + 센서 (카메라·뎁스·라이다·IMU) + 미션 시설물 전환 (신호등 주기, 갈림길 A/B), `/sim/reset` |
| `autorace_bringup` | `bringup.launch.py mode:=sim|real`, 시뮬·실차 파라미터 |
| `autorace_lane` | 슬라이딩 윈도우 차선 추종 (SEAME 2026 Hackathon D-Racer-Kit 에서 옮김), 정지선 검출, `/lane/state` |
| `autorace_perception` | `yolo_detector` (표지판 YOLOv8n + 갈림길 A/B 보조 판별), `traffic_light` (신호등 색 + 광고 모니터 거르기), `depth_obstacle`, `lidar_processor` (라바콘) |
| `autorace_mission` | `mission_manager` (출발 ~ M1 상태 기계), `parking_controller` (M4), `cmd_vel_mux` (차선/미션/주차 명령 선택 + 라이다 안전 정지), `drive.launch.py` |
| `autorace_msgs` | `LaneState`, `LaneCmd`, `DetectionArray`, `ObstacleArray`, `TrafficLight` |

```
카메라 ─┬→ lane_follower ──→ /lane/cmd_vel ──────────┐
        ├→ yolo_detector ─→ /perception/detections ─┐ │
        └→ traffic_light ─→ /perception/traffic_light├→ mission_manager ─→ /mission/cmd_vel (M7·M8) ─┤
뎁스 ───→ depth_obstacle ─→ /perception/obstacles ──┤        └→ /mission/lane_cmd (속도 상한·정지) → lane_follower
라이다 ─→ lidar_processor → /perception/cones ──────┘                                              ├→ cmd_vel_mux → /cmd_vel
                                         /parking/start → parking_controller → /parking/cmd_vel ───┘
```

## 미션 (주행 순서)

| 미션 | 인식 | 동작 |
|---|---|---|
| M5 방지턱 서행 | YOLO `slow`·`speed_bump` | 0.15 m/s, 표지판 마지막으로 본 뒤 1.5 m 에서 해제 |
| M6 서행 유도선 (지그재그) | 차선 피팅 잔차 ≥ 7.5 px | 0.15 m/s (조향은 차선 추종) |
| M3 보행자 표지판 | YOLO `crosswalk` + BEV 정지선 | 정지선에 앞바퀴 맞춰 정지 → 4 s → 발진 |
| M7 3차선 정적 장애물 | 뎁스 장애물 (오도메트리로 기억) | 미션 매니저가 직접 운전: 차로 격자 (정차 LIMO + 노란 바깥선 보정), 차 사각형 쓸기 검사로 빈 차로로 변경, 막히면 후진 |
| M2 갈림길 A/B | YOLO `turn_a`/`turn_b` + 파란 원·그림 상관 보조 | M7 끝에서 표지판 쪽 차로 (A = 왼쪽, B = 오른쪽) 로 빠짐 |
| M8 라바콘 | 라이다 묶음 | Follow-the-Gap (콘 틈 가운데선 pure pursuit), 끝에서 통로 출구선에 맞춘 뒤 차선 추종 |
| M1 신호등 | 신호등 박스 세 칸 밝기, 크기·위치로 광고 모니터 거르기 | 빨강·노랑 → 정지선 정지, 초록 → 발진. 통과 중에도 서행, 선 전에 노랑·빨강이면 다시 정지 |
| M4 T자 주차 | 라이다 (양옆 LIMO 사이 빈 칸) | 칸 가운데 정렬 → 90° → 전진 진입 → 2.5 s → 후진 → 90° → 종료 |

## 시뮬 확인 결과 (Isaac headless, 자세한 표는 정리 문서 §8-5)

| | 결과 |
|---|---|
| 차선 추종 한 바퀴 (시설물 끔) | 바퀴가 선에 닿은 적 없음 (선까지 최소 0.11 m) |
| M5 / M6 | 방지턱 위 평균 0.151 m/s / 지그재그 0.150 m/s |
| M3 정지 | 앞바퀴 ↔ 정지선 −0.2, −0.1 cm, 4.02 s 뒤 발진 |
| M7 (막힘 수정 뒤) | 10 판 + 사용자 GUI 1 판, 정차 LIMO 충돌 0 (배치 6 가지) |
| M2 | 6 판 모두 표지판 쪽 길 |
| M1 | 빨강 정지 +0.2, +1.4 cm → 초록 발진 / 초록이면 서지 않고 통과 |
| M4 | 단독 10 배치 모두 완료 (네 모서리 모두 칸 안, 가장 좁은 곳 8.9 cm), M8 → M1 → M4 통합 2 판 완료 |

## 최근 변경 (2026-11-02 오후)

- **M1**: 초록 보고 통과하기로 한 뒤 정지선 앞에서 노랑이 되면 그대로 지나가던 것 → 앞바퀴가 선에 닿기 전이면 언제나 다시 정지선에 섬 (노랑도 정지). 통과할 때도 정지선 지나 0.3 m 까지 0.15 m/s 서행.
- **M7**: 3차선에서 차가 차로 가운데보다 왼쪽으로 치우쳐 선을 물고 가던 것 → ① 옆 차로 LIMO 는 뎁스에서 먼 옆면이 덜 보여 중심이 차 쪽으로 1.5–4.5 cm 잡혔다 → 가까운 옆면 + LIMO 폭으로 채움 ② 맨 왼쪽·오른쪽 차로에서 노란 바깥선으로 잰 차로 가운데 (실제와 0.3–0.8 cm) 로 차로 격자 보정. 기존 방식은 그대로 두고 더한 것.
- 이 두 변경은 rosbag 재생으로만 확인했다 (시뮬 주행 확인 전).

## 남은 일

- 이번 M1·M7 변경을 시뮬 주행으로 확인
- 실차 젯슨 지연을 넣은 시뮬 시험, M7 정차 LIMO 위치를 바꾼 시험
- 한 바퀴 전체 통합 주행
- 실차 (`mode:=real`): 차선 색 문턱, 카메라 높이·기울기, 센서 위치, 정지 미끄러짐 (`m3_stop_gap_m`), 신호등 위치 (`light_beyond_m`) 실측
