# 2026 AutoRace 트랙 + LIMO — Isaac Sim 정리

2026 국민대 AutoRace 트랙 도면(`2026Track.dxf`)을 Isaac Sim 맵으로 만들고 AgileX LIMO 를 올려,
실차와 같은 ROS2 토픽으로 쓸 수 있게 만든 것의 정리. (작성 2026-10-25, 갱신 2026-11-02 오후: M1 통과 중 서행·정지선 전 노랑 정지, M7 차로 중심 보정 §8-4 · 2026-11-01: 미션 M5 서행·M6 지그재그·M3 보행자 정지·M7 3차선 회피·M2 갈림길 §8 · 2026-10-31: 다시 달리기 수정 §7-2 · 2026-10-26: 미션 시설물 §6, 슬라이딩 윈도우 차선 추종 §7)

결과물은 두 가지다.

| | A. `autorace_2026` 확장 (WeGo LimoIsaacSIM 기반) | **B. `autorace_ws`** (자체 ROS2 워크스페이스) |
|---|---|---|
| 위치 | `~/LimoIsaacSIM/autorace_2026` (git 브랜치 `autorace_2026`, 커밋 `f6706a5`) | `~/autorace_ws` (git init 만, 커밋 없음) |
| 차량 | WeGo `limo_diff_thin.usd` + 원본 센서·OmniGraph 그대로 | Isaac 에셋 `limo.usd` + 직접 붙인 센서 |
| 실행 | Isaac GUI → Examples → Wego → AutoRace 2026 | `ros2 launch autorace_bringup bringup.launch.py mode:=sim` |
| 창 제목 | Isaac Sim 5.1.0 | Isaac Sim **Python** 5.1.0 |
| 미션 시설물 (§6) | ✅ | ✅ (A 에서 만든 USD 를 같이 씀) |
| 차선 추종 (§7) | 토픽이 같아 돌아는 감 (카메라 1280×720, 화각 60° — 설정 바꿔야 함) | ✅ 시뮬 확인. 실차 LIMO Pro 와 토픽·카메라가 같음 |
| 트랙 | 둘 다 같은 도구로 DXF 에서 생성 (스케일 1 m, 원점 = 트랙 좌하단) | |

Stage 트리로 구분: A = `/World/AutoRaceTrack`, `/World/Limo`, `/World/AutoRaceMissions` / B = `/World/Track`, `/World/limo`, `/World/Missions`.

---

## 0. 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 22.04 |
| Isaac Sim | 5.1.0 (`~/isaacsim`) — WeGo 원본은 4.5 대상이지만 5.1 에서 동작 확인 |
| ROS2 | Humble, Fast DDS, `ROS_DOMAIN_ID=30` (`~/.bashrc`) |
| 트랙 도구용 venv | `~/venvs/autorace` (ezdxf, shapely, matplotlib, usd-core==24.5) |

---

## 1. 트랙 (공통)

- 도면: 1:30, A2 → DXF 좌표에 DIMENSION 의 `dimlfac = 30` 을 곱해 실치수. 결과 **10000 × 6000 mm**, 차로 폭 500 mm.
- 모든 선이 `Visible` 레이어 하나에 윤곽선만 있어서, 선을 이어 닫힌 영역 154 개를 만든 뒤 크기·모양으로 분류.

| 분류 | 개수 | 색 (USD) |
|---|---|---|
| 도로 경계선 (20 mm) / 지그재그 | 4 / 2 | 노랑 `#e5aa43` |
| 차선 점선 (20×200) | 24 | 흰색 `#dfdaca` |
| 정지선 (40×500) | 7 | 흰색 |
| 횡단보도 (막대 70×200 + 가로선 30×500) | 2 곳, 20 | 흰색 |
| '천천히' 마름모 / 글자 | 3 / 36 | 흰색 |
| A·B 분기 글자 | 2 | 흰색 |
| 주차칸 (500×600 × 6) | 1 | 흰색 |
| 출발선 (흰 체커 / 검정 띠) | 13 / 4 | 흰색 / 검정 |

- 좌표: 원점 = 매트 좌하단, +x = 도면 오른쪽, +y = 도면 위쪽, z = 0 이 매트 윗면 (Isaac: m, Z-up).
- **LIMO 시작 자세**: 출발선 체크무늬 가운데 **(1.010, 5.580) m, yaw 0° (+x)**. 방향은 같은 차로에서 가장 가까운
  정지선·마름모 쪽으로 자동 결정. 트랙 USD customData `autorace:start_pose` 에 저장됨.
- 노면표시 = 매트 위 1 mm 평면 메쉬 (충돌 없음, 시맨틱 라벨 `class=<분류>`). 바닥 충돌 = 무한 평면.
- 바닥 마찰 기본값: 정지 0.8 / 운동 0.7.

---

## 2. A. `autorace_2026` 확장 (WeGo LimoIsaacSIM 기반)

`limo_ros2_differential` 을 복사해 만든 확장. **차량·센서·OmniGraph 는 원본 그대로**이고 바뀐 것은:

- 기본 트랙(`LIMO_simulation_table.usd`, scale 0.01) → 대회 트랙 USD (스케일 1, 원점 그대로)
- LIMO 시작 위치 → 출발선 위, 진행 방향
- 토픽 `/sim/...` → 실차 이름 (아래 표)
- 조명 위치 → 트랙 가운데 위, 바닥 마찰 → extension 설정으로

### 2-1. 처음 한 번 준비

```bash
# 1) 저장소 (이미 받아 둠: ~/LimoIsaacSIM, 브랜치 autorace_2026)
cd ~/LimoIsaacSIM && git branch          # * autorace_2026

# 2) WeGo LIMO USD (이미 풀어 둠: ~/LimoIsaacSIM/USD/WegoLimo/)
#    다른 PC 라면 WeGo 위키에서 WegoLimoUSD.zip 을 받아서:
#    https://goofy-pleasure-a84.notion.site/Limo-Isaac-Sim-1ff09b2ac4bf8086bdf5e94521f74946
unzip WegoLimoUSD.zip -d ~/LimoIsaacSIM/USD/
ls ~/LimoIsaacSIM/USD/WegoLimo/Limo/limo_diff_thin.usd   # 있어야 함
```

### 2-2. 실행

**명령줄로 바로:**

```bash
~/isaacsim/isaac-sim.sh --ext-folder ~/LimoIsaacSIM --enable autorace_2026
```

**또는 GUI 에서 (위키 방식):**

1. `~/isaacsim/isaac-sim.sh`
2. Window → Extensions → ☰ → Settings → `+` 로 `~/LimoIsaacSIM` 추가
3. Third Party 에서 **autorace_2026** Enable
4. Window → Examples → Robotics Examples → **Wego → AutoRace 2026** → **Load**
5. (정지 버튼 후) **Play** → 시뮬레이션 시작

Linux 의 `isaac-sim.sh` 는 ROS2 브리지(`isaacsim.ros2.bridge`)를 자동으로 켠다.

**바닥 마찰 바꾸기** (Load 할 때 적용):

```bash
~/isaacsim/isaac-sim.sh --ext-folder ~/LimoIsaacSIM --enable autorace_2026 \
    --/exts/autorace_2026/floor_static_friction=0.6 \
    --/exts/autorace_2026/floor_dynamic_friction=0.5
```

| 설정 (`config/extension.toml` `[settings]`) | 기본값 | 설명 |
|---|---|---|
| `track_usd` | `""` | 비우면 `data/track/autorace_2026_track.usda` |
| `floor_static_friction` | 0.8 | |
| `floor_dynamic_friction` | 0.7 | |

### 2-3. 토픽 (Play 후, 다른 터미널)

```bash
ros2 topic list
ros2 topic hz /odom
sudo apt install ros-humble-teleop-twist-keyboard           # 키보드 조종 패키지 (처음 한 번)
ros2 run teleop_twist_keyboard teleop_twist_keyboard        # 키보드 조종
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"   # 직진
```

| 원본 | autorace_2026 | 타입 |
|---|---|---|
| `/sim/cmd_vel` | `/cmd_vel` | geometry_msgs/Twist (구독) |
| `/sim/odom` | `/odom` | nav_msgs/Odometry (odom → base_link) |
| `/sim/imu` | `/imu` | sensor_msgs/Imu |
| `/sim/scan` | `/scan` | sensor_msgs/LaserScan |
| `/sim/camera/color/image_raw` | `/camera/color/image_raw` | sensor_msgs/Image |
| `/sim/camera/depth/image_raw` | `/camera/depth/image_raw` | sensor_msgs/Image |
| `/sim/camera_points` | `/camera/depth/points` | sensor_msgs/PointCloud2 |
| `/sim/camera/camera_info` | `/camera/color/camera_info` | sensor_msgs/CameraInfo |
| (그대로) | `/clock`, `/tf` | |

토픽 이름은 `autorace_2026/autorace_2026_python/autorace_2026.py` 맨 위 `TOPICS` 한 곳에서 바꾼다.
ROS 노드는 `use_sim_time:=true` 로 (시뮬 속도가 실시간과 다를 수 있음).

### 2-4. 원본 그대로라서 실차와 다른 점

- 카메라 frame_id `depth_link` (실차 `camera_color_optical_frame` / `camera_depth_optical_frame`), 1280×720
- 뎁스 32FC1 [m] (실차 Orbbec 드라이버는 16UC1 [mm])
- `/camera/depth/camera_info` 없음 (원본 그래프의 camera_info 헬퍼가 하나 → 컬러 쪽만)
- 라이다는 RPLidar S2E RTX 모델 (실차 LIMO 는 EAI X2L), 3200 점, 0.05–30 m
- 바퀴 충돌체가 실차보다 바깥(y ±0.123)에 있고 `wheelDistance` 0.43 — 원본이 PhysX 에서 스키드 회전이 되도록 맞춘 값

### 2-5. 폴더 구조

```
~/LimoIsaacSIM/
  USD/WegoLimo/                     WeGo 차량·기본 트랙 에셋 (git 미포함, ~300 MB)
  limo_ros2_differential/           원본 예제
  limo_ros2_differential_obs/       원본 예제 (장애물)
  autorace_2026/                    ← 새 확장
    config/extension.toml             확장 정의 + 설정 (마찰 등)
    config/track_build.yaml           트랙 USD 색·마찰 기본값
    autorace_2026_python/
      autorace_2026.py                장면 구성 (트랙, LIMO 시작 자세, OmniGraph, TOPICS)
      extension.py                    Examples 브라우저 등록
    data/track/
      2026Track.dxf                   대회 도면
      track_classification.png        분류 확인 그림 (+ 시작 자세)
      track_elements.json             분류 결과 (mm)
      autorace_2026_track.usda        생성된 트랙
      preview_start.png, preview_overview.png
    tools/                            트랙 재생성 도구
    docs/README.md, CHANGELOG.md
```

---

## 3. 트랙 다시 만들기 (도면이 바뀌었을 때)

```bash
# venv (처음 한 번)
python3 -m venv ~/venvs/autorace
~/venvs/autorace/bin/pip install -r ~/LimoIsaacSIM/autorace_2026/tools/requirements.txt

cd ~/LimoIsaacSIM/autorace_2026
cp /새/도면.dxf data/track/2026Track.dxf
~/venvs/autorace/bin/python tools/dxf_classify.py      # → track_classification.png 로 분류·시작 자세 확인
~/venvs/autorace/bin/python tools/build_track_usd.py   # → data/track/autorace_2026_track.usda
# 색·마찰 기본값은 config/track_build.yaml
```

미분류 영역이 있으면 `dxf_classify.py` 가 에러로 끝난다 (규칙은 파일 위쪽 상수).
같은 도구가 `~/autorace_ws/src/autorace_track/tools/` 에도 있다 (기본 경로만 다름).

---

## 4. B. `autorace_ws` (자체 ROS2 워크스페이스) — 참고

1~3 단계에서 만든 것. 실차와 프레임·뎁스 형식·라이다 사양까지 맞추고 `mode:=sim|real` 로 바꿔 쓰는 구조.

```bash
cd ~/autorace_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch autorace_bringup bringup.launch.py mode:=sim                  # Isaac 창 포함
ros2 launch autorace_bringup bringup.launch.py mode:=sim headless:=true   # 창 없이
ros2 launch autorace_bringup bringup.launch.py mode:=real                 # 실차 (limo_base·ydlidar·astra_camera 필요, 미검증)

# 점검 (다른 터미널 — 새 터미널마다 source 먼저!)
source ~/autorace_ws/install/setup.bash
ros2 run autorace_sim smoke_test            # 토픽 주기 + TF
ros2 run autorace_sim smoke_test --drive    # 직진·회전 시켜서 /odom 확인
ros2 run autorace_sim smoke_test --out /tmp/snap   # color/depth/scan 이미지 저장
```

- 설정: `src/autorace_bringup/config/params_sim.yaml`, `params_real.yaml` (센서 위치 `sensor_mounts`, 바닥 마찰, 시작 자세, 토픽/프레임)
- 실차와 같은 것: 토픽·프레임 이름 전부, 뎁스 16UC1 mm, 카메라 optical 프레임, 라이다 X2L 사양(375 점, 0.12–8 m)
- 구동: LIMO 스키드(축거/윤거 1.14)는 PhysX 마찰로는 거의 안 돌아서, 타이어 힘 모델 + 속도 추종으로 `/cmd_vel` 을 그대로 따라감
- 자세한 내용: `~/autorace_ws/README.md`, `~/autorace_ws/CLAUDE.md`

---

## 5. (참고) 두 시뮬레이터 중 어느 것을 쓰나

차선 추종·실차 이식까지 생각하면 **B (`autorace_ws`)** 가 맞다. 실차 LIMO Pro 와 카메라(640×480, 화각 71°)·토픽·프레임이
같고, `mode:=real` 로 같은 노드를 실차에 그대로 쓴다. A 는 WeGo 원본 차량·센서를 그대로 쓰는 쪽.

---

## 6. 미션 시설물 (규정집 2026-10-08 초안_수정본, 8 개 미션)

배치 한 곳: `~/LimoIsaacSIM/autorace_2026/config/missions.yaml` → `tools/build_mission_usd.py` 로 USD 생성.
A·B 둘 다 이 USD 를 올린다. 규정집에 위치가 없는 것은 도면(정지선·횡단보도·분기)에 맞춘 **추정값**.
배치 그림: `~/LimoIsaacSIM/autorace_2026/data/mission/mission_layout.png`

| 미션 | 시설물 |
|---|---|
| M5 서행 (방지턱) | 1차로 '천천히' 뒤 방지턱 (높이 2 cm, 충돌 있음) + 서행·방지턱 표지판 |
| M6 서행 유도선 | 노면 지그재그 (트랙 USD) |
| M3 보행자 | 오른쪽 횡단보도 앞 보행자 표지판 |
| M7 3차선 장애물 | 정차 LIMO 7 대 (4 줄, 줄마다 차로 변경) |
| M2 갈림길 | 분리대 앞 A(좌회전)/B(우회전) 표지판 — **주기적으로 바뀜** |
| M8 라바콘 | 왼쪽 직선 라바콘 14 개 (부딪히면 넘어짐). 높이 0.202 m (바닥판 0.11 × 0.11 × 0.012 + 몸통 0.19, 아래 반지름 0.045 → 위로 갈수록 좁음). 시뮬 라이다 높이 0.111 m 에서 몸통 지름 약 4.3 cm. 배치는 리셋 때 바꿀 수 있음 (아래) |
| M1 신호등 | 2차로 횡단보도 건너편 신호등 (적 → 녹 → 황, **주기적으로 바뀜**) + 광고 모니터 |
| M4 T자 주차 | 주차 칸 양옆 정차 LIMO 2 대 + P 표지판 |

주기 (시뮬레이션 시간 [s]) 바꾸기:

```bash
# A: 명령줄
~/isaacsim/isaac-sim.sh --ext-folder ~/LimoIsaacSIM --enable autorace_2026 \
    --/exts/autorace_2026/traffic_light_red_s=5 --/exts/autorace_2026/branch_sign_period_s=20 \
    --/exts/autorace_2026/branch_sign_random=true
# B: ~/autorace_ws/src/autorace_bringup/config/params_sim.yaml 의 missions: 를 고친 뒤 bringup 다시 실행
#    traffic_light_s: {red: 10.0, green: 10.0, yellow: 3.0}, branch_sign_period_s: 0.0, branch_sign_random: true …
#    갈림길 표지판 기본 (11-01 부터): 주기 0 + 무작위 = 리셋 (판 시작) 마다 A/B 를 무작위로 정해 그 판 동안 고정 (실제 대회처럼).
#    한쪽으로 고정: branch_sign_random: false + branch_sign_start: "A" 또는 "B". 주기 > 0 이면 예전처럼 그 주기로 바뀜 (시험용)
#    시설물 없이 트랙만: missions.enabled: false
#    라바콘 배치 (M8): missions.cones.layout = default (USD 그대로, S) | straight | curve (한쪽으로 휨) | s (큰 S) | narrow (곧게 점점 좁아짐,
#      콘 바닥판 사이 0.43 → 0.31 m) | wide (쌍 간격 0.33 m) | jitter (default 에 콘마다 ±3 cm, 리셋마다 새로) | random
#      판 사이에 바꾸기: ros2 topic pub --once /sim/cone_layout std_msgs/msg/String "{data: s}" → 다음 /sim/reset 부터
#      /sim/cone_status (1 Hz): 이번 배치 자리에서 라바콘이 밀린 최대 거리, 3 cm 넘게 밀린 수, 넘어진 수, 위치
```

---

## 7. 슬라이딩 윈도우 차선 추종 (LIMO Pro, `autorace_ws/src/autorace_lane`)

[SEAME 2026 Hackathon](https://github.com/mark2455/2026_SEAME_Hackathon) (team chita, Apache-2.0) 의
`D-Racer-Kit/src/perception/perception/lane_detector.py` 슬라이딩 윈도우(SW) 차선 인식과 `decision` 의 조향 PID 를 옮긴 것.
원본 README 의 Out 코스 "상시 SW 코리도" 경로 (양쪽 실선, 주행 내내 SW) 를 그대로 쓰고, 알고리즘·파라미터 이름·기본값도 원본과 같다.

**처리 순서** (`sliding_window.py`): 영상 하단 65 % ROI → BEV 워프 → HSV 마스크 (노랑) → MORPH_OPEN
→ ① 정지선 행 제거 (가로 점유율 45 % 초과 행) ② 시드 = 직전 피팅 하단 + 하단 히스토그램 피크
③ 상자 9 개 체인 (상자당 이동 ≤ 35 px, 빈 상자는 직전 추세 유지) ④ 2차 피팅 + 게이트 (적중 상자 ≥ 3, 픽셀 ≥ 50, 잔차 ≤ 14 px)
⑤ 차폭 간격 두 선 → 중점 / 한 선 → 직전 중심에 가까운 쪽으로 반차폭 이동 → 밴드 4 개 가중 합성 + look-ahead 블렌딩
→ offset [-1, 1] (EMA). SW 락이 실패한 프레임은 원본처럼 guided band (밴드별 좁은 창 평균) 로 폴백.

**조향** (`lane_follower.py`, 원본 `state_machine._lane_steer` 와 같은 규칙): 목표 = offset + 0.5 × 곡률 → PID (kp 0.6, kd 0.15)
→ `/cmd_vel` angular.z = −3.0 × PID × (v / 0.3), linear.x = 0.3 × (1 − 0.5 × |곡률|) (하한 0.2).
차선을 놓치면 직전 조향을 유지하며 3 s 시정수로 직진 쪽으로 (원본), 20 s 계속 못 찾으면 정지 (LIMO 안전용으로 추가).

원본에서 뺀 것: D-Racer 트랙 전용 회전교차로·갈림길·노랑 래치·ArUco·신호등(YOLO) 로직. **차선 추종만** 한다 — 미션(정지·회피·주차)은 아직 없다.

### 7-1. 빌드 (처음 한 번, 새 패키지라 필요)

```bash
cd ~/autorace_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 7-2. 시뮬에서 실행 (B)

```bash
# 터미널 1: 시뮬 (트랙 + LIMO + 시설물)
source ~/autorace_ws/install/setup.bash
ros2 launch autorace_bringup bringup.launch.py mode:=sim

# 터미널 2: 차선 추종 (시뮬이 다 뜨고 /camera/color/image_raw 가 나오면)
source ~/autorace_ws/install/setup.bash
ros2 launch autorace_lane lane_follow.launch.py              # LIMO 를 출발선으로 되돌린 뒤 시작 (use_sim_time 기본 true)

# 터미널 3: 인식 화면 (왼쪽 BEV + 상자(보라)·피팅(주황)·차선 중심(빨강)·화면 중심(초록), 오른쪽 마스크)
ros2 run rqt_image_view rqt_image_view /lane/debug
```

**다시 달리기 (Isaac 은 끄지 않는다)** — 터미널 2 를 Ctrl+C 하고 같은 명령을 다시 치면 된다.
launch 가 먼저 `/sim/reset` 을 불러 LIMO 를 출발 자세로, 넘어진 라바콘을 처음 자리로, 신호등·갈림길·광고판 주기를 0 초로 되돌린 뒤
차선 추종을 띄운다 (로그 `[reset_sim] 완료: …`).
**리셋이 실패하면 차선 추종을 띄우지 않고 끝난다** (차가 엉뚱한 자리에서 출발하지 않게). 로그에 이유와 할 일이 찍힌다:

| 로그 | 할 일 |
|---|---|
| `시뮬은 떠 있지만 리셋 서비스가 없는 옛 버전입니다` | 2026-10-26 밤 이전에 띄운 시뮬. 터미널 1 Ctrl+C → bringup 다시 |
| `시뮬(/clock)이 안 보입니다` | 터미널 1 이 안 떠 있거나 `ROS_DOMAIN_ID` 가 다름 (`echo $ROS_DOMAIN_ID`, 둘 다 30) |
| `시뮬이 정지(■) 상태` | Isaac 창에서 ▶ |

```bash
ros2 run autorace_sim reset_sim                               # 리셋만 (노드를 켜 둔 채로도 된다 — 출발선에서 선을 다시 잡고 달림)
ros2 launch autorace_lane lane_follow.launch.py reset_sim:=false   # 리셋 없이 지금 자리에서 이어서
```

- Isaac 창의 ■(정지) → ▶(재생) 도 처음 상태로 돌아간다 (이때는 시뮬 시간도 0 초부터). ‖(일시정지) → ▶ 는 그 자리에서 이어서.
- 리셋 뒤 `/odom` 은 0 으로 돌아온다 (시뮬 odom = 출발 자세 기준). 시뮬 시간(`/clock`)은 리셋해도 계속 흐른다.

- 시설물을 켠 채로 돌리면 3차선의 정차 LIMO 와 왼쪽 직선의 라바콘에 **부딪힌다** (회피 없음).
  차선 추종만 보려면 `params_sim.yaml` 의 `missions.enabled: false`.
- 노드는 2 초마다 `fps, 차선 %, SW 락 %, offset, curv, 차폭 px → v, w` 를 찍는다.
- 2차로 끝(주차장 옆)은 막다른 길이라 차선 추종만으로는 오른쪽 차로로 넘어가 다음 바퀴를 돈다 (주차 미션 몫).

### 7-3. 실차 LIMO Pro 에서 실행

토픽은 실차와 같다: 카메라 `/camera/color/image_raw` (Orbbec Dabai, rgb8) → `/cmd_vel` (limo_base).

```bash
# LIMO Pro 위에서 (차량 쪽: limo_base + 카메라. autorace_ws 를 LIMO 에 옮겨 빌드했다면)
ros2 launch autorace_bringup bringup.launch.py mode:=real
#   (또는 LIMO 기본: ros2 launch limo_bringup limo_start.launch.py + ros2 launch astra_camera dabai.launch.py)

# 1) 먼저 인식만 — 바퀴 안 돈다. 노트북에서 /lane/debug 를 보며 노란 선이 마스크에 잡히는지, 빨간 중심선이 차로 가운데인지 확인
ros2 launch autorace_lane lane_follow.launch.py use_sim_time:=false enable:=false
# 2) 주행
ros2 launch autorace_lane lane_follow.launch.py use_sim_time:=false
# 끄면 노드가 /cmd_vel 0 을 한 번 보내고 끝난다. 영상이 0.5 s 끊겨도 정지.
```

**실차에서 꼭 다시 맞출 것** (`~/autorace_ws/src/autorace_lane/config/lane_follower.yaml`, `--symlink-install` 이라 고치고 노드만 다시 켜면 됨):

1. **색** `yellow_hsv_lo / hi` — 지금 값은 Isaac 렌더에서 잰 것 (H 18–26). 실제 테이프·조명에서 다시 잰다:
   ```bash
   ros2 run autorace_sim smoke_test --out /tmp/snap     # 실차 카메라 한 장 → /tmp/snap/color.png
   ros2 run autorace_lane lane_replay /tmp/snap --out /tmp/lane_dbg   # 마스크·SW 결과 그림
   ```
2. **카메라 기하** `camera_height` (바닥→렌즈 [m]), `camera_pitch_deg` (아래로 +) — 재서 넣으면 BEV 점이 다시 계산된다
   (시작 로그 `birdeye_src_ratio = [...]`). 직선 차로에서 BEV 의 두 선이 세로로 나란하고, 노드 로그의 `W` 가 ≈ 192 px 이면 맞다.
3. **속도·조향** `linear_speed` 0.3 → 처음엔 0.2 로. 코너에서 바깥으로 밀리면 `curve_steer_bias`·`angular_scale` ↑, 안쪽 선을 밟으면 ↓.
   LIMO Pro 를 Ackermann 모드로 쓰면 limo_base 가 각속도를 조향각으로 바꾼다 (같은 `/cmd_vel`).

영상을 녹화해 두었다가 노트북에서 돌려 볼 수도 있다: `lane_replay <동영상.mp4|사진 폴더> [--params 다른.yaml] [--every 5]`.

### 7-4. 시뮬 확인 결과 (Isaac 5.1 headless, 시설물 끔, 기본 설정)

| 구간 | 결과 |
|---|---|
| 출발 → 1차로 → 오른쪽 위 코너 → 지그재그(M6) → 횡단보도 → 3차선(A 차로) → 왼쪽 직선 → 2차로 끝 | 한 바퀴 약 110 s, **바퀴가 선에 닿은 적 없음** (선까지 최소 0.11 m) |
| SW 락 | 2 초 구간 75 개 중 69 개에서 100 % (나머지는 코너·3차선 가운데처럼 한 선만 보이거나 안 보일 때) |
| 방지턱 (시설물 켬) | 0.29 m/s 로 넘음 (차체 9.5 mm 오르내림, 걸림 없음) — 서행 표지판 인식은 아직 없음 |
| 다시 달리기 (10-31, 시설물 켬) | 실행 → Ctrl+C → 다시 실행 4 번 연속: 매번 출발선 리셋, 방지턱 지나 차로 가운데 유지 (W 179–211 px), Traceback 없이 종료 |

시험하면서 알게 된 것:
- 흰색 마스크를 켜면 출발선 체커·'천천히' 글자를 선으로 물어 트랙 밖으로 나간다 → 노랑만.
- 원본 BEV (320 폭, 좌우 ±0.43 m) 로는 3차선 구간에서 노란 선이 시야 밖으로 빠진다 → 480 폭 + dst 0.3–0.7 (차로 폭 192 px 그대로).
- 1.0 m 앞까지 보면 R≈0.85 m 코너 안쪽 선을 밟는다 → 0.37–0.75 m.
- `curve_steer_bias` 0 / 0.2 / 0.35 는 왼쪽 위 코너에서 2차로로 못 꺾었다 (1차로 끝이 정면에 보이는 곳) → 0.5.
- 다시 달릴 때 방지턱에서 흔들린 뒤 차로 폭 학습값이 100 px 까지 줄어 엉뚱한 선 쌍을 따라 차로 사이로 달린 적이 있다 (원본은 하한 96 px 만 있음)
  → `lane_width_tol: 0.25` (폭 학습을 144–240 px 안에서만). 원본 동작으로 돌리려면 0.0.

### 7-5. 파일

```
~/autorace_ws/src/autorace_lane/
  autorace_lane/sliding_window.py   검출기 (원본 lane_detector.py 의 SW·guided band·BEV·밴드 합성)
  autorace_lane/pid.py              원본 decision/pid.py 그대로
  autorace_lane/lane_follower.py    ROS2 노드 (영상 → 검출 → 조향 → /cmd_vel, /lane/debug)
  scripts/lane_follower, scripts/lane_replay
  config/lane_follower.yaml         파라미터 (이 트랙·LIMO 에 맞춘 값과 이유가 주석에)
  launch/lane_follow.launch.py      use_sim_time, enable, params_file, reset_sim (시뮬에서 시작 전 리셋, 기본 true — 실패하면 차선 추종 안 띄움)
  LICENSE.D-Racer-Kit               원본 Apache-2.0 라이선스
~/autorace_ws/src/autorace_sim/
  isaac/run_sim.py                  /sim/reset 서비스 (std_srvs/Trigger), ■→▶ 재시작 처리
  autorace_sim/reset_sim.py         ros2 run autorace_sim reset_sim (실패하면 이유를 찍고 exit 1)
```

---

## 8. 미션 동작 (v3 「미션 해결 방식과 진행 방향」 순서) — 지금: M5 서행 → M6 지그재그 → M3 보행자 정지 → M7 3차선 회피 → M2 갈림길 → M8 라바콘 → M1 신호등 → M4 주차

v3 문서 4장 구조에서 주행 순서대로 M5·M6·M3·M7·M2·M8·M1·M4 에 필요한 것만 만들었다:

```
컬러 → yolo_detector ──→ /perception/detections ─┐
뎁스 → depth_obstacle ─→ /perception/obstacles ──┤
라이다 → lidar_processor → /perception/cones ────┤
컬러 + 신호등 박스 → traffic_light → /perception/traffic_light ┤   (박스: 시뮬은 fake_traffic_light, 실차는 YOLO traffic_light 클래스)
컬러 → lane_follower ──→ /lane/state ────────────┼→ mission_manager (FSM) ─→ /mission/lane_cmd → lane_follower → /lane/cmd_vel ─┐
       (선 피팅 잔차, 정지선, 노란 선 쪽)  /odom ──┘        │  /mission/state                                            ├→ cmd_vel_mux → /cmd_vel
                                                            └→ /mission/cmd_vel (M7·M8 동안 직접 운전) ──────────────────┘
```

`cmd_vel_mux` 는 `/mission/state` 가 `M7_AVOID`·`M8_AVOID` 일 때만 미션 매니저 명령을, 그 밖에는 차선 추종 명령을 차로 보낸다 (두 노드가 동시에 차를 움직이지 않게).
`/cmd_vel` 을 낼 때마다 출처 (`lane` | `mission` | `…+guard` = 라이다 안전 정지가 v 를 0 으로 바꿈 | `zero:…` = 입력 끊김) 를 `/cmd_vel_mux/selected` 로 같이 내고,
2 s 마다 `[mux] 고름 …` 요약 (넘김·버림 수, 입력 나이), 정지 명령 시작·끝 (출처와 함께), 입력 끊김을 로그로 남긴다.
**라이다 안전 정지** (M7·M8 동안, 뎁스·미션 판단과 별개, 마지막 단계): `/scan` 에서 차 앞 통로 (차 폭 + 2 cm) 범퍼 10 cm 안에 점이 있으면 전진 (v > 0) 을 0 으로,
뒤 8 cm 안이면 후진을 0 으로 — 회전은 늘 통과. 라이다 최소 거리 (0.12 m) 때문에 범퍼에 붙은 물체는 안 보이므로, 한 번 막으면 3 cm 물러나거나 15° 돌 때까지 막아 둔다.
`odom_fusion`·`parking_controller` 는 아직 없다 — 거리는 휠 오도메트리(`/odom`) 만 쓴다 (시뮬 오도메트리는 정확. 실차는 IMU 를 섞어야 할 수 있음).

| 상태 (`/mission/state`) | 켜지는 조건 | 하는 일 | 끝나는 조건 |
|---|---|---|---|
| `LANE` | — | 차선 추종 0.3 m/s | — |
| `M5_SLOW` | YOLO `slow`·`speed_bump` | 0.15 m/s | 표지판 마지막으로 본 뒤 1.5 m |
| `M6_SLOW` | 선 피팅 잔차 ≥ 7.5 px (톱니 모양 지그재그 선) | 0.15 m/s (조향은 차선 추종 그대로) | 잔차 < 5 px 가 된 뒤 0.3 m |
| `M3_APPROACH` | YOLO `crosswalk` (보행자 표지판) | 0.15 m/s 로 정지선 찾기 → 보이면 남은 거리 비례 감속 | 앞바퀴가 정지선에 닿음 |
| `M3_STOP` | ↑ | 정지 | 완전 정지 후 4 s |
| `M3_GO` | ↑ | 발진 | 0.5 m 지나면 `LANE` (그 뒤 3 m 동안 보행자 표지판 무시) |
| `M7_AVOID` | 뎁스: 내 차로 앞 1.2 m 안에 폭 12 cm 넘는 장애물 (코너에서 도는 중·노란 선이 양쪽 다 보이는 1차로 도로에서는 안 켜짐) | 미션 매니저가 직접 운전: 막히면 빈 옆 차로로 차로 변경 (0.15 m/s, 바꾸는 동안 0.10) | 내 차로 앞 1 m 가 비고 변경이 끝나면 `LANE` (슬라이딩 윈도우 리셋) |
| `M8_AVOID` | M2 뒤, 라이다 라바콘이 앞 1.5 m 안 양쪽에 4 개 이상 3 번 연속 | 미션 매니저가 직접 운전: Follow-the-Gap — 통로를 가로지르는 두 콘 사이 틈 (빈 폭 ≥ 0.32 m) 의 가운데점을 이은 길을 따라감, 0.15 m/s (좁으면 0.10). 마지막 틈 뒤는 통로 출구선 | 앞 1 m 안에 라바콘이 없고 다 0.4 m 지남 → 출구선에 맞춘 뒤 `LANE` (윈도우 리셋, 1.5 s 0.15 m/s), 이후 라바콘 무시 |
| `M1_APPROACH` | M8 뒤, `/perception/traffic_light` 색 (크기 확인된 것) 5 번 연속 | 0.15 m/s 로 정지선 찾기 → M3 와 같은 코드로 남은 거리 비례 감속. 크기 확인된 초록 8 번 연속이면 서지 않고 `M1_GO` | 앞바퀴가 정지선에 닿음 → `M1_STOP` |
| `M1_STOP` | ↑ | 정지 유지 (빨강·노랑·안 보임·애매 모두). 진행 감시 없음. 2 s 마다 판단 로그 | 크기 확인된 초록 8 번 연속 → `M1_GO` |
| `M1_GO` | ↑ | 발진. 정지선 지나 0.3 m 까지 0.15 m/s 서행. 앞바퀴가 선에 닿기 전에 노랑·빨강 3 번 → `M1_APPROACH` (정지선에 다시 섬) | 정지선 0.3 m 지나면 `M4_PARKING` + `/parking/start` (std_msgs/Empty) |
| `M4_PARKING` | M1 끝 | 차선 추종으로 가다가 `parking_controller` 가 주차 구역 (앞옆 LIMO) 에서 넘겨받아 주차·탈출 (mux 가 `/parking/state` 를 보고 `/parking/cmd_vel` 을 고름) | `/parking/done` → `DONE` (멈춤) |
| (M2 — 따로 상태 없음) | 갈림길 표지판 A/B 3 장 연속 → 결과 기억 (바뀌면 마지막 것) | `M7_AVOID` 가 3차선 끝 차로를 표지판 쪽으로 고름 (A = 맨 왼쪽, B = 맨 오른쪽) | M7 이 끝난 차로 = 들어간 길 (로그 `[M2] … 길로 들어감`) |

우선순위: 정지 미션(M3) > 서행(M5·M6) > LANE — 지그재그 서행 중에 보행자 표지판이 보이면 M3 로 넘어간다.

| 노드 | 패키지 | 받는 것 | 내는 것 |
|---|---|---|---|
| `yolo_detector` | `autorace_perception` | `/camera/color/image_raw` | `/perception/detections` (autorace_msgs/DetectionArray), `/perception/debug` (박스 그린 영상. 갈림길 보조 박스는 파랑) |
| `mission_manager` | `autorace_mission` | `/perception/detections`, `/perception/obstacles`, `/lane/state`, `/odom` | `/mission/state` (std_msgs/String), `/mission/lane_cmd` (autorace_msgs/LaneCmd), `/mission/cmd_vel` (M7) |
| `lane_follower` | `autorace_lane` | 영상 + `/mission/lane_cmd` | `/lane/cmd_vel` (drive.launch 일 때. lane_follow.launch 단독이면 `/cmd_vel`) — lane_cmd 의 `max_speed` = 속도 상한, `stop` = 정지, `reset_window` = 윈도우 리셋. `/lane/state` (autorace_msgs/LaneState: 중심 오차, 차선 방향, 노란 선 쪽, 선 피팅 잔차, 정지선 거리) |
| `depth_obstacle` | `autorace_perception` | `/camera/depth/image_raw`, `/camera/depth/camera_info` | `/perception/obstacles` (autorace_msgs/ObstacleArray, base_link 기준 상자), `/perception/obstacle_markers` (RViz 주황 상자) |
| `traffic_light` | `autorace_perception` | `/camera/color/image_raw`, `/camera/color/camera_info`, `/perception/traffic_light_detections`, `/lane/state`, `/odom` | `/perception/traffic_light` (autorace_msgs/TrafficLight: 색, 대비, 박스, 크기 거리·예상 거리, 크기 확인 여부, 후보·버린 수, 이유), `/perception/traffic_light_debug` (고른 박스 초록, 버린 박스 빨강 + 이유) |
| `fake_traffic_light` | `autorace_sim` | camera_info, 영상, `/odom` | 시뮬 전용 (YOLO 신호등 모델이 생기기 전): 시뮬이 아는 신호등 몸체 앞면을 카메라에 투영한 박스 (`decoy:=true` 면 모니터 그림 자리에도 박스 — YOLO 가 모니터를 잘못 잡는 경우 흉내) |
| `lidar_processor` | `autorace_perception` | `/scan`, `/odom` | `/perception/cones` (geometry_msgs/PoseArray, odom 좌표 라바콘 기억), `/perception/cone_markers` (RViz 주황 원기둥) |
| `cmd_vel_mux` | `autorace_mission` | `/lane/cmd_vel`, `/mission/cmd_vel`, `/mission/state`, `/scan` | `/cmd_vel`, `/cmd_vel_mux/selected` |

YOLO 모델: [github.com/mark2455/2026_Autorace](https://github.com/mark2455/2026_Autorace) `yolo-training/models/best.pt`
(YOLOv8n 640, 클래스 `crosswalk, parking, slow, speed_bump, turn_a, turn_b`) 를 `autorace_perception/models/autorace_yolov8n.pt` 로 복사해 씀.
메시지 정의: `autorace_msgs/msg/` (`LaneCmd` 에는 이후 미션용 칸 — 따라갈 선, 목표 오프셋, 파라미터 세트, 흰색, 윈도우 리셋 — 도 미리 있고 아직 무시됨).

### 8-1. 처음 한 번

```bash
pip3 install ultralytics          # YOLO (이 PC 는 이미 있음: ultralytics 8.4, torch 2.14 CUDA)
cd ~/autorace_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install    # 새 패키지 3 개 (autorace_msgs, autorace_perception, autorace_mission)
                                  # M2 갈림길 (표지판 그림·branch_sign.py 새 파일) 받은 뒤에도 한 번 더
```

### 8-2. 실행 — 터미널 2 개

새 터미널마다 먼저 `source ~/autorace_ws/install/setup.bash`.

```bash
# 터미널 1: 시뮬 — 차는 출발선에 멈춰 있다 (/cmd_vel 을 내는 노드가 없으면 안 움직인다)
ros2 launch autorace_bringup bringup.launch.py mode:=sim
```

```bash
# 터미널 2: 주행 — YOLO + 미션 매니저 + 차선 추종을 한 번에, 로그도 이 터미널에 같이
ros2 launch autorace_mission drive.launch.py
```

터미널 2 에서 일어나는 순서와 로그:

| 순서 | 로그 |
|---|---|
| 1. 같은 노드가 이미 떠 있는지 확인 (다른 창에 남은 차선 추종이 `/cmd_vel` 을 같이 내는 것 방지) | `[check_nodes] 겹치는 노드 없음` — 있으면 `이미 떠 있는 노드: …` 를 찍고 시작하지 않음 |
| 2. YOLO·뎁스 장애물·속도 선택기 켜기 | `[drive] YOLO 모델 올리는 중 … (몇 초)` |
| 3. YOLO 준비되면 출발선으로 리셋 | `[drive] YOLO 준비됨 → 출발선으로 리셋`, `[reset_sim] 완료: …` |
| 4. 미션 매니저 + 차선 추종 출발 (리셋 뒤에 켜야 지난번 멈춘 자리의 장애물을 보고 미션을 켜지 않는다) | `[drive] 리셋 완료 → 미션 매니저 + 차선 추종 출발` |
| 서행 | `[mission_manager] … [상태] LANE → M5_SLOW (slow 0.52 박스 높이 39px …)`, `[lane_follower] … v 0.15 … (상한 0.15)` |
| 서행 해제 | `[mission_manager] … [상태] M5_SLOW → LANE (표지판 마지막으로 본 뒤 1.50 m …)`, `[lane_follower] … v 0.30` |
| 지그재그 서행 | `[상태] LANE → M6_SLOW (선 피팅 잔차 8.0px, 5 장 연속 …)` |
| 보행자 표지판 | `[상태] M6_SLOW → M3_APPROACH (crosswalk 0.78 박스 높이 45px …)` |
| 정지선 잡음 | `[M3] 정지선 보임: 카메라에서 0.734 m (덮은 비율 1.00), 앞바퀴까지 0.698 m …` |
| 횡단보도 뒤 가로선 | `[M3] 다른 가로선 무시: 측정 0.711 m, 잡은 정지선 예상 0.174 m` (정상 — 처음 잡은 선만 따라감) |
| 정지 | `[상태] M3_APPROACH → M3_STOP (…)`, `[M3] 완전 정지: 앞바퀴 축 → 정지선 가까운 끝 +0.0 cm (…), 4.0 s 대기 시작` |
| 발진 | `[상태] M3_STOP → M3_GO (정지 4.0 s 뒤 발진)`, `[상태] M3_GO → LANE (…)` |
| 3차선 들어감 | `[M7] 기억한 장애물 (…)`, `[상태] LANE → M7_AVOID (… 차로 방향 -180° …, 차로 격자 정차 LIMO 6 대 (차로 3 개), 지금 차로 1 …)`, `[mux] lane → mission` |
| 정렬 | `[M7] 차로 방향 정렬 시작: 방향 오차 +11.4°`, `[M7] 차로 방향 정렬 끝: 방향 오차 -2.4° (1.4 s, IMU)` |
| 차로 변경 | `[M7] 차로 1 → 0: 내 차로 앞 0.71 m 막힘, 차로 0 는 그 장애물 지나고 0.41 m 비어 있음 (s 0.19 m)` (차로 번호 0 = 맨 왼쪽) |
| 운전 방식 (바뀔 때만) | `[M7] 제자리 회전 → +45° (그 방향 1.00 m 빔)`, `[M7] 대각 +45° 로 차로 0 쪽 (…)`, `[M7] 직진 (앞 0.68 m 빔, 차로 0)` |
| 막힘 → 후진 | `[M7] 막힘: 2 s 동안 갈 방향 없음 → 후진 0.15 m (같은 자리 1/3 번째, 뒤 라이다 빈 거리 …)`, `[M7] 후진 끝: 0.15 m 물러남 → 차로 다시 고르기` |
| mux | `[mux] lane → mission (상태 M7_AVOID)`, `[mux] 정지 명령 시작 — 출처 mission …`, 2 s 마다 `[mux] 고름 mission … lane 넘김 0/버림 42, mission 넘김 40/버림 0 …`, 라이다가 막으면 `[mux] 라이다 안전 정지 (앞): 범퍼에서 0.08 m — mission 의 v +0.15 를 0 으로 …` |
| 갈림길 표지판 | `[M2] 갈림길 표지판 B (turn_b 0.69, 박스 높이 15px, 3 장 연속, …) → 오른쪽 길` (3차선 입구, 표지판 약 4 m 앞). 달리는 중에 바뀌면 `(전에 본 A 에서 바뀜 — 마지막 것을 따름)` |
| 갈림길 잠금 | `[M2] 분기점 가까움 (표지판 박스 47px ≥ 45px) → A 로 잠금` |
| 3차선 끝 | `[상태] M7_AVOID → LANE (앞 1.0 m 비어 있음, 차로 변경 5 번, 지금 차로 2 …)`, `슬라이딩 윈도우 리셋`, `[mux] mission → lane`, `[M2] 표지판 B → B (오른쪽) 길로 들어감 (차로 2)` |

YOLO 가 준비된 뒤에 출발하는 이유: 출발하자마자 (약 0.8 m 앞) 서행 표지판이 나온다. YOLO 모델을 올리는 데 몇 초 걸린다.

| 하고 싶은 것 | 명령 (터미널 2) |
|---|---|
| **다시 달리기** | Ctrl+C → `ros2 launch autorace_mission drive.launch.py` (모든 노드가 처음부터, 차는 출발선으로) |
| 리셋 없이 지금 자리에서 | `ros2 launch autorace_mission drive.launch.py reset_sim:=false` |
| 인식·판단만 (바퀴 안 돌림) | `ros2 launch autorace_mission drive.launch.py enable:=false` |
| 실차 | `ros2 launch autorace_mission drive.launch.py use_sim_time:=false` (리셋 없음) |

더 보고 싶을 때 (다른 터미널):

```bash
ros2 run rqt_image_view rqt_image_view /perception/debug   # YOLO 박스 (라벨·신뢰도·박스 높이 px)
ros2 topic echo /mission/state
ros2 topic echo /mission/lane_cmd
ros2 topic echo /perception/detections
ros2 topic echo /lane/state --field line_resid_px            # 지그재그 판단값
ros2 topic echo /lane/state --field stop_line_dist_m         # 정지선까지 거리 (안 보이면 -1)
ros2 topic echo /perception/obstacles                        # 뎁스 장애물 (base_link 기준 앞·옆 범위)
ros2 topic echo /cmd_vel                                     # 차에 실제로 들어가는 명령 (mux 뒤)
```

노드 하나만 따로 띄워 디버깅할 때는 각 패키지 launch 를 쓴다 (`drive.launch.py` 는 끄고):
`ros2 launch autorace_perception yolo_detector.launch.py` · `ros2 launch autorace_mission mission_manager.launch.py` ·
`ros2 launch autorace_lane lane_follow.launch.py` (§7-2, 리셋 포함). 이때는 YOLO·미션 매니저를 먼저 띄우고 차선 추종을 마지막에.

실차: 젯슨에서 YOLO 가 느리면 `yolo_detector.yaml` 의 `max_hz` 를 낮춘다 (NCNN 변환은 나중).

### 8-3. 확인 화면 — 차선 인식 · YOLO 박스 · RViz2

터미널 1 (시뮬)·2 (drive) 를 띄운 뒤 **다른 터미널**에서. 새 터미널마다 먼저 `source ~/autorace_ws/install/setup.bash`.

#### 차선 인식 화면 (가장 간단)

```bash
ros2 run rqt_image_view rqt_image_view /lane/debug
```

| 위치·색 | 뜻 |
|---|---|
| 왼쪽 그림 | 위에서 내려다본 영상 (BEV, 카메라 앞 0.37–0.75 m) |
| 보라 네모 | 슬라이딩 윈도우 상자 (아래에서 위로 9 개) |
| 주황 곡선·점 | 상자로 찾은 선을 곡선으로 맞춘 것 |
| 빨간 세로선 | 찾은 차선 가운데 |
| 초록 세로선 | 화면 가운데 (= 차 가운데). 빨강과 초록이 겹치면 차로 가운데로 가고 있는 것 |
| 위 글자 | `off` 가운데에서 벗어난 정도 (−1~1), `curv` 곡률, `SW` = 슬라이딩 윈도우로 잡음 / `band` = 놓쳐서 대신 찾음, `W` 차로 폭 px (≈ 192 이 정상), `r` 선 피팅 잔차 px (지그재그에서 8 이상, 평소 2~3) |
| 하늘색 가로선 | 정지선(흰 가로줄)의 가까운 쪽 끝 — 이 위치로 정지선까지 거리를 잰다 |
| 오른쪽 그림 | 노란색 마스크 (흰 = 노란 선으로 본 픽셀) |

YOLO 박스는 같은 창 왼쪽 위 토픽 칸에서 `/perception/debug` 로 바꾸거나, 따로:

```bash
ros2 run rqt_image_view rqt_image_view /perception/debug
```

초록 박스 + `라벨 신뢰도 h박스높이px` (예 `slow 0.67 h36`). 박스 높이가 `m5_min_box_px` (20) 이상이어야 서행이 켜진다.

#### RViz2

```bash
rviz2 --ros-args -p use_sim_time:=true
```

`use_sim_time:=true` 는 시뮬일 때 꼭 — 시뮬 토픽의 시각이 시뮬 시간이라, 없으면 라이다·TF 가 "too old" 로 버려져 안 보인다. 실차는 그냥 `rviz2`.

RViz 창이 뜨면:

1. **Fixed Frame** — 왼쪽 Displays → Global Options → Fixed Frame 칸에 `odom` 입력 (기본 `map` 이면 "Frame [map] does not exist" 빨간 경고).
2. **Add** — 왼쪽 아래 Add 버튼 → 아래 표대로 하나씩. "By topic" 탭은 지금 나오는 토픽 목록에서 고르는 것 (drive 가 떠 있어야 `/lane/debug`·`/perception/debug` 가 보인다).

| 켤 것 | Add 에서 고르는 곳 | 바꿀 설정 | 볼 것 |
|---|---|---|---|
| TF | By display type → `TF` | Show Names 체크 | `odom` → `base_link` → `laser_link`·`camera_link`·`imu_link` 가 이어져 있는지, 차가 움직일 때 같이 가는지 |
| 라이다 | By topic → `/scan` → `LaserScan` | Size (m) `0.03` | 빨간 점 = 정차 LIMO·라바콘·주차 LIMO |
| 오도메트리 | By topic → `/odom` → `Odometry` | Keep `100`, Covariance 체크 해제 | 지나온 자취 (화살표) |
| 카메라 원본 | By topic → `/camera/color/image_raw` → `Image` | | 앞 카메라 그대로 |
| 차선 인식 | By topic → `/lane/debug` → `Image` | | 위 rqt 와 같은 그림 |
| YOLO 박스 | By topic → `/perception/debug` → `Image` | | 표지판 박스 |
| 뎁스 점 (선택) | By topic → `/camera/depth/points` → `PointCloud2` | Size (m) `0.01` | 앞 물체의 3D 점 (M7 장애물용). 무거우면 끈다 |
| 뎁스 장애물 | By topic → `/perception/obstacle_markers` → `MarkerArray` | | 주황 상자 = depth_obstacle 이 찾은 장애물 (정차 LIMO 등) |

3. **차 따라가게** — 오른쪽 Views → Current View → Target Frame 을 `base_link` 로 (기본은 Fixed Frame 고정 시점).
4. **저장** — File → Save Config As → `~/autorace_ws/autorace.rviz`. 다음부터는 이 한 줄로 같은 화면:

```bash
rviz2 -d ~/autorace_ws/autorace.rviz --ros-args -p use_sim_time:=true
```

미션 상태(`LANE` / `M5_SLOW`)는 RViz 에 표시할 수 없는 글자 토픽이라 터미널 2 로그나 `ros2 topic echo /mission/state` 로 본다.

| RViz 증상 | 해결 |
|---|---|
| "Frame [map] does not exist", 아무것도 안 보임 | Fixed Frame → `odom` |
| LaserScan·TF 가 안 보이고 "Message removed because it is too old" | `use_sim_time:=true` 를 빼고 켬 → RViz 끄고 위 명령으로 다시 |
| Image 칸이 "No Image" | `/lane/debug`·`/perception/debug` 는 drive (터미널 2) 가 떠 있어야 나온다. 카메라 원본도 안 나오면 시뮬 확인 |
| 토픽 목록에 아무것도 없음 | RViz 터미널의 `ROS_DOMAIN_ID` 가 시뮬과 다름 (`echo $ROS_DOMAIN_ID`, 30) |

### 8-4. 미션별 켜짐·끝남과 설정

| | 조건 | 설정 (`autorace_mission/config/mission_manager.yaml`) |
|---|---|---|
| 켜짐 LANE → M5_SLOW | `slow` 또는 `speed_bump` 가 3 장 연속, 신뢰도 ≥ 0.5, 박스 높이 ≥ 20 px | `m5_labels`, `m5_min_conf`, `m5_min_box_px` (임시값), `m5_confirm_frames` |
| 동작 | `lane_cmd.max_speed = 0.15` → lane_follower 가 0.5 m/s² 로 서서히 0.15 m/s 까지 (규정: 기본의 1/2, 0.2 이하) | `m5_speed`, (lane_follower) `speed_ramp` |
| 끝남 M5_SLOW → LANE | 표지판을 **마지막으로 본 곳**에서 오도메트리 1.5 m — 방지턱 표지판이 연달아 보이면 그만큼 늦게 끝남 | `m5_release_m` (임시값) |

**M6 서행 유도선 (지그재그)** — 조향은 차선 추종 그대로 (시뮬에서 슬라이딩 윈도우 두 선의 가운데로 잘 간다), 속도만 줄인다.

| | 조건 | 설정 |
|---|---|---|
| 켜짐 LANE → M6_SLOW | `/lane/state` `line_resid_px` ≥ 7.5 px 가 5 장 연속. 톱니 모양 노란 선은 슬라이딩 윈도우 2차 곡선에서 많이 벗어난다 (시뮬 한 바퀴: 지그재그 8.0–8.7, 직선 2.2–2.4, 코너 최대 6.9 px) | `m6_resid_on`, `m6_confirm_frames` |
| 동작 | 0.15 m/s | `m6_speed` |
| 끝남 M6_SLOW → LANE | 잔차 < 5 px 가 된 뒤 0.3 m (지그재그 선이 BEV 에서 사라짐) | `m6_resid_off`, `m6_release_m` (임시값) |

차로 폭 변화(지그재그라 넓어졌다 좁아짐)로도 재 봤지만 일반 구간과 구분이 안 됐다 (최대 ±31 px). 실차에서는 잔차 문턱을 다시 잴 것.

**M3 보행자 표지판 정지**

| | 조건 | 설정 |
|---|---|---|
| 켜짐 → M3_APPROACH | YOLO `crosswalk` (보행자 표지판) 3 장 연속, 신뢰도 ≥ 0.5, 박스 높이 ≥ 30 px (시뮬: 표지판 1.3 m 앞에서 38 px). LANE·서행 중 어디서든 | `m3_labels`, `m3_min_box_px`, `m3_confirm_frames` |
| 정지선 찾기 | 0.15 m/s. lane_follower 가 BEV 흰색 마스크(노랑 뺌)에서 차로 폭의 90 % 이상을 덮는 가로줄 (두께 10 행 이상) 을 찾아 가장 가까운 것의 가까운 끝까지 거리를 낸다 — 차선용 `use_white: false` 와 별개 | (lane_follower) `stop_line_enabled`, `stop_line_min_cov` 0.9, `stop_line_min_rows` |
| 감속 | 앞바퀴까지 남은 거리 = 측정 − 카메라→앞바퀴 0.016 m − 0.02 m. 속도 = 0.5 × 남은 거리 (0.04~0.15 m/s). 선이 BEV 아래(0.37 m)로 사라지면 오도메트리로 이어서. 영상 찍은 순간의 위치와 짝지음 | `m3_kp`, `m3_min_speed`, `m3_cam_to_front_axle_m` (실차 실측), `m3_stop_gap_m` |
| 다른 선 무시 | 처음 잡은 선과 10 cm 넘게 어긋나는 측정은 버림 (앞 가로선이 사라진 뒤 0.54 m 뒤 가로선이 들어온다) | `m3_line_match_m` |
| 정지·대기 | 남은 거리 ≤ 0 → `stop` (바로 0). 속도 < 0.01 m/s 된 순간부터 4 s (규정 3 s 이상, 15 s 안 발진) | `m3_wait_s`, `m3_still_mps` |
| 끝남 | 발진 후 0.5 m → LANE, 이후 3 m 동안 보행자 표지판 무시. 표지판 뒤 2 m 안에 정지선이 안 보이면 포기 → LANE | `m3_exit_m`, `m3_cooldown_m`, `m3_search_m` |

`m3_stop_gap_m` 0.02 = 정지 명령 뒤 미끄러지는 거리 (시뮬 0.04 m/s 에서 1.6–1.9 cm) 만큼 앞에서 명령. 실차는 미끄러짐을 재서 바꾼다.
시뮬 트랙에는 지그재그 시작 앞(y 4.47)에도 흰 가로줄이 있지만, 보행자 표지판이 보이기 전이라 M3 는 횡단보도 앞 가로선(y 2.51)을 정지선으로 쓴다.

**M7 3차선 정적 장애물 회피** — v3: "내 차로는 RGB, 장애물은 뎁스, 차로 번호는 ±1 로 기억, 지나친 걸 확인한 뒤에만 다음 변경"

| | 방법 | 설정 |
|---|---|---|
| 장애물 | `depth_obstacle`: 뎁스 점 → 바닥에서 높이 4 cm 이상 → 바닥 격자로 묶음 → base_link 상자. mission_manager 가 오도메트리 좌표로 **기억** (뎁스 화각 ±35° 라 옆을 지날 때는 안 보인다). 같은 LIMO 를 여러 각도에서 본 관측은 0.3 m 안이면 합치고 상자는 관측별 중앙값. 옆에 있는 장애물 (가운데가 차 옆 0.1 m 밖) 은 차에서 먼 옆면을 가까운 옆면 + 0.22 m 로 채움 (11-02 오후 — 먼 옆면이 덜 보여 중심이 차 쪽으로 잡혔다) | (depth_obstacle) `min_height`, `max_range`; `m7_merge_m`, `m7_obstacle_len_m`, `m7_obstacle_width_m`, `m7_side_min_m` |
| 켜짐 | 내 차로 (옆 ±0.2 m) 앞 1.2 m 안에 폭 12 cm·높이 10 cm 넘는 장애물 3 장 연속. 코너에서 도는 중 (0.25 rad/s 넘음) 이거나 노란 선이 양쪽 다 보이는 1차로 도로면 안 켜짐 | `m7_trigger_m`, `m7_min_width_m`, `m7_min_height_m`, `m7_max_yaw_rate` |
| 차로 기준 | 방향 = 최근 (차 yaw + 차선 방향) 평균을 90° 단위로 맞춤 (이 트랙 직선은 출발 방향과 평행·직각). 차로 중심 = 기억한 LIMO 들의 옆 위치로 맞춘 0.52 m 격자 — 세 차로를 다 확인하면 고정. 차로 번호 0 = 맨 왼쪽 | `m7_axis_snap_deg`, `m7_lane_width_m` |
| 차로 중심 보정 (11-02 오후) | 위 격자는 그대로 두고 더함: 맨 왼쪽 (노란 선 왼쪽만) / 맨 오른쪽 (오른쪽만) 차로를 거의 곧게 (차·선 방향 15° 안) 갈 때 lane_follower 가 그 노란 바깥선으로 잰 차로 가운데 (`center_offset_m`, 방향 오차 × 0.6 m 보정) 와 격자 중심의 차이를 8 번 모아 중앙값의 절반만큼 격자를 옮김 (합 최대 15 cm). 가운데 차로는 노란 선이 없어 보정 안 하지만, 옮긴 격자는 세 차로 다 적용. 로그 `[M7] 노란 선으로 차로 0 가운데 보정 -1.5 cm (합 …)` | `m7_line_fix`, `m7_line_fix_heading_deg`, `m7_line_fix_ahead_m`, `m7_line_fix_frames`, `m7_line_fix_gain`, `m7_line_fix_gate_m`, `m7_line_fix_max_m` |
| 들어가자마자 | IMU 자이로 적분 방향 (들어갈 때 odom yaw 에 맞춤) 으로 제자리에서 차로 방향에 맞춤 (±3°, 최대 3 s) — 비스듬한 채로 차로를 판단하지 않게 | `m7_align_deg`, `m7_align_timeout_s`, `imu_topic` |
| 차로 고르기 | 내 (목표) 차로 앞 **1.2 m** 안이 막히면 갈 차로를 정한다: 막은 장애물을 지나가는 구간 [앞면 − 0.30 m, 뒷면 + 0.15 m] 이 비어 있는 옆 차로 중 그 뒤로 가장 오래 비는 곳 (M2 표지판 쪽 먼저, 같으면 `m7_prefer`). 실제로 꺾는 때는 아래 쓸기 검사가 정한다. 막히면 **변경 중이어도** 다시 고른다. 변경 완료 = 목표 차로 가운데에서 10 cm 안 | `m7_block_m`, `m7_enter_window_m`, `m7_lane_done_m`, `m7_prefer` |
| 운전 (쓸기 검사) | 기억한 장애물 상자 + **이번 라이다 점** (범퍼 0.06 m 밖부터 정확 — 뎁스는 가까이서 LIMO 앞면을 17 cm 멀게 잡은 적이 있다) 에 대해, 차 사각형 (+3 cm) 을 방향 후보 (바라는 방향 = 목표 차로 가운데를 0.3 m 앞에서 보는 pure pursuit, 그리고 0~60° 10° 간격) 로 곧게 밀어 보며 기억한 장애물 상자에 닿기 전까지 거리를 잰다. 바라는 방향에 가장 가까우면서 15 cm 이상 갈 수 있고 제자리에서 그쪽으로 돌 수 있는 방향을 고름. 없으면 **제자리 회전 (v = 0)** 으로 더 멀리 갈 수 있는 방향, 그것도 없으면 '갈 방향 없음'. 속도 = 갈 수 있는 거리 − 7 cm (지금 향한 방향의 빈 거리로도 제한), 0.15 m/s (대각 0.08). 고른 방향과 25° 넘게 다르면 먼저 제자리 회전 | `m7_margin_m`, `m7_go_free_m`, `m7_heading_step_deg`, `m7_max_heading_deg`, `m7_stop_gap_m`, `m7_turn_first_deg` |
| 막힘 → 후진 | 명령은 움직이라는데 (v > 0.03 또는 \|w\| > 0.3) **1 s** 동안 1 cm·2° 도 못 움직이면 (밀고 있음) 바로 전진을 멈추고 후진. '갈 방향 없음' 이 1 s 면 차로 다시 고르기, 2 s 면 후진. 후진은 오도메트리로 0.15 m, 뒤는 라이다 (5 cm) 와 기억한 장애물로 확인 | `m7_stuck_*`, `m7_blocked_s`, `m7_back_*` |
| 재시도 한도 | 같은 자리 (0.2 m 안) 에서 3 번 후진·재시도해도 막히면 반대쪽 차로로 한 번 더, 그래도 안 되면 멈추고 `ERROR [M7] … 멈춤` | `m7_retry_radius_m`, `m7_retry_max` |
| 장애물 거르기 | 폭 0.45 m 넘는 뎁스 장애물은 버림 (방지턱에서 차가 기울면 바닥이 폭 2 m 장애물로 보여 1차로에서 M7 이 켜진 적이 있다). 비스듬히·옆에서 본 관측은 앞에서 본 관측이 3 개 넘게 있으면 상자에 안 넣음 (가정 길이가 엉뚱한 쪽으로 붙어 차 뒤까지 늘어남). M5 서행 중·M2 뒤에는 M7 을 켜지 않음 | `m7_max_width_m`, `m7_oblique_deg` |
| 장애물 기억 | 시간으로 안 지운다 — 범퍼에 붙어 뎁스에 안 보여도 남아 있게. 오도메트리로 그 장애물 뒷면이 차 뒤끝보다 0.3 m 뒤로 지나간 뒤에 지움 (4 m 넘게 멀어지거나, 1 m 밖에서 30 s 안 보인 것도 지움) | `m7_pass_margin_m` |
| 끝남 | 변경이 끝났고 모든 차로 앞 1 m 가 비면 → LANE + 슬라이딩 윈도우 리셋. 분리대 앞 표지판도 장애물로 보여 표지판 쪽 (못 봤으면 `m7_prefer`) 차로로 빠진 뒤 끝난다 | `m7_clear_m` |

차로 중심 보정의 근거 (11-02 오후, 사용자: "왼쪽으로 차선을 물고 감"): 지난 시험 rosbag 4 판에서 뎁스로 본 옆 차로 LIMO 의 중심이 실제보다 차 쪽으로
앞에서 볼 때 1.5–2 cm, 비스듬히 볼 때 3–4.5 cm 당겨져 있었고 (먼 옆면 모서리가 5–7 cm 안쪽), 3차선에서 차가 차로 가운데보다 왼쪽으로 2–6 cm,
한 판은 점선 위를 곧게 달렸다 (차로 가운데에서 약 15 cm). 노란 바깥선으로 잰 차로 가운데는 방향 15° 안이면 실제와 0.3–0.8 cm 차이 (rosbag 5 판).
rosbag 재생 (시뮬 없이) 으로 예외 없이 돌고 보정 −3 ~ +4.4 cm 가 걸리는 것까지 확인 — 시뮬 주행 확인은 아직. 선을 밟는 시간의 대부분은
0.9 m 간격 줄 사이 대각 차로 변경 중이라 이 보정으로 없어지지 않는다.

2026-11-01 사용자 시뮬에서 갇힌 판 (rosbag 으로 확인): 차로 0 → 1 변경을 앞 LIMO 0.54 m 앞에서 시작해 모서리가 닿았고, 그 뒤 650 s 넘게
`/mission/cmd_vel` v +0.08 (변경 중 속도) 로 앞 LIMO 를 밀었다 — 뎁스는 붙은 LIMO 를 못 보고 (카메라에서 8 cm) 기억은 30 s 뒤 지워졌으며,
옛 진행 감시 (8 s) 는 목표 차로만 되돌려 같은 자리에서 반복. `/lane/cmd_vel` 은 M7 동안 한 번도 차로 안 갔다. → 위 표의 쓸기 검사·후진·라이다 안전 정지·기억 유지로 고침.

**M2 갈림길 A/B** — v3: "YOLO 로 A/B 를 인식해 결과를 저장해 두고, 분기점에서 A 면 왼쪽, B 면 오른쪽". 이 트랙은 3차선 끝 분리대에서
갈라지고 (A = 분리대 아래 왼쪽 길, B = 위 오른쪽 길) 그 직전까지 M7 이 직접 운전하므로, 분기는 M7 이 마지막에 들어갈 차로를 고르는 것으로 한다.

| | 방법 | 설정 |
|---|---|---|
| 표지판 인식 | YOLO `turn_a` = A, `turn_b` = B ([2026_Autorace](https://github.com/mark2455/2026_Autorace) 모델, 좌우 반전 증강을 끄고 학습해 A/B 를 안 헷갈림). **보조**: 이 모델은 시뮬 렌더 표지판을 1 m 앞 (지름 53 px) 에서도 못 잡고, 바로 옆 정차 LIMO 바퀴를 `turn_a` 로 잡았다 → `yolo_detector` 가 ① YOLO turn 박스 안에 파란 원이 없으면 버리고 ② YOLO 가 못 잡은 파란 원은 원 안 흰 무늬를 A·B 그림과 상관으로 판별해 같은 라벨로 넣는다 (시뮬: 4 m 앞 지름 14 px 부터 맞는 쪽 0.6–0.95, 반대쪽 ≤ 0.47) | (yolo_detector) `branch_assist`, `branch_min_score` 0.6, `branch_min_margin` 0.25, `branch_check_yolo`; 그림 `autorace_perception/models/sign_branch_A.png`·`_B.png` |
| 결과 기억 | 같은 결과 3 장 연속 (신뢰도 ≥ 0.5, 박스 ≥ 12 px) 이면 채택. 다른 결과가 다시 3 장 확인되면 바꿈 (마지막 것을 따름). 같은 결과를 박스 45 px 이상 (표지판 약 1.2 m 앞 = 분기점 가까이) 으로 확인하면 **잠금** — 더 안 바뀜. 3차선 안에서 잠금 전에 결과가 바뀌면 지금 위치 차로에서 다시 고름 | `m2_label_a`, `m2_label_b`, `m2_min_conf`, `m2_min_box_px`, `m2_confirm_frames`, `m2_lock_box_px` |
| 3차선에서 | 내 차로가 막혀 옆으로 갈 때 **표지판 쪽 옆 차로를 먼저** (갈 수 있으면). 표지판 반대쪽 끝 차로에 있으면 앞이 그만큼 빈 옆 차로로 미리 옮김 (앞에 장애물이 하나도 없으면 — 분리대 옆 — 안 옮김: 분리대를 가로지르게 된다) | — |
| 끝 | M7 이 끝난 차로 = 들어간 길. 결과를 확정하고 다음 리셋까지 표지판을 더 안 봄. 표지판을 못 봤으면 `m7_prefer` (left) 쪽 + 경고 | `m7_prefer` |

**M8 라바콘** — 규정: 라바콘 사이 안쪽 길로 통과, 넘어지거나 바닥 표시를 완전히 벗어나면 −1, 차선 이탈 판정 없음, 멈추면 재시작 (멈춤이 가장 나쁨).
라바콘은 2D 라이다 높이에 걸리는 높이라 라이다로 본다 (뎁스·YOLO 안 씀).

| | 방법 | 설정 |
|---|---|---|
| 라바콘 찾기 (`lidar_processor`) | `/scan` 앞 ±90°, 0.12–1.5 m → 이웃 점 (6 cm 안) 묶기 → 1 점 이상 · 크기 12 cm 이하 묶음만 (정차 LIMO 0.22 m·벽은 버림) → 점 평균을 몸통 반지름 (2.2 cm) 만큼 안쪽으로 = 라바콘 중심 | `autorace_perception/config/lidar_processor.yaml`: `fov_deg`, `range_min/max`, `cluster_gap_m`, `min_points`, `max_size_m`, `cone_radius_m` |
| 기억 | 스캔 찍은 순간의 오도메트리로 odom 좌표에 쌓음 (8 cm 안이면 같은 콘, 2 번 이상 본 것만 냄). **시간으로 안 지움** — 진행 방향으로 0.4 m 지나간 뒤 지움 (3 m 넘게 멀어진 것도) | `merge_m`, `min_hits`, `pass_margin_m`, `max_keep_m` |
| 켜짐 | M2 를 지난 뒤, 앞 1.5 m 안 양쪽 (왼쪽·오른쪽 다) 라바콘 4 개 이상이 3 번 연속 | `m8_require_m2`, `m8_trigger_m`, `m8_trigger_count`, `m8_confirm_frames` |
| 틈 (Follow-the-Gap) | 두 콘을 잇는 선이 진행 방향과 거의 수직 (±50°) 이고 실제 빈 거리 (중심 거리 − 라이다 높이 몸통 지름) ≥ 차 폭 0.22 + 0.10 = 0.32 m 인 쌍 = 통로를 가로지르는 틈. 틈 가운데점을 앞 순으로 이은 길에서 0.25 m 앞 점으로 향함 (pure pursuit), 방향 ±45° 안. 그런 틈이 없으면 (콘이 한 줄·흩어짐) 방향 순으로 이웃한 콘 사이 틈의 가운데로 | `m8_gap_min_m`, `m8_body_r_m`, `m8_aim_m`, `m8_max_heading_deg` |
| 쓸기 검사 (M7 과 같은 `avoid.py`) | 라바콘 = 바닥판 0.11 m 정사각형 상자 (바퀴가 바닥판에 안 닿게). 그 방향으로 차 사각형을 밀어 15 cm 이상 못 가면 ±10·20° 다른 방향, 다 막히면 조금이라도 앞으로 가는 쪽. 0.15 m 쓸어 볼 때 여유 5 cm 로 닿으면 0.10 m/s (아니면 0.15) | `m8_margin_m`, `m8_slow_clear_m`, `m8_slow_check_m`, `m8_speed`, `m8_slow_speed` |
| 막힘 | 움직이라는데 1 s 동안 1 cm·2° 도 못 움직이면 오도메트리로 0.10 m 후진 (뒤는 라이다·라바콘 상자로 확인) 후 다시 계산. 같은 자리 3 번 넘으면 후진 대신 조금이라도 앞으로 가는 쪽으로 계속 (서 있지 않게). 통로 안 제자리 회전은 ±15° | `m8_stuck_check_s`, `m8_back_m`, `m8_retry_max`, `m8_rot_limit_deg` |
| 끝 | 마지막 틈을 지나면 **통로 출구선** (마지막 쌍 가운데를 지나고 방향 = 처음 쌍 가운데 → 마지막 쌍 가운데) 위 0.25 m 앞 점을 따라감. 앞 1 m 안에 라바콘이 없고 옆·뒤 라바콘도 다 0.4 m 지나면 출구선에 맞춤 (방향 6°·옆 4 cm 안, 최대 3 s) → `LANE` + 슬라이딩 윈도우 리셋 + 1.5 s 동안 0.15 m/s. 이후 라바콘 무시, 보행자·서행 표지판·지그재그·M7 도 안 켬 | `m8_end_clear_m`, `m8_end_pass_m`, `m8_exit_deg`, `m8_exit_lat_m`, `m8_exit_s`, `m8_exit_slow_s`, `m8_exit_speed` |

출구선을 따라가는 이유: S 배치 끝 쌍이 비스듬해 마지막 틈 뒤로 곧게 가면 25° 틀어지고 차로 가운데에서 20 cm 벗어난 채 나오는데, 왼쪽 직선 끝이 바로
오른쪽 커브라 차선 추종이 못 잡고 트랙 밖으로 나가거나 (방향만 맞췄을 때도 24 판 중 3 판) 바깥 선을 따라 크게 돌았다.
그리고 슬라이딩 윈도우를 리셋하면 첫 프레임에 선이 하나만 보일 때 그 선을 늘 오른쪽 경계로 봤는데, 커브 입구 가운데서는 앞을 가로지르는 왼쪽 경계 하나만
보여 반대로 꺾었다 (라바콘 없이 같은 자리에서 차선 추종만 새로 켜도 같음) → 미션 매니저가 리셋할 때는 "차가 차로 가운데쯤" 을 알려 첫 프레임 단선 판정에 쓴다
(`sliding_window.reset(center_hint=0.5)` — M7 끝에도 같이 적용).

**M4 T자 주차·탈출** (`parking_controller`, 디퍼렌셜 — 돌 때 각도는 IMU 자이로 적분, 곧게 갈 때 거리는 휠 오도메트리) — 규정: 전면·후면 진입
모두 허용, 차체 네 모서리가 모두 라인 안 15 점 / 한 곳 걸침 7 / 두 곳 이상 0, 양옆 정차 LIMO 최소 2 대 (접촉 감점), 15 s 안 진입 시작. 우리는 전면 진입 →
빠져나와 처음 방향 → 종료. 시뮬 주차 구역: 위 줄 3 칸 + 아래 줄 3 칸, 칸 안쪽 0.50 (폭) × 0.60 m (깊이), 선 2 cm, 통로 (주차선 → 2차로 바깥 노란 선) 0.52 m,
빈 칸 = 위 줄 가운데 (양옆 LIMO). LIMO 가 칸 가운데면 옆 여유 14.1, 앞뒤 13.9 cm. 제자리 회전 반경 0.194 m < 통로 반폭 0.26 m.

| 단계 (`/parking/state`) | 하는 일 | 설정 (`autorace_mission/config/parking_controller.yaml`) |
|---|---|---|
| `WAIT` (통합) | `/parking/start` 뒤 차선 추종이 몰고 감. 앞옆 0.9 m 안에 주차 LIMO 가 보이면 `APPROACH` (단독 모드는 바로 `APPROACH`) | `wait_detect_m` |
| `APPROACH` | 0.10 m/s, IMU 방향 유지 + 주차 LIMO 줄과 옆 거리 유지. 차 중심 옆 라이다 띠가 **가까움 → 멂 → 가까움** (LIMO → 빈 칸 → LIMO) 이면 빈 폭 (LIMO 옆면 사이) 확인 → 칸 가운데 (진행 거리) 와 깊이 (LIMO 끝면 + LIMO 반 길이) 를 odom 에 저장 | `approach_speed`, `side_near_m`, `side_far_m`, `gap_min_m`, `gap_max_m` |
| `ALIGN` | 차 중심을 칸 가운데선에 (±0.8 cm, 앞뒤로), 제자리 회전 반경 + 3 cm 를 라이다 점 쓸기 검사 (M7 과 같은 `avoid.rot_ok`) | `align_tol_m`, `rot_margin_m` |
| `ROTATE_IN` | IMU 로 제자리 90° (±1.5°). 오차 비례로 줄이고 (최소 0.2 rad/s — 스키드는 더 작으면 안 돔), 구동 지연 0.45 s 동안 더 돌 것을 빼고 멈춤. 멈춘 뒤 흔들림이 가라앉으면 끝 | `rot_tol_deg`, `rot_min_w`, `rot_max_w`, `rot_lag_s` |
| `MEASURE` | 돌면 위치가 밀리므로 라이다로 다시 잼: 양옆 LIMO 안쪽 면 가운데 = 칸 가운데선. 옆 오차 5 cm 넘으면 돌아 나와 `ALIGN` 부터 | `measure_tol_m` |
| `ENTER` | 0.09 m/s 전진, 양옆 거리 같게 P + 칸 축 방향, 각속도 ±0.25 rad/s. 칸 깊이 가운데에서 정지: 칸 끝선 (lane_follower 정지선 거리 → odom, M3·M1 과 같은 방식 `stopline.StopLine`) − 칸 깊이 반, 못 보면 LIMO 로 잰 깊이. 남은 거리 비례 감속, 앞으로 쓸기 검사 | `enter_speed`, `enter_k_lat`, `enter_max_w`, `depth_tol_m` |
| `CHECK` / `FIX` | 네 모서리 → 칸 선 여유 (odom 칸 좌표) — 3 cm 보다 작으면 앞뒤 보정 (2 번까지) | `corner_min_m`, `max_fix` |
| `HOLD` → `EXIT` | 2.5 s 정지 → 들어간 거리만큼 후진 (라이다로 양옆 유지) | `hold_s`, `exit_speed` |
| `ROTATE_OUT` → `DONE` | IMU 로 반대 90° → `/parking/done`, `/parking/result` (네 모서리 여유·회전 오차·시간) | |
| 안전 | mux 라이다 범퍼 (주차 중에도: 앞 10 / 뒤 8 cm, 회전은 통과). 단계마다 1 s 못 움직이면 정렬부터 (칸 안이면 먼저 빠져나옴), 3 번 넘으면 `FAIL` 정지 | `stuck_s`, `max_retry` |

단독 실행 (주차 구역 앞에서 바로, `/parking/start` 없이):
```bash
ros2 launch autorace_mission parking.launch.py        # 시뮬: 차를 (5.30, 4.84, 0°) 로 옮긴 뒤 시작 (reset_sim --pose)
# RViz: /parking/markers — 빈 칸 (초록), 목표 화살표 (노랑), 차 사각형 (파랑), 네 모서리 여유 cm (선 밖이면 빨강)
# 시뮬 주차 LIMO 배치 바꾸기 (다음 리셋부터): ros2 topic pub -t 3 /sim/parking_layout std_msgs/msg/String "{data: 'seed=3'}"
#   default | seed=N (칸 안 옆 ±5, 깊이 ±6 cm, 방향 ±10°, 앞뒤 뒤집기) | "x,y,yaw;x,y,yaw"
```

시뮬 결과 (11-02): 단독 10 판 (LIMO 배치 default + seed 1–9) 모두 `DONE`, 네 모서리 모두 안 (가장 좁은 모서리 8.9 cm), LIMO 최소 간격 22.5 cm (접촉 0),
90° 회전 오차 (시뮬 실제 방향) 들어갈 때 −0.10 ~ +1.18°, 나올 때 −1.27 ~ −0.51°, 시작 → 끝 51–54 s (칸 찾기 19–21 s). 통합 (B 길 → M8 → M1 → M4) 2 판
(빨강 정지 +0.2 cm → 발진 / 초록 통과): 둘 다 `DONE`, 모서리 +10.8 ~ +14.3 cm, 회전 오차 +0.26/−0.68°, +0.28/+0.17°, `/parking/start` → 끝 73 s.

**M1 신호등** — 규정: 빨강·노랑·초록 인식, 옆 모니터에 광고·오인식 유도 그래픽, 정지선 ±5 cm (앞바퀴 기준) 완전 정지, 초록 인식 뒤 15 s 안 출발,
신호등은 카메라 화각 (약 71°) 안. YOLO 신호등 모델이 아직 없어 시뮬에서는 `fake_traffic_light` (시뮬이 아는 신호등 위치를 카메라에 투영) 로 FSM 부터 확인한다.

| | 방법 | 설정 (`autorace_perception/config/traffic_light.yaml`, 미션은 `mission_manager.yaml` m1_*) |
|---|---|---|
| 색 | 몸체 박스 (YOLO 1 클래스 `traffic_light`) 를 세로 3 칸 (위 빨강, 가운데 노랑, 아래 초록) 으로 나눠 칸 가운데 폭의 밝기 (RGB 최대) 상위 10 % 평균이 가장 큰 칸 — 색상이 아니라 **밝기**로 비교 | `light_label` |
| 모니터 거르기 | ① 세로:가로 1.4–2.1 (실물 20.5 × 12 cm ≈ 1.7) ② 한 칸만 켜짐: 가장 밝은 칸 ÷ 두 번째 ≥ 1.6 ③ **크기**: 크기로 잰 거리 (fy × 0.205 / 박스 높이) 가 정지선으로 잰 예상 거리 (카메라 → 정지선 + 정지선 → 신호등 1.65 m 앞·0.37 m 오른쪽) 와 25 % 안 — 모니터에 그린 16 cm 신호등은 0.78 배라 걸러짐 ④ 위치 고정: 크기까지 확인한 박스 위치 (odom) 에서 0.35 m 넘게 벗어난 박스는 버림. 여러 개 통과하면 예상 크기에 가장 가까운 것. **정지선을 보기 전에는 색을 안 냄** (크기로 못 거르므로) | `aspect_min/max`, `contrast_min`, `size_tol`, `light_beyond_m`, `light_lateral_m`, `track_radius_m`, `require_line` |
| 정지선 (신호등 노드) | `/lane/state` 정지선 거리를 odom 으로 이어 감, 처음 잡은 선만. 차 뒤로 1 m 지나거나 진행 방향이 45° 넘게 바뀌면 잊음 (라바콘 직선 끝 정지선을 2차로까지 들고 와 예상 거리가 틀렸었다) | `line_match_m`, `line_keep_m`, `line_turn_deg` |
| 램프 전환 | 두 칸이 같이 밝아 대비가 모자라면 (램프가 막 바뀜 — 시뮬 렌더러는 멈춘 카메라에서 꺼진 램프 빛을 몇~십몇 초 남긴다) 최근 3 s 동안 어두웠다가 2 배 넘게 밝아진 칸이 하나뿐일 때만 그 칸, 아니면 애매 → 정지 유지 | `transition_window_s`, `transition_ratio` |
| 정지선 정지 (M3 와 같은 코드) | `line_update`·`line_remaining` 공통: lane_follower 가 BEV 에서 찾은 정지선 (차로를 가로지르는 흰 줄, 기울기 ±15° 안 — 횡단보도 막대는 진행 방향으로 길어 덮는 비율에서 걸러짐) 을 영상 시각의 위치와 짝지어, 화면 아래로 사라진 뒤는 오도메트리로. 카메라→앞바퀴 보정, 남은 거리 비례 감속, lane_follower 가속 제한 | `m3_*` (공통) |
| 동작 | 빨강·노랑 → 정지선에서 서서 유지 (진행 감시 꺼짐). 크기 확인된 초록 8 번 연속 → 발진 (도착 때 이미 초록이면 안 섬). 안 보임·애매 → 정지 유지 + 2 s 마다 로그. 정지선 0.3 m 지나면 M1 끝 → `/parking/start` | `m1_confirm_frames`, `m1_green_frames`, `m1_exit_m`, `m1_log_s` |
| 통과 중 (11-02 오후) | 초록 보고 통과할 때도 정지선 지나 0.3 m 까지 **0.15 m/s 서행**. 앞바퀴가 선에 닿기 전에 크기 확인된 노랑·빨강이 3 번 (초록 없이) 이면 다시 정지선에 서서 초록까지 기다림 — 노랑도 정지. 램프 전환 중 애매해서 색이 안 나온 영상은 이 수를 지우지 않음. 선을 이미 지난 뒤의 노랑은 그대로 통과 (규정: 정지선 ±5 cm) | `m1_go_speed`, `m1_stop_frames`, `m1_go_abort_m` (0 = 선 전이면 언제나) |

11-02 오후 사용자 판: 초록 보고 정지선 47 cm 앞에서 통과하기로 한 뒤 선 약 15 cm 앞에서 노랑이 됐는데, 예전에는 '정지선까지 25 cm 넘게 남았을 때만
다시 섬' 이라 그대로 지나갔고, 선을 지난 뒤 빨강이 되어 주차 쪽으로 천천히 가는 모습이 '빨강인데 서행' 으로 보였다 → 위 '통과 중' 으로 고침.

정지선 검출을 기울기에 강하게 바꾼 이유: 2차로 정지선은 오른쪽 커브를 막 빠져나온 곳이라 차가 7–18° 틀어진 채 정지선이 BEV 에 들어오는데, 예전 검출 (가로 행마다
90 % 를 덮는 줄) 은 4 cm 선이 10° 만 기울어도 어느 행도 못 덮어 정지선을 놓치고 횡단보도 앞 줄 (x 3.20) 에 섰다 (−92 cm). 이제 기울기 13 가지 (2.5° 간격) 로 줄을 따라
덮은 비율을 잰다 (BEV 세로·가로 축척 차이 반영, 프레임당 약 3 ms).

시뮬 시험: `ros2 topic pub --once /sim/traffic_light std_msgs/msg/String "{data: green}"` (red | yellow | green 고정, cycle = 주기대로), `/sim/traffic_light_state` 로 지금 색.
모니터 방해 시험: 미션 USD 사본의 광고 화면을 그린 신호등·빨간 원·초록 원·그린 초록 신호등 4 장으로 바꾸고 `drive.launch.py decoy:=true` (가짜 검출이 모니터 그림에도 박스를 냄).
실차: YOLO 에 `traffic_light` 클래스가 생기면 `traffic_light.yaml` 의 `detections_topic` 을 `/perception/detections` 로, `fake_light:=false`. 정지선 → 신호등 거리 (`light_beyond_m`, `light_lateral_m`) 는 대회 트랙에서 다시 잴 것.

M2 는 지금 M7 안에서만 동작한다 — 3차선 장애물 없이 갈림길에 오면 (실제 대회 배치가 다르면) 차선 추종이 따라갈 선을 고르게 (`lane_cmd.follow`) 더 만들어야 한다.

mission_manager 를 안 띄우면 lane_follower 는 상한·정지 없이 예전과 똑같이 달린다 (`lane_cmd_timeout_s` 1 s).

### 8-5. 시뮬 확인 결과 (2026-11-01, Isaac headless, 시설물 켬)

`drive.launch.py` 로 2 번 더 (다시 달리기 포함): 매번 check → YOLO 준비 → 리셋 → 출발 순서, 방지턱 위 평균 0.151 m/s, 해제 뒤 0.30 m/s,
Traceback 없이 종료. 시뮬만 켜 둔 15 초 동안 차 위치 그대로 (`/cmd_vel` 발행자 0). 차선 추종이 이미 떠 있을 때 drive 를 하나 더 켜면
`이미 떠 있는 노드: lane_follower` 를 찍고 시작하지 않음.

노드를 따로 띄운 첫 시험 2 번:

| | 1 번째 | 2 번째 |
|---|---|---|
| 서행 켜짐 (차 중심 world x) | 1.51 m — `slow` 0.67, 박스 36 px | 1.59 m — `slow` 0.53, 박스 38 px |
| 방지턱 위 속도 (앞바퀴 올라감 → 뒷바퀴 내려옴) | 0.129–0.172, 평균 0.151 m/s | 0.131–0.175, 평균 0.152 m/s |
| 서행 풀림 | 4.36 m (방지턱 끝 3.825, 차 뒤끝까지 넘어간 뒤), 서행 구간 2.84 m | 4.36 m, 2.77 m |
| 풀린 뒤 속도 | 0.30 m/s | 0.30 m/s |
| 차로 중심에서 벗어난 정도 | ±5 cm 안 | — |

YOLO 는 RTX 4060 Ti 에서 추론 12–21 ms, 7 Hz 남짓 (영상 30 Hz 중 최신 것만). 서행 표지판은 약 1.2 m 앞부터, 방지턱 표지판은 그 뒤에 연달아 잡힌다.

**M5 → M6 → M3 이어서** (`drive.launch.py`, 정지 보정 `m3_stop_gap_m` 0.02 넣고 2 번):

| | 1 번째 | 2 번째 |
|---|---|---|
| M5 서행 → 해제 | x 0.7 → 3.6 m (odom) | 같음 |
| M6 켜짐 | 잔차 8.0 px, 코너 지나 지그재그 시작 | 같음 |
| 지그재그 구간 속도 | 평균 0.150 m/s | 0.150 m/s |
| M3 켜짐 | `crosswalk` 0.5–0.8, 박스 39–45 px (표지판 약 1.3 m 앞) | 같음 |
| **앞바퀴 축 ↔ 정지선 가까운 끝** (시뮬 실제 위치) | **−0.2 cm** | **−0.1 cm** |
| 정지 명령 때 속도 / 그 뒤 미끄러짐 | 0.04 m/s / 1.9 cm | 0.04 m/s / 1.8 cm |
| 완전 정지 → 발진 | 4.02 s | 4.02 s |

**M7 3차선** (`drive.launch.py` 로 출발부터, 최종 설정 4 번 — 정차 LIMO 7 대와의 접촉은 기록한 차 경로와 LIMO 상자로 계산):

| | 1 | 2 | 3 | 4 |
|---|---|---|---|---|
| 차로 변경 (0 = 맨 왼쪽) | 1→0→1→0→1→0 | 같음 | 같음 | 같음 |
| 접촉 | 없음 | 없음 | 없음 | 없음 |
| 가장 가까운 간격 | 12.1 cm | 11.6 cm | 12.9 cm | 2.8 cm (줄 3 가운데) |
| 3차선 끝 | 분리대 앞에서 왼쪽 차로 → LANE (x 3.87) | 같음 | 같음 | 같음 |

고치기 전에는: 코너 끝에서 켜져 차로 방향을 18° 틀리게 잡음 (→ 90° 맞춤), 출발선 리셋 전에 켜진 미션 매니저가 지난 자리 LIMO 로 M7 을 켬
(→ 리셋 뒤에 켜기), 조향 진동 ±30° (→ 게인 ↓ + 감쇠), 한 LIMO 가 둘로 기억돼 변경이 늦음 (→ 0.3 m 안 합치기), 옆에 둔 LIMO 때문에 서서
갇힘 (→ 속도 계산에서 빼기 + 진행 감시), 방지턱에서 차가 기울어 바닥을 장애물로 봄 (→ 높이 10 cm 이상). 1 cm 안팎 스침이 남아 있던
조건들을 하나씩 고쳐 위 4 번이 나왔다. 간격 2.8 cm 인 판이 있어 여유가 넉넉하지는 않다.

한 바퀴를 넘겨 계속 달리면 3차선 뒤 (M8 라바콘·M1·M4 구간) 에서 M6·M3 가 잘못 켜지는 일이 있다 — 그 구간 미션을 만들 때 막는다.

**M2 갈림길** (`drive.launch.py` 로 출발부터. 표지판을 고정한 시험은 `params_sim.yaml` 사본에서 `branch_sign_period_s: 0`, `branch_sign_start` 만 바꿔 `params_file:=` 로):

| 표지판 | A 고정 | A 고정 | B 고정 | B 고정 | 30 s 마다 바뀜 (기본 설정) | 30 s 마다 바뀜 |
|---|---|---|---|---|---|---|
| 처음 인식 | `turn_a` 0.74, 15 px (표지판 약 4 m 앞, 3차선 입구) | 0.71, 15 px | `turn_b` 0.67, 15 px | 0.68, 15 px | A → B → A (마지막 것을 따름) | A → B → A |
| 차로 변경 (0 = 맨 왼쪽) | 1→0→1→0→1→0 | 같음 | 1→0→1→2→1→2 | 같음 | 1→0→1→0→1→0 | 같음 |
| 들어간 길 | **A** | **A** | **B** | **B** | **A** (지날 때 표지판 A) | **A** (지날 때 A) |
| 정차 LIMO 최소 간격 | 9.2 cm | 10.2 cm | 9.0 cm | 4.8 cm | **0.1 cm** | 2.0 cm |

여섯 번 모두 표지판 쪽 길로 들어갔고, 분리대 뒤 합류·왼쪽 직선 진입도 정상이었다. 시뮬에서는 A/B 판별을 **전부 보조** (파란 원 + 그림 상관) 가 했다 (yolo_detector 로그 `검출: turn_b(보조) 134` 등, YOLO turn 0 — 바퀴 오인식은 파란 원 확인에서 버려짐). 실차 표지판은 YOLO 가 학습한 사진과 같은 종류라 YOLO 가 잡을 것으로 보지만 확인 필요.
마지막 두 판의 간격 0.1·2.0 cm 는 갈림길과 상관없는 3차선 앞쪽 변경 (차로 0 → 1, 줄 1 가운데 LIMO 뒤로 들어가며 줄 2 왼쪽 LIMO 옆)
에서 나왔다 — M7 의 여유가 좁은 곳 (위 2.8 cm 판과 같은 자리). 

고치기 전에는: YOLO 만으로는 표지판 B 를 한 번도 못 잡고 정차 LIMO 바퀴를 `turn_a` 로 잡음 (→ 보조 판별 + 파란 원 확인),
표지판 앞 마지막 변경에서 '앞이 더 빈 쪽' 인 반대 차로로 빠진 뒤 분리대 옆에서 표지판 쪽으로 옮기다 분리대를 가로지름
(→ 표지판 쪽 먼저 + 기다림, 앞에 장애물이 없으면 안 옮김).

보정 없이 (`m3_stop_gap_m` 0) 2 번: −2.1 cm, −1.8 cm. 처음 시험에서는 앞 가로선이 화면 아래로 사라진 뒤 뒤 가로선으로 바꿔 잡아 56 cm 지나쳐
섰다 → "처음 잡은 선만 따라가기" 로 고침.
정지선 거리 측정 자체는 도면 위치와 1–2 cm 차 (0.3 m/s 주행 중, 영상 지연 포함).

**M7 막힘 수정 뒤 10 판** (2026-11-02, 시험 시뮬 headless 도메인 77, 실시간 배속 0.95–0.96, `drive.launch.py` 로 출발부터, 판마다 rosbag. 정차 LIMO 배치
L0 = 사용자 시뮬에서 갇힌 기본 배치, L1–L5 = LIMO 위치를 바꾼 배치. 갈림길 표지판은 리셋마다 무작위 A/B):

| 판 | 배치 | 표지판 → 들어간 길 | 충돌 | LIMO 밀기 | 5 s 이상 정지 | 0.5 s 넘는 멈춤 | 최소 간격 | M7 동안 `/cmd_vel` 출처 |
|---|---|---|---|---|---|---|---|---|
| 1 | L0 | B → B | 0 | 0 s | 0 | 0 | 11.6 cm | mission 1443/1443 |
| 2 | L0 | B → B | 0 | 0 s | 0 | 0 | 14.3 cm | mission 1442/1442 |
| 3 | L0 | B → B | 0 | 0 s | 0 | 0 | 11.3 cm | mission 1427/1427 |
| 4 | L1 | A → A | 0 | 0 s | 0 | 1 번 (1.1 s, 첫 LIMO 앞 제자리 회전 명령 중) | 6.8 cm | mission 1508/1508 |
| 5 | L2 | B → B | 0 | 0 s | 0 | 0 | 11.0 cm | mission 1475/1475 |
| 6 | L3 | A → A | 0 | 0 s | 0 | 0 | 9.9 cm | mission 1425/1425 |
| 7 | L3 | B → B | 0 | 0 s | 0 | 0 | 7.9 cm | mission 1425/1425 |
| 8 | L4 | A → A | 0 | 0 s | 0 | 0 | 6.4 cm | mission 1456/1456 |
| 9 | L5 | B → B | 0 | 0 s | 0 | 1 번 (1.1 s, 같은 자리) | 11.2 cm | mission 1580/1580 |
| 10 | L5 | A → A | 0 | 0 s | 0 | 2 번 (1.1 s 같은 자리, 0.6 s) | 2.6 cm | mission 1527/1527 |
| GUI | L0 | B → B | 0 | 0 s | 0 | 0 | 14.0 cm | mission 1436/1436 |

마지막 줄은 사용자 GUI 시뮬 (도메인 30, 배속 0.96). 후진은 10 판 모두 0 번 (막히기 전에 쓸기 검사가 멈춤). 라이다 안전 정지 시험:
앞이 막히면 전진만 0 (회전·후진 통과), 앞뒤가 다 막히면 회전만 통과, 4 cm 물러나면 풀림.

**M8 라바콘** (2026-11-02, 시험 시뮬 headless, 배속 0.94–0.95, B 길 (3.40, 1.46, 180°) 출발 → 합류 → 오른쪽 커브 → 왼쪽 직선 라바콘 → 2차로.
`m8_require_m2: false` 만 바꾼 시험 설정. 라바콘 이동은 시뮬 `/sim/cone_status`, 여유는 기록한 차 경로 (0.322 × 0.218 m) 와 라바콘 바닥판 (0.11 m) 사이. 최종 코드로 14 판):

| 배치 | 판 | M8 시간 (시뮬) | 라바콘 3 cm 넘게 밀림 / 넘어짐 | 최소 여유 (바닥판) | 1 s 넘는 정지 | 끝 → 차선 | M8 동안 `/cmd_vel` 출처 |
|---|---|---|---|---|---|---|---|
| default (S, 흔들림 0.12) | 1 / 2 | 19.8 / 19.9 s | 0 / 0 | 2.9 / 2.9 cm | 0 | 2차로, 벗어남 없음 | mission 396 / 398 (+ 넘김 순간 lane 1) |
| straight | 1 / 2 | 19.2 / 19.4 s | 0 / 0 | 7.5 / 7.4 cm | 0 | 같음 | mission 383 / 388 |
| curve | 1 / 2 | 19.6 / 19.6 s | 0 / 0 | 7.3 / 7.3 cm | 0 | 같음 | mission 392 / 391 |
| s (흔들림 0.16) | 1 / 2 | 20.8 / 20.8 s | 0 / 0 | 0.7 / 0.6 cm | 0 | 같음 | mission 416 / 414 |
| narrow (0.43 → 0.31 m) | 1 / 2 | 20.0 / 19.6 s | 0 / 0 | 4.4 / 4.5 cm | 0 | 같음 | mission 400 / 392 |
| wide (쌍 간격 0.33) | 1 / 2 | 20.6 / 20.5 s | 0 / 0 | 2.3 / 2.3 cm | 0 | 같음 | mission 412 / 409 |
| jitter (±3 cm) | 1 / 2 | 20.5 / 20.2 s | 0 / 0 | 1.9 / 1.7 cm | 0 | 같음 | mission 410 / 404 |

14 판 모두 라바콘 이동 최대 0.0 cm, 후진 0 번, 출구선 맞춤 끝 옆 오차 0.3 cm 안. 가운데선을 그대로 따라갈 때의 이론 최대 여유는 default 4.7, s 3.2,
narrow 4.6, jitter 2.6 cm — s 0.6 cm 는 첫 쌍 → 둘째 쌍 (옆으로 14 cm 꺾임) 에서 늦게 따라가서. 마지막 라바콘 (y 3.85) → M1 정지선 (2차로 x 2.283)
주행 거리 2.1–2.4 m. M1 시험 8 판 (default 배치) 도 같음: 밀림 0, 여유 2.8–2.9 cm, 2차로 진입.
고치기 전에는: 보이는 통로 맨 끝 틈 (1 m 앞) 을 목표로 잡아 S 에서 직선 쓸기 검사가 막혀 늦게 따라감 (최소 0.5 cm → 관문 가운데선 추종),
S 끝에서 틀어진 채 커브로 들어가 트랙 밖 (→ 끝 정렬), 곧은 배치 끝에서 윈도우 리셋 첫 프레임이 왼쪽 경계를 오른쪽으로 봐 바깥 도로로 감 (→ 리셋 때 중심 힌트),
방향만 맞추면 S 끝에서 20 cm 치우쳐 커브에서 가끔 바깥 선 추종 (24 판 중 3 판 → 출구선 따라가기).

**M1 신호등** (2026-11-02, 시험 시뮬 headless, 배속 0.90–0.93, B 길 출발 → M8 라바콘 → M1. `m8_require_m2: false`. 신호등 색은 시험 도구가
`/sim/traffic_light` 로 고정: red = 도착 때 빨강 → 정지 8 s 뒤 초록, green = 처음부터 초록, flip = 빨강 → 정지 1.5 s 뒤 초록 (정지 중 초록),
decoy = 모니터 화면 4 장 (그린 빨강 신호등·빨간 원·초록 원·그린 초록 신호등, 3 s 마다) + 가짜 검출이 모니터에도 박스, 빨강 → 정지 6 s 뒤 초록.
정지 오차 = 시뮬 실제 앞바퀴 축 → 정지선 가까운 끝 (− = 선 넘음), 초록 → 발진 = 시뮬 신호등이 초록이 된 순간부터):

| 판 | 정지 오차 (규정 ±5 cm) | 초록 → 발진 명령 / 실제 움직임 | 모니터를 고른 프레임 | 초록 아닌데 발진 | `/parking/start` |
|---|---|---|---|---|---|
| 빨강 도착 1 | −1.0 cm | 0.38 / 0.84 s | 0 | 0 | 1 |
| 빨강 도착 2 | −0.2 cm | 0.60 / 1.29 s | 0 | 0 | 1 |
| 초록 도착 1 | 안 섬 (정지선 앞 68 cm 에서 통과 결정, 최저 0.15 m/s) | — | 0 | 0 | 1 |
| 초록 도착 2 | 안 섬 (6 cm 앞에서 결정, 최저 0.04 m/s) | — | 0 | 0 | 1 |
| 정지 중 초록 1 | −1.5 cm | 0.31 / 0.87 s | 0 | 0 | 1 |
| 정지 중 초록 2 | −1.0 cm | 0.80 / 1.33 s | 0 | 0 | 1 |
| 모니터 방해 1 | −0.7 cm | 0.65 / 1.16 s | 0 (모니터 박스 788 개 들어옴) | 0 | 1 |
| 모니터 방해 2 | −0.9 cm | 0.42 / 0.82 s | 0 (792 개) | 0 | 1 |

정지선에서 신호등이 화면 안에 있는지: `~/Desktop/M1_정지선_화면/` (카메라 원본, 판단 화면 — 진짜 신호등 초록 박스 "red … ok", 모니터 박스는 빨강 "크기 거리 2.14 ≠ 예상 1.71").
초록 도착 2 는 초록 확인 (크기 확인된 초록 8 번) 이 정지선 6 cm 앞에서야 끝나 0.04 m/s 까지 줄었다가 통과했다 — 신호등은 정지선을 본 뒤에만 크기 확인이 되므로
(약 0.7 m 앞) 도착 직전에 초록이면 이렇게 거의 섰다 간다.
고치기 전에는: 커브 출구에서 틀어진 채 본 정지선을 놓쳐 횡단보도 앞 줄에 섬 (−92 cm → 기울기 검출), 신호등 노드가 라바콘 직선 끝 정지선을 들고 와
예상 거리 모름 → 초록을 못 믿고 계속 정지 (→ 방향이 바뀌면 잊음), 정지선 전 먼 거리에서 진짜 신호등이 대비로 버려지고 모니터 그림을 고름 (4 프레임 → 정지선 전에는 색 안 냄),
시뮬 렌더러 잔상으로 초록 뒤 16 s 동안 빨강 칸도 밝아 발진 지연 (8 판 중 2 판 14.9·16.6 s → 램프 전환 감지).

### 8-6. 파일

```
~/autorace_ws/src/
  autorace_msgs/msg/          Detection, DetectionArray, LaneCmd (max_speed, stop, reset_window, …), LaneState, Obstacle, ObstacleArray
  autorace_perception/        yolo_detector (config/yolo_detector.yaml, launch/, models/autorace_yolov8n.pt + 출처 README),
                              depth_obstacle (config/depth_obstacle.yaml), lidar_processor (config/lidar_processor.yaml, M8 라바콘),
                              traffic_light (config/traffic_light.yaml, M1 색·모니터 거르기)
                              branch_sign.py = 갈림길 표지판 보조 판별 (models/sign_branch_A.png, sign_branch_B.png)
  autorace_mission/           parking_controller (M4, config/parking_controller.yaml, launch/parking.launch.py 단독 실행), stopline.py (정지선·끝선 추적),
                              mission_manager, avoid.py (M7·M8 공통: 쓸기 검사, 라이다 점 상자, 막힘 감지, 재시도, 라바콘 틈),
                              check_nodes, cmd_vel_mux (config/mission_manager.yaml, config/cmd_vel_mux.yaml,
                              launch/drive.launch.py = YOLO + 미션 매니저 + 차선 추종 한 번에, launch/mission_manager.launch.py)
  autorace_sim/isaac/         run_sim.py, cone_layouts.py (M8 라바콘 배치 7 종, /sim/cone_layout · /sim/cone_status), missions.py (신호등 고정 /sim/traffic_light)
  autorace_sim/               fake_traffic_light (시뮬 신호등 박스, decoy)
  autorace_msgs/msg/          TrafficLight (M1)
  autorace_lane/              lane_follower: /mission/lane_cmd 속도 상한·정지, /lane/state 발행
                              sliding_window: 정지선 검출 (_detect_stop_line), 두 선 폭·피팅 잔차 결과 추가
```

---

## 9. 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| Examples 에 AutoRace 2026 이 안 보임 | Extensions 에서 `~/LimoIsaacSIM` 폴더 추가 + autorace_2026 Enable 확인 |
| Load 했는데 LIMO 가 없음 | `~/LimoIsaacSIM/USD/WegoLimo/Limo/limo_diff_thin.usd` 없음 → WegoLimoUSD.zip 풀기 |
| 트랙이 없음 (로그 "트랙 USD 없음") | `data/track/autorace_2026_track.usda` 확인, 또는 `track_usd` 설정 경로 |
| `Package 'autorace_sim' not found` | 그 터미널에서 `source ~/autorace_ws/install/setup.bash` 를 안 함 (항상 쓰려면 `~/.bashrc` 에 추가) |
| `Package 'teleop_twist_keyboard' not found` | `sudo apt install ros-humble-teleop-twist-keyboard` |
| `ros2 topic list` 에 아무것도 없음 | Play 를 눌렀는지, 두 터미널의 `ROS_DOMAIN_ID` 가 같은지 (30) |
| 로봇이 cmd_vel 에 반응 없음 | 토픽 이름 `/cmd_vel` (원본 `/sim/cmd_vel` 아님), 시뮬 Play 상태인지 |
| ROS 를 source 한 셸에서 Isaac python 스크립트의 `import rclpy` 실패 | python3.10 rclpy 가 Isaac 3.11 을 가림 → `~/autorace_ws/src/autorace_sim/isaac/run_isaac.sh 스크립트.py` 로 실행 (확장 자체는 해당 없음) |
| 시설물(장애물)이 안 보임 | 2026-10-26 이전에 띄운 시뮬이면 다시 띄운다. B 는 로그에 `missions on` 과 `[autorace_2026] 신호등 → red` 가 찍혀야 함. `missions.usd` 경로에 파일이 있는지 |
| `Package 'autorace_perception' / 'autorace_mission' not found` 또는 `No module named 'autorace_msgs'` | 새 패키지 — §8-1 빌드 후 그 터미널에서 `source ~/autorace_ws/install/setup.bash` |
| `No module named 'ultralytics'` | `pip3 install ultralytics` (§8-1) |
| `drive.launch.py` 가 `이미 떠 있는 노드: …` 를 찍고 끝남 | 다른 터미널·창에 그 노드가 남아 있음 → 그 창에서 Ctrl+C. 모르겠으면 `ps aux \| grep -E 'lane_follower\|yolo_detector\|mission_manager'` 로 PID 를 찾아 `kill -INT <PID>` |
| 시뮬만 켰는데 차가 움직임 | 어딘가 차선 추종이 떠 있음 (위와 같음). `ros2 topic info /cmd_vel` 의 Publisher count 가 0 이어야 한다 |
| 지그재그에서 `M6_SLOW` 가 안 켜짐 / 코너에서 켜짐 | `ros2 topic echo /lane/state --field line_resid_px` 로 그 구간 값을 보고 `m6_resid_on` 조정 |
| 보행자 표지판에서 안 멈춤 | 로그 `[M3] 정지선 보임` 이 없으면 정지선 검출 문제 → `/lane/debug` 에 하늘색 가로선이 뜨는지, 흰색 문턱 (`white_hsv_*`, `stop_line_min_cov`). `표지판 뒤 2.0 m 안에 정지선을 못 찾음` 이면 `m3_search_m` |
| 3차선에서 LIMO 에 닿음 / 변경이 늦음 | 로그 `[M7] 기억한 장애물 (…)` 의 상자가 실제 LIMO (0.32 × 0.22 m) 보다 크거나 하나가 둘로 나오는지. `m7_enter_overlap_m` ↑ 면 더 일찍 바꿈 (들어갈 차로 LIMO 와 가까워짐) |
| 3차선에서 서서 안 움직임 | 로그 `[M7] 갈 방향 없음 (쓸기 검사): …` 의 방향별 빈 거리 → 다 짧으면 기억한 장애물 상자가 실제보다 큼 (`/perception/obstacle_markers` 로 확인). `[M7] … 반대쪽 차로도 실패 — 멈춤` (ERROR) 이면 재시도 한도까지 간 것. 어느 입력이 차를 움직였는지는 `ros2 topic echo /cmd_vel_mux/selected` 또는 `[mux] 고름 …` 로그 |
| 앞 LIMO 를 밀고 있음 | 이제 생기면 안 됨: 1 s 안 움직이면 후진 (`[M7] 막힘: 명령 v … 인데 1.0 s 동안 …`), mux 라이다 안전 정지 (`[mux] 라이다 안전 정지 (앞)`). 둘 다 안 나오면 `/scan` 이 들어오는지 (`[mux] 라이다 … 안 들어옴` 경고) |
| 갈림길 표지판 결과가 안 나옴 (`[M2] 갈림길 표지판을 못 봄`) | 터미널 2 의 `yolo_detector` 로그에 `검출: turn_a(보조) …` 가 나오는지. `/perception/debug` 에서 표지판에 파란 박스가 뜨는지 → 안 뜨면 표지판 파란색이 `branch_sign.py` 의 HSV (H 100–130) 밖인지, 실제 대회 표지판 그림이 `models/sign_branch_*.png` 와 다른지 |
| 표지판과 다른 길로 들어감 (`[M2] 표지판 B 인데 …`) | 3차선 끝에서 반대쪽 끝 차로에 갇힌 것 — 로그의 마지막 `[M7] 차로 …` 줄들과 `/perception/obstacle_markers` 로 그 순간 옆 차로가 왜 막혔는지 |
| 신호등에서 안 섬 / 횡단보도에서 섬 | 로그 `[M1] 정지선 보임: 카메라에서 … m` 의 거리가 정지선 (2차로 x 2.283) 인지. 안 나오면 `/lane/debug` 에서 정지선에 하늘색 줄이 뜨는지 (`stop_line_min_cov`, `stop_line_max_deg`) |
| 초록인데 안 떠남 | 로그 `[M1] 정지 유지 — … 모두 버림: …` 의 이유. `크기 거리 a ≠ 예상 b` 면 정지선을 잘못 잡았거나 `light_beyond_m` 이 트랙과 다름, `켜진 칸 대비` 면 꺼진 램프가 밝게 보임 (`/perception/traffic_light_debug`), `정지선 아직 안 봄` 이면 신호등 노드가 정지선을 못 받음 |
| 주차: 빈 칸을 못 찾음 (`APPROACH` 가 안 끝남) | 로그 `[M4] 옆: 가까움 → 멂 …`, `틈 … 칸 폭이 아님` — 옆 라이다 거리가 `side_near_m`/`side_far_m` 사이에서 애매한지, 빈 폭이 `gap_min_m`~`gap_max_m` 안인지 |
| 주차: 90° 회전이 안 끝남 / 오락가락 | `rot_lag_s` (명령 → 실제 회전 지연), `rot_min_w` (너무 작으면 안 돎, 너무 크면 지나침). 실차에서 다시 잴 것 |
| 라바콘 구간에서 `M8_AVOID` 가 안 켜짐 | M2 를 지났는지 (`m8_require_m2`). `ros2 topic echo /perception/cones` 에 콘이 나오는지, RViz `/perception/cone_markers`. 안 나오면 `/scan` 높이에 라바콘이 걸리는지 (시뮬 라이다 0.111 m), `max_size_m` |
| 라바콘을 건드림 | 로그 `[M8] …` 의 틈 폭·방향. `/sim/cone_status` 의 max_cm. 실차 라바콘 몸통 지름 (라이다 높이) 으로 `cone_radius_m`·`m8_body_r_m`, 바닥판 크기로 `m8_cone_half_m` |
| 라바콘 뒤 차선을 못 잡음 | 로그 `[M8] 마지막 라바콘 지남 → 통로 방향 …` 과 `통로 방향에 맞춤 (오차 …)`. `/lane/debug` 에서 리셋 직후 선을 어느 쪽 경계로 잡는지 |
| 3차선이 아닌 곳에서 `M7_AVOID` 가 켜짐 | 그 자리 `/perception/obstacles` 의 높이·폭 → `m7_min_height_m`, `m7_min_width_m` |
| 정지 위치가 늘 앞/뒤로 몇 cm 어긋남 | `m3_stop_gap_m` (정지 명령 뒤 미끄러짐), `m3_cam_to_front_axle_m` (카메라 → 앞바퀴 실측) |
| 서행 표지판을 지나도 `M5_SLOW` 가 안 켜짐 | 터미널 2 로그에 `검출: slow …` 가 나오는지 → 안 나오면 `/perception/debug` 로 표지판이 보이는지. 나오는데 안 켜지면 박스 높이(`h…`)가 `m5_min_box_px` 보다 작은지 |
| `Package 'autorace_lane' not found` | 새 패키지라 `colcon build --symlink-install` 한 번 더 + `source install/setup.bash` |
| 차선 추종 노드가 차를 안 움직임 | `enable:=false` 로 켰는지, `/camera/color/image_raw` 가 나오는지 (`ros2 topic hz`), 시뮬이면 `use_sim_time` true, 실차면 false |
| 차선 % 가 0 | `/lane/debug` 오른쪽 마스크가 비어 있으면 색 문턱 (`yellow_hsv_*`), 선이 BEV 밖이면 카메라 높이·기울기 |
| 한 번 달린 뒤 차선 추종을 다시 켜도 차가 안 움직임 (로그 `차선 0%`) | `reset_sim:=false` 로 켰거나 리셋이 실패한 것 (아래 두 줄). 차가 시설물에 부딪히거나 트랙 밖이라 선이 안 보임 → `ros2 run autorace_sim reset_sim` |
| launch 를 다시 쳤는데 `[reset_sim] … 리셋 서비스가 없는 옛 버전` 후 끝남 | 2026-10-26 밤 이전에 띄운 시뮬 (리셋 기능이 없음). 터미널 1 Ctrl+C → `bringup … mode:=sim` 다시 |
| `[reset_sim] /sim/reset 응답 없음 (5 s) — 다시 부름 (1/3)` | 시뮬을 막 띄운 직후에는 시뮬이 리셋을 하고도 응답을 못 보낼 때가 있다 (시뮬 터미널에 `failed to send response (timeout)`). 리셋은 여러 번 해도 같아서 3 번까지 다시 부른다 — 이어서 `완료` 가 나오면 정상 |
| `[reset_sim] … 시뮬(/clock)이 안 보입니다` 후 끝남 | 시뮬이 안 떠 있거나 `ROS_DOMAIN_ID` 가 다름. A(확장) 시뮬에는 이 서비스가 없다 — 확장 창의 Reset 버튼 사용 |
| 다시 달렸는데 차가 두 차로 사이로 달림 (로그 `W` 가 100~130 px) | `lane_follower.yaml` 의 `lane_width_tol: 0.25` 확인 |
| 차선 추종을 켜지도 않았는데 차가 움직임 | 다른 터미널(창)에 차선 추종이 아직 켜져 있음 → `ros2 node list` 에 `/lane_follower` 가 있으면 그 창에서 Ctrl+C. 시뮬이 둘 떠 있지 않은지도 (`ps aux \| grep run_sim`) |
| 두 시뮬을 같이 켰더니 로봇이 이상하게 움직임 | 같은 `ROS_DOMAIN_ID` 면 `/cmd_vel` 을 둘 다 받는다 → 하나만 켜거나 `ROS_DOMAIN_ID` 를 다르게 |

---

## 10. git

```bash
cd ~/LimoIsaacSIM
git log --oneline -1          # f6706a5 Add autorace_2026 extension ...
git status                    # USD/WegoLimo/ 는 일부러 추적 안 함
```

전역 git 설정이 예시 값(`user.name=내이름`, `user.email=깃허브이메일`)이라 이 커밋은 `kb4 <dkkim528@kookmin.ac.kr>` 로 남겼다.
본인 이름으로 바꾸려면:

```bash
git config --global user.name "본인 이름"
git config --global user.email "본인 이메일"
cd ~/LimoIsaacSIM && git commit --amend --reset-author --no-edit
```

원격(GitHub)에는 아직 push 하지 않았다.
2026-10-26 작업 (미션 시설물, `autorace_lane`, `run_sim.py` 시설물 로딩) 은 아직 커밋하지 않았다 — `~/LimoIsaacSIM` 은 `git status`,
`~/autorace_ws` 는 커밋이 하나도 없는 상태.
