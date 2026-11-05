# autorace_ws — 2026 AutoRace 트랙 + LIMO (Isaac Sim / 실차 공용)

2026 국민대 AutoRace 트랙(10 m × 6 m) 도면을 Isaac Sim 맵으로 만들고, AgileX LIMO를 올려
실차와 **같은 토픽·프레임**으로 ROS2에 내보낸다. `mode:=sim|real` 하나로 시뮬레이터와 실차를 바꾼다.

| 항목 | 값 |
|---|---|
| Isaac Sim | 5.1.0 (`~/isaacsim`) |
| ROS2 | Humble, Fast DDS, `ROS_DOMAIN_ID=30` (`~/.bashrc`) |
| OS | Ubuntu 22.04 |

## 구조

```
src/
  autorace_track/      도면 → 분류 → USD (트랙 자산)
    data/source/         2026Track.dxf (원본), .pdf, 참고 png
    data/generated/      track_elements.json (분류 결과, mm), 확인용 PNG
    config/track_build.yaml   색·페인트 높이·바닥 물리 기본값
    tools/               dxf_classify.py, build_track_usd.py, check_track_isaac.py
    usd/autorace_track.usda   생성된 트랙 (m, Z-up, 원점 = 매트 좌하단)
  autorace_sim/        Isaac Sim 쪽
    isaac/run_sim.py       트랙 + LIMO + 센서 + ROS2 브리지 (Isaac python 으로 실행)
    isaac/limo_drive.py    /cmd_vel → 바퀴 속도 + 타이어 힘 모델
    isaac/missions.py      미션 시설물 신호등·갈림길 표지판·광고 화면 주기 전환 (LimoIsaacSIM/autorace_2026 과 같은 파일)
    isaac/run_isaac.sh     Isaac python 래퍼 (ROS 경로 정리, Ctrl+C 전달)
    autorace_sim/depth_converter.py   뎁스 32FC1(m) → 실차와 같은 16UC1(mm)
    autorace_sim/smoke_test.py        토픽·TF·구동 점검
    autorace_sim/reset_sim.py         /sim/reset 호출 — Isaac 을 끄지 않고 처음 상태로
  autorace_bringup/    실행 진입점
    launch/bringup.launch.py   mode:=sim|real
    config/params_sim.yaml, params_real.yaml
  autorace_lane/       슬라이딩 윈도우 차선 추종 (SEAME 2026 Hackathon D-Racer-Kit 에서 옮김, Apache-2.0)
    autorace_lane/sliding_window.py   검출기 (ROI → BEV → HSV → SW 코리도 → offset)
    autorace_lane/lane_follower.py    노드: /camera/color/image_raw → /cmd_vel (+ /lane/debug)
    scripts/lane_replay               저장 영상으로 검출기만 돌리기 (실차 튜닝용)
    config/lane_follower.yaml, launch/lane_follow.launch.py
```

미션 시설물 USD 는 `~/LimoIsaacSIM/autorace_2026/data/mission/` 의 것을 쓴다 (`params_sim.yaml` `missions.usd`, 만드는 법은 그쪽 docs/README.md).

## 빌드

```bash
cd ~/autorace_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 이면 launch·config·isaac 스크립트·USD 가 소스로 링크되므로 수정 후 다시 빌드할 필요가 없다
(새 파일을 추가했을 때만 다시 빌드).

## 실행

```bash
# 시뮬레이터 (Isaac 창 포함). 첫 실행은 셰이더 컴파일·LIMO 에셋 다운로드로 1~2분 걸린다
ros2 launch autorace_bringup bringup.launch.py mode:=sim
# 창 없이
ros2 launch autorace_bringup bringup.launch.py mode:=sim headless:=true
# 실차 (LIMO 위에서, limo_base·ydlidar_ros2_driver·astra_camera 설치 필요 — 아직 실차 미검증)
ros2 launch autorace_bringup bringup.launch.py mode:=real
# 다른 파라미터 파일
ros2 launch autorace_bringup bringup.launch.py mode:=sim params_file:=/path/to/my_params.yaml
```

점검 (다른 터미널):

```bash
ros2 run autorace_sim smoke_test            # 토픽 주기, TF 확인
ros2 run autorace_sim smoke_test --drive    # 로봇을 직진 0.3 m/s·제자리 회전 1 rad/s 로 움직여 /odom 확인
ros2 run autorace_sim smoke_test --out /tmp/snap   # color.png, depth.png, scan.png 저장
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 키보드 조종 (sudo apt install ros-humble-teleop-twist-keyboard)
```

Ctrl+C 로 launch 를 끄면 Isaac 까지 같이 종료된다.

다시 달리기 (Isaac 을 끄지 않고): `ros2 run autorace_sim reset_sim` → LIMO 출발 자세·정지, 움직인 라바콘 원위치,
신호등·갈림길·광고판 주기 0 초부터, `/odom` 0. 시뮬 시간은 계속 흐른다. Isaac 창의 ■ → ▶ 도 같은 처음 상태 (시뮬 시간도 0).

### 슬라이딩 윈도우 차선 추종 (`autorace_lane`)

```bash
# 시뮬: bringup 을 띄운 뒤 다른 터미널에서
ros2 launch autorace_lane lane_follow.launch.py                       # 먼저 /sim/reset 으로 출발선에 돌려놓고 시작
ros2 launch autorace_lane lane_follow.launch.py reset_sim:=false      # 지금 자리에서 이어서
ros2 run rqt_image_view rqt_image_view /lane/debug                    # 왼쪽 BEV + 상자·피팅·중심, 오른쪽 마스크
# 실차 LIMO Pro: bringup mode:=real (또는 LIMO 기본 limo_bringup + 카메라) 뒤에
ros2 launch autorace_lane lane_follow.launch.py use_sim_time:=false enable:=false   # 먼저 인식만 (바퀴 안 돔)
ros2 launch autorace_lane lane_follow.launch.py use_sim_time:=false                # 주행
# 저장 영상으로 검출기만 (HSV·BEV 맞출 때)
ros2 run autorace_lane lane_replay <영상 폴더|동영상> --out /tmp/lane_dbg
```

차선 추종만 한다 (미션·장애물 회피 없음). 시뮬(시설물 끔)에서 출발선 → 2차로 끝(주차장 앞)까지 한 바퀴를
바퀴가 선에 닿지 않고 (선까지 최소 0.11 m) 돈다. 2차로 끝은 막다른 길이라 그 뒤는 주차 미션 몫.

## 토픽 / 프레임 (sim = real)

실차 기준: `agilexrobotics/limo_ros2` humble 브랜치 (limo_base, URDF), YDLidar ROS2 드라이버, Orbbec Dabai.

| 토픽 | 타입 | frame_id | sim 주기 | 비고 |
|---|---|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | – | 구독 | 0.5 s 동안 없으면 정지 |
| `/odom` | nav_msgs/Odometry | odom → base_link | 60 Hz | sim 은 물리엔진 참값 (출발 자세 = 원점) |
| `/tf` | odom → base_link | | 60 Hz | 실차 limo_base `pub_odom_tf` |
| `/tf_static` | base_link → laser_link, imu_link, camera_link, camera_link → camera_{color,depth}_optical_frame | | | launch 의 static_transform_publisher |
| `/imu` | sensor_msgs/Imu | imu_link | 120 Hz | 중력 포함 가속도 |
| `/scan` | sensor_msgs/LaserScan | laser_link | 7.5 Hz | 375 점/회전, 0.12–8 m |
| `/camera/color/image_raw` | sensor_msgs/Image (rgb8) | camera_color_optical_frame | 30 Hz | 640×480, HFOV 71° |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | camera_color_optical_frame | 30 Hz | |
| `/camera/depth/image_raw` | sensor_msgs/Image (16UC1, mm) | camera_depth_optical_frame | 30 Hz | 0.3–3 m 밖은 0 |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | camera_depth_optical_frame | 30 Hz | |
| `/camera/depth/points` | sensor_msgs/PointCloud2 | camera_depth_optical_frame | 30 Hz | |
| `/clock` | rosgraph_msgs/Clock | – | 30 Hz | sim 전용. sim 노드는 `use_sim_time: true` |

`/limo_status` (limo_msgs) 는 실차 전용이라 sim 에서는 나오지 않는다.

## 파라미터

### `params_sim.yaml` → `isaac_sim`

| 키 | 기본값 | 설명 |
|---|---|---|
| `isaacsim_dir` | `~/isaacsim` | Isaac Sim 설치 경로 |
| `headless` | false | launch 인자 `headless:=true` 로도 켬 |
| `realtime` | true | 시뮬 시간을 벽시계에 맞춤 |
| `physics_hz` / `render_hz` | 120 / 30 | 센서 주기는 이 값의 약수로 반올림됨 |
| `track.floor_static_friction` | 0.8 | **바닥 마찰** (실행 시 적용, 재빌드 불필요). 타이어 모델의 μ 상한 |
| `track.floor_dynamic_friction` | 0.7 | 바닥 위 다른 물체(미션 오브젝트 등)용 |
| `reset_service` | `/sim/reset` | std_srvs/Trigger. 리셋 = 출발 자세 + 라바콘 원위치 + 미션 주기 0 초 |
| `robot.start_x / start_y / start_yaw_deg` | 0.74 / 5.58 / 0 | 출발 자세. 출발선(x 0.95–1.07 m) 바로 뒤, 1차로 중앙, +x 방향 |
| `missions.enabled` | true | 미션 시설물 (규정집 M1–M8: 신호등·표지판·정차 LIMO·라바콘·방지턱) |
| `missions.usd` | `~/LimoIsaacSIM/autorace_2026/data/mission/autorace_2026_missions.usda` | 트랙과 같은 좌표 |
| `missions.traffic_light_s` | red 10 / green 10 / yellow 3 | 신호등 색별 시간 [s, 시뮬레이션 시간]. 적 → 녹 → 황 |
| `missions.branch_sign_period_s` / `_start` / `_random` | 30 / A / false | 갈림길 표지판 A(좌회전) ↔ B(우회전) 주기, 0 = 고정 |
| `missions.ad_slide_period_s` | 3 | 방해 광고판 화면 넘김 |
| `robot.max_linear_vel / max_angular_vel` | 1.0 / 2.5 | 명령 상한 |
| `robot.cmd_vel_timeout` | 0.5 s | |
| `robot.drive.model` | `tire` | `tire`: 미끄럼 비례 타이어 힘 / `physx`: PhysX 마찰 그대로 (아래 '구동 모델') |
| `robot.drive.tire.c_long / c_lat` | 50 / 30 | 바퀴당 종·횡 미끄럼 강성 [N/(m/s)] |
| `robot.drive.tracking.enabled` | true | 몸체 속도 되먹임으로 명령 트위스트 추종 |
| `lidar.*` | X2L | `min_range` 0.12, `max_range` 8, `resolution_deg` 0.96, `rate_hz` 8 |
| `camera.*` | Dabai | `width` 640, `height` 480, `hfov_deg` 71, `rate_hz` 30 |
| `imu.rate_hz` | 100 | |
| `topics.*`, `frames.*` | 실차 이름 | |

### `sensor_mounts` (sim·real 공통, base_link 기준, base_link = 바닥 위 0.145 m)

| 프레임 | xyz [m] | 출처 |
|---|---|---|
| `laser_link` | (0.103, 0, −0.034) | limo_description URDF |
| `camera_link` | (0.084, 0, 0.03) | limo_description (ackermann / limo_car) URDF |
| `imu_link` | (0, 0, 0) | limo_base 가 내는 정적 TF |

### `params_real.yaml`

`real_drivers` (어떤 드라이버를 띄울지), `limo_base_node`, `ydlidar_ros2_driver_node` 파라미터, `sensor_mounts`.
설치 안 된 드라이버 패키지는 경고만 내고 건너뛴다.

### `autorace_lane/config/lane_follower.yaml` (차선 추종)

검출 파라미터 이름·기본값은 원본 `lane_node.py` 와 같다 (`sw_*`, `guide_*`, `near_weight` …). 이 트랙·LIMO 에 맞춰 바꾼 것만:

| 키 | 값 | 이유 |
|---|---|---|
| `use_white` / `use_yellow` | false / true | 경계선이 노랑. 흰색을 켜면 출발선 체커·글자를 물고 트랙 밖으로 나간다 (시뮬 확인) |
| `yellow_hsv_lo` | [15, 45, 130] | Isaac 렌더 실측: 노란 선 H 18–26, S 40–120 / 매트 S < 40. **실차는 다시 잴 것** |
| `process_width` + `birdeye_dst_ratio` | 480 + 0.3–0.7 | 차로 폭 192 px (원본과 같음) 그대로 좌우 시야만 ±0.65 m |
| `birdeye_src_ratio` | [0.0] = 자동 | `camera_height` 0.175, `camera_pitch_deg` 0, `camera_hfov_deg` 71 로 계산. 실차는 재서 넣기 |
| `bev_near_m` / `bev_far_m` | 0.37 / 0.75 | 1.0 m 까지 보면 코너(R≈0.85) 안쪽 선을 밟는다 (look-ahead 가 길어 파고듦) |
| `kp` / `ki` / `kd` | 0.6 / 0 / 0.15 | 원본 그대로 |
| `angular_scale` | 3.0 | PID 1 → 0.3 m/s 에서 3 rad/s. 감속하면 각속도도 비례해 줄어 곡률 유지 (원본 서보 조향과 같은 의미) |
| `curve_steer_bias` | 0.5 | 원본 곡률 feed-forward (원본 기본 0). 0~0.35 는 왼쪽 위 코너에서 2차로로 못 꺾었다 |
| `linear_speed` / `slow_speed` / `curve_slow` | 0.3 / 0.2 / 0.5 | 규정집 권고 0.3–0.4, 서행 0.2 이하 |
| `lost_stop_s` | 20 | 차선 소실 시 직전 조향 유지(원본) 후 정지. 3차선 가운데 차로는 노란 선이 안 보이는 4 m |

### `autorace_track/config/track_build.yaml` (트랙 USD 빌드)

| 키 | 기본값 | 설명 |
|---|---|---|
| `mat.color` | `#36342e` | 매트 색 (참고 그림에서 추출) |
| `paint.colors.white / yellow / black` | `#dfdaca` / `#e5aa43` / `#111111` | |
| `paint.class_color` | 경계선·지그재그 = yellow, 출발선 띠 = black, 나머지 white | |
| `paint.height_m` | 0.001 | 노면표시를 매트 위로 띄우는 높이 |
| `physics.*` | 0.8 / 0.7 / 0 | USD 에 박히는 바닥 마찰 기본값 (sim 실행 시 params_sim 값으로 덮어씀) |

## 트랙 다시 만들기

도구는 일반 venv 에서 돈다 (`src/autorace_track/tools/requirements.txt`, Isaac 내장 USD 24.x 에 맞춰 `usd-core==24.5`).

```bash
python3 -m venv ~/venvs/autorace && ~/venvs/autorace/bin/pip install -r src/autorace_track/tools/requirements.txt
cd src/autorace_track
~/venvs/autorace/bin/python tools/dxf_classify.py      # DXF → data/generated/track_elements.json + track_classification.png
~/venvs/autorace/bin/python tools/build_track_usd.py   # JSON → usd/autorace_track.usda
~/isaacsim/python.sh tools/check_track_isaac.py        # 미리보기 렌더 + 바닥 마찰 적용 확인
```

## 구동 모델 (sim)

LIMO 4륜 스키드 구동은 축거/윤거 = 0.20/0.175 = 1.14 > 1 이다. PhysX 의 굳은 쿨롱 마찰에서는 이런 차량이
옆 미끄러짐 저항을 이기지 못해, 바퀴를 기구학대로 돌려도 제자리 회전이 명령의 약 20 %, 주행 중 회전은 약 5 % 만 나왔다.
실차는 타이어 변형 덕분에 돈다. 그래서 `drive.model: tire` 는 바퀴 충돌체를 마찰 0 (하중만 받음)으로 두고,
바퀴 접지점마다 미끄럼 속도에 비례하는 종·횡력을 계산해 μ·N 마찰원으로 잘라 차체에 건다. 여기에 몸체 속도
되먹임(PI)을 얹어 `/cmd_vel` 을 따라간다.

| 명령 (v, w) | 실측 정상상태 (tire + tracking, μ 0.8) |
|---|---|
| (0, 1.0) | (0.000, 1.001) |
| (0, 2.0) | (0.000, 2.003) |
| (0.3, 0.5) | (0.301, 0.502) |
| (0.3, 1.0) | (0.301, 1.002) |
| 직진 0.3 m/s × 3 s | 0.882 m (가속 포함) |

## sim 과 실차의 알려진 차이

- `/odom` 은 시뮬레이터 참값이다. 실차 바퀴 오도메트리는 스키드 미끄러짐 오차가 있다.
- 라이다가 아무것도 맞지 않은 방향은 `range_max`(8.0) 로 나온다 (실차 YDLidar 는 0). 뒤쪽 약 70° 는 차체에 가려 0.12–0.2 m 가 나온다.
- 카메라는 하나로 컬러·뎁스·포인트클라우드를 낸다 (뎁스가 컬러에 정합된 상태). 실차 Dabai 뎁스는 640×400, HFOV 67.9°.
- 라이다 장착 위치: URDF 는 (0.103, 0, −0.034), 실차 `limo_base` 의 `tf_pub` 노드는 (0.1, 0, 0.05). 기본값은 URDF. 실차 측정값으로 `sensor_mounts` 를 고칠 것.
- `mode:=real` 은 이 PC 에 실차 드라이버가 없어 실행 검증을 못 했다.
