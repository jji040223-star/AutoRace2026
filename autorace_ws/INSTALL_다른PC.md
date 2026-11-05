# 다른 PC 에서 autorace_ws 실행하기

압축(`autorace_ws_B_*.zip`)에는 소스만 들어 있다 (build/install/log 는 빼고 새로 빌드). 실행법·파라미터 전체는 압축 안 `README_AutoRace2026_LIMO.md`.
트랙 USD(`src/autorace_track/usd/autorace_track.usda`)는 이미 들어 있어서 도면 도구 없이 바로 실행된다.

## 0. 필요 조건

| 항목 | 값 |
|---|---|
| OS | Ubuntu 22.04 |
| GPU | NVIDIA RTX (VRAM 8 GB 이상 권장) + 드라이버 |
| Isaac Sim | **5.1** — 기본 위치 `~/isaacsim` (다르면 4번) |
| ROS2 | Humble (`ros-humble-desktop`) |
| 인터넷 | 첫 실행 때 LIMO 모델(64 MB)을 NVIDIA 에셋 서버에서 받음 (없으면 8번) |

Isaac Sim 5.1 을 새로 깔았다면 `~/isaacsim/isaac-sim.sh` 로 한 번 띄워 창이 뜨는지 먼저 확인 (첫 실행은 셰이더 컴파일로 몇 분 걸림).

## 1. 압축 풀기

```bash
cd ~
unzip ~/Desktop/autorace_ws_B_*.zip        # → ~/autorace_ws
```

## 2. 의존 패키지 (처음 한 번)

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
    ros-humble-teleop-twist-keyboard python3-matplotlib python3-yaml python3-numpy
sudo rosdep init 2>/dev/null; rosdep update
source /opt/ros/humble/setup.bash
cd ~/autorace_ws && rosdep install --from-paths src --ignore-src -y
```

미션 YOLO (M2 갈림길 표지판, M1 신호등 — `yolo_detector`) 는 ultralytics 가 필요 (원래 PC 8.4.153, 모델 `.pt` 는 압축에 들어 있음):

```bash
pip3 install --user ultralytics==8.4.153
```

## 3. 빌드

```bash
cd ~/autorace_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 4. 셸 설정 (`~/.bashrc` 에 한 번 추가)

```bash
cat >> ~/.bashrc <<'EOF'
source /opt/ros/humble/setup.bash
source ~/autorace_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=30          # 원래 PC 값. 같은 네트워크의 다른 ROS 와 겹치지 않게, 쓰는 터미널끼리 같은 값
EOF
source ~/.bashrc
```

Isaac Sim 이 `~/isaacsim` 이 아니면 둘 중 하나:

```bash
ln -s /실제/isaac-sim/경로 ~/isaacsim                          # 방법 1: 링크
# 방법 2: src/autorace_bringup/config/params_sim.yaml 의 isaacsim_dir 수정
```

## 5. 실행

```bash
ros2 launch autorace_bringup bringup.launch.py mode:=sim                 # Isaac 창 포함 (첫 실행 1~2분)
ros2 launch autorace_bringup bringup.launch.py mode:=sim headless:=true  # 창 없이
```

## 6. 점검 (다른 터미널)

```bash
ros2 run autorace_sim smoke_test               # 토픽 9개 + TF
ros2 run autorace_sim smoke_test --drive       # 로봇이 움직임: 직진 3초 → 제자리 회전 3초
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

정상이면 `/odom /imu /scan /camera/color/image_raw /camera/depth/image_raw ...` 가 모두 0 Hz 가 아니다.

## 7. (선택) 트랙을 도면에서 다시 만들 때만

```bash
python3 -m venv ~/venvs/autorace
~/venvs/autorace/bin/pip install -r ~/autorace_ws/src/autorace_track/tools/requirements.txt
cd ~/autorace_ws/src/autorace_track
~/venvs/autorace/bin/python tools/dxf_classify.py      # 분류 + data/generated/track_classification.png
~/venvs/autorace/bin/python tools/build_track_usd.py   # usd/autorace_track.usda
```

## 8. (선택) 인터넷이 없거나 에셋 서버가 막혔을 때

인터넷 되는 곳에서 LIMO 모델 두 파일을 받아 같은 구조로 옮긴 뒤:

```bash
B=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/AgilexRobotics/limo
mkdir -p ~/isaac_assets/limo/configuration
curl -o ~/isaac_assets/limo/limo.usd $B/limo.usd
curl -o ~/isaac_assets/limo/configuration/limo_robot_schema.usd $B/configuration/limo_robot_schema.usd
```

`src/autorace_bringup/config/params_sim.yaml` → `isaac_sim.ros__parameters.robot.usd: "~/isaac_assets/limo/limo.usd"`

## 문제 해결

| 증상 | 해결 |
|---|---|
| `Package 'autorace_sim' not found` | 그 터미널에서 `source ~/autorace_ws/install/setup.bash` (4번) |
| Isaac 이 안 뜨고 `python.sh: No such file` | Isaac 경로 (4번) |
| `import rclpy` 에러 (Isaac 로그) | `run_isaac.sh` 를 거치지 않고 Isaac python 을 직접 실행한 경우 — launch 로 실행 |
| 토픽이 다른 터미널에서 안 보임 | 두 터미널의 `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` 이 같은지 |
| LIMO 가 안 나옴 (로그에 limo.usd 에러) | 인터넷 / 8번 |
| Ctrl+C 후 Isaac 이 남아 있음 | `pkill -f run_sim.py` |

실차(`mode:=real`)는 LIMO 위에서 `limo_base`, `ydlidar_ros2_driver`, `astra_camera` 가 설치돼 있어야 한다 (없으면 경고만 내고 건너뜀).
자세한 설명·파라미터 표: `README.md`, 개발 메모: `CLAUDE.md`.
